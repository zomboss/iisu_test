#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <EasiiSDK/Iisu.h>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include "glog/logging.h"
#include "MyTools.hxx"
#include "HandPose.hxx"
#include "HandModel.hxx"
#include "DataDriven.hxx"
#include "ICPPCA.hxx"
#include "Initialization.hxx"

using namespace std;
using namespace SK;
using namespace SK::Easii;
using namespace Eigen;
using namespace cv;

const int HEIGHT = 240;
const int WIDTH = 320;
// for testing
const bool skel = true;

PointCloud<PointXYZRGB> cloud;
PointCloud<PointXYZRGB>::Ptr cloudptr(&cloud);
boost::shared_ptr<visualization::PCLVisualizer> viewer (new visualization::PCLVisualizer ("3D Viewer"));

void addHandModel(HandModel &handmodel, bool isdot)
{
	SK::Array<Sphere> sgroup = handmodel.getFullHand();
	if(isdot)
	{
		int jointspot[] = {16, 18, 20, 21, 22, 24, 26, 27, 28, 30, 32, 33, 34, 36, 38, 39, 40, 44, 46, 47};
		for(size_t i = 0; i < 20; i++)
			viewer->addSphere(MyTools::vectortoPointXYZ(sgroup[jointspot[i]].getCenter()), 3, 
							  sgroup[jointspot[i]].getColor()[0], sgroup[jointspot[i]].getColor()[1], sgroup[jointspot[i]].getColor()[2], 
							  sgroup[jointspot[i]].getName());
	}
	else
		for(size_t i = 0; i < sgroup.size(); i++)
			viewer->addSphere(MyTools::vectortoPointXYZ(sgroup[i].getCenter()), sgroup[i].getRadius(), sgroup[i].getColor()[0], sgroup[i].getColor()[1], sgroup[i].getColor()[2], sgroup[i].getName());
}

void updateHandModel(HandModel &handmodel, bool isdot)
{
	SK::Array<Sphere> sgroup = handmodel.getFullHand();
	if(isdot)
	{
		int jointspot[] = {16, 18, 20, 21, 22, 24, 26, 27, 28, 30, 32, 33, 34, 36, 38, 39, 40, 44, 46, 47};
		for(size_t i = 0; i < 20; i++)
			viewer->updateSphere(MyTools::vectortoPointXYZ(sgroup[jointspot[i]].getCenter()), 3, 
								 sgroup[jointspot[i]].getColor()[0], sgroup[jointspot[i]].getColor()[1], sgroup[jointspot[i]].getColor()[2], 
								 sgroup[jointspot[i]].getName());
	}
	else
		for(size_t i = 0; i < sgroup.size(); i++)
			viewer->updateSphere(MyTools::vectortoPointXYZ(sgroup[i].getCenter()), sgroup[i].getRadius(), sgroup[i].getColor()[0], sgroup[i].getColor()[1], sgroup[i].getColor()[2], sgroup[i].getName());

}

void addHandSkeleton(HandModel &handmodel)
{
	SK::Array<ModelCoefficients> coeff = handmodel.getSkeleton();
	for(size_t i = 0; i < coeff.size(); i++)
	{
		string name = "cylinder ";
		name = name + to_string(static_cast<long long>(i));
		viewer->addCylinder(coeff[i], name);
	}
}

void updateHandSkeleton(HandModel &handmodel)
{
	SK::Array<ModelCoefficients> coeff = handmodel.getSkeleton();
	for(size_t i = 0; i < coeff.size(); i++)
	{
		string name = "cylinder ";
		name = name + to_string(static_cast<long long>(i));
		viewer->removeShape(name);
		viewer->addCylinder(coeff[i], name);
	}
}

void showParameter(HandPose &strpose, HandPose &endpose)
{
	cout << "checking parameter..." << endl;
	cout << "thumb, origin:\t" << strpose.getThumbPose()[0] << ", " << strpose.getThumbPose()[1] << ", " << strpose.getThumbPose()[2] << endl
		 << "after opt:\t" << endpose.getThumbPose()[0] << ", " << endpose.getThumbPose()[1] << ", " << endpose.getThumbPose()[2] << endl
		 << "index, origin:\t" << strpose.getIndexPose()[0] << ", " << strpose.getIndexPose()[1] << ", " << strpose.getIndexPose()[2] << endl
		 << "after opt:\t" << endpose.getIndexPose()[0] << ", " << endpose.getIndexPose()[1] << ", " << endpose.getIndexPose()[2] << endl
		 << "middle, origin:\t" << strpose.getMiddlePose()[0] << ", " << strpose.getMiddlePose()[1] << ", " << strpose.getMiddlePose()[2] << endl
		 << "after opt:\t" << endpose.getMiddlePose()[0] << ", " << endpose.getMiddlePose()[1] << ", " << endpose.getMiddlePose()[2] << endl
		 << "ring, origin:\t" << strpose.getRingPose()[0] << ", " << strpose.getRingPose()[1] << ", " << strpose.getRingPose()[2] << endl
		 << "after opt:\t" << endpose.getRingPose()[0] << ", " << endpose.getRingPose()[1] << ", " << endpose.getRingPose()[2] << endl
		 << "little, origin:\t" << strpose.getLittlePose()[0] << ", " << strpose.getLittlePose()[1] << ", " << strpose.getLittlePose()[2] << endl
		 << "after opt:\t" << endpose.getLittlePose()[0] << ", " << endpose.getLittlePose()[1] << ", " << endpose.getLittlePose()[2] << endl
		 << "position, origin:\t" << strpose.getPosition() << endl
		 << "after opt:\t" << endpose.getPosition() << endl
		 << "rotation, origin:\t" << strpose.getRotation() << endl
		 << "after opt:\t" << endpose.getRotation() << endl
		 << "orientation, origin:\t" << strpose.getOrientation() << endl
		 << "after opt:\t" << endpose.getOrientation() << endl;
}

void showParameter(HandPose &pose)
{
	cout << "checking parameter..." << endl;
	cout << "thumb:\t" << pose.getThumbPose()[0] << ", " << pose.getThumbPose()[1] << ", " << pose.getThumbPose()[2] << endl
		 << "index:\t" << pose.getIndexPose()[0] << ", " << pose.getIndexPose()[1] << ", " << pose.getIndexPose()[2] << endl
		 << "middle:\t" << pose.getMiddlePose()[0] << ", " << pose.getMiddlePose()[1] << ", " << pose.getMiddlePose()[2] << endl
		 << "ring, origin:\t" << pose.getRingPose()[0] << ", " << pose.getRingPose()[1] << ", " << pose.getRingPose()[2] << endl
		 << "little, origin:\t" << pose.getLittlePose()[0] << ", " << pose.getLittlePose()[1] << ", " << pose.getLittlePose()[2] << endl
		 << "position, origin:\t" << pose.getPosition() << endl
		 << "rotation, origin:\t" << pose.getRotation() << endl
		 << "orientation, origin:\t" << pose.getOrientation() << endl;
}

int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);

	// Viewer initialization
	viewer->setBackgroundColor (0, 0, 0);
	viewer->initCameraParameters();
	viewer->addCoordinateSystem (1.0);
	viewer->addText("frame: ", 0, 480, "frame_num");
	
	// get dataset
	int file_num, length;
	cout << "choose one trail and pc space size: ";
	cin >> file_num >> length;
	DataDriven data_driven = (file_num >= 0 && file_num < 17)? DataDriven(file_num) : DataDriven();
	SK::Array<SK::Array<float>> dataset = data_driven.getDataSet();
	cout << "loading dataset done, set size = " << data_driven.getDataSetSize() << endl;
	if(length < 0)	data_driven.startPCA(true);
	else	data_driven.startPCA(length, true);
	MatrixXf transmat = data_driven.getTransMatrix();
	cout << endl << setprecision(3) << "trans matrix: " << endl << transmat << endl;

	// Hand Model & pose initialization
	HandModel handmodel = HandModel();
	HandPose handpose = HandPose();
	
	// PCA trans testing
	int index = 450;
	cout << "enter index: ";
	cin >> index;
	handpose.setAllParameters(data_driven.getSpecData(index), false);/**/

	cout << "current length = " << data_driven.getPCSpaceSize() << endl;
	// ICPPCA Optimization
	clock_t sect_2 = clock();
	ICPPCA icppca = ICPPCA(4, 5, data_driven.getPCSpaceSize(), handpose, data_driven.getTransMatrix(), 
						   data_driven.getMeanVector(), data_driven.getMaxVector(), data_driven.getMinVector());

	// new part!!!
	Mat mat_o;
	Eigen::Matrix<float, 20, -1> mat_e = data_driven.getDataMatrix();
	cv::eigen2cv(mat_e, mat_o);
/*	for(int i = 0; i < mat_o.rows; i++)
	{
		for(int j = 0; j < mat_o.cols; j++)
		{
			if(mat_o.at<float>(i, j) - mat_e(i, j) != 0)	cout << "fucking wrong in (" << i << ", " << j <<")!!!" << endl;
		}

	}*/
	int cvlen = (length < 0) ? data_driven.getPCSpaceSize() : length;
	cv::PCA cvpca = cv::PCA(mat_o, Mat(), 1/*PCA::DATA_AS_COL*/, cvlen);
	
	Eigen::Matrix<float, 26, 1> curr_para = MyTools::SKtoEigenVector(data_driven.getSpecData(index));
	Eigen::Matrix<float, 20, 1> local_para = curr_para.block(0, 0, 20, 1);
	Mat para;
	cv::eigen2cv(local_para, para);
	cout << "para size = (" << para.rows << ", " << para.cols << ")\n";
	Mat coeff;
	cvpca.project(para, coeff);
	cout << "coeff size = (" << coeff.rows << ", " << coeff.cols << ")\n";
	Mat recov;
	cvpca.backProject(coeff, recov);
	cout << "recov size = (" << recov.rows << ", " << recov.cols << ")\n";
	Eigen::Matrix<float, 20, 1> aft_mat;
	cv::cv2eigen(recov, aft_mat);
	SK::Array<float> aft_para = MyTools::EigentoSKVector((Eigen::VectorXf)aft_mat);
	for(int i = 0; i < 6; i++)
		aft_para.pushBack(curr_para[20 + i]);
	HandPose cvpose = HandPose();
	cvpose.setAllParameters(aft_para, false);

	
	// PCA trans testing
	HandPose tmppose = icppca.pureTrans(length);
	showParameter(handpose, tmppose);/**/

	// Show the model
	addHandModel(handmodel, skel);
	if(skel)	addHandSkeleton(handmodel);

	// Main while
	int frame = 0;
	while(!viewer->wasStopped())
	{
		handmodel = HandModel();

		if(frame % 3 == 0)		handpose.applyPose(handmodel);
		else if(frame % 3 == 1)	cvpose.applyPose(handmodel);
		else if(frame % 3 == 2)	tmppose.applyPose(handmodel);

		updateHandModel(handmodel, skel);
		if(skel)	updateHandSkeleton(handmodel);

		stringstream ss;
		ss << "frame: " << frame << endl;
		viewer->updateText(ss.str(), 0, 480, "frame_num");
		viewer->spinOnce(100);
		frame++;
	}

	return 0;

}