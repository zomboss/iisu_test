#include <pcl/visualization/cloud_viewer.h>
#include <EasiiSDK/Iisu.h>
#include <iomanip>
#include <iostream>
#include <fstream>
#include "mat.h"
#include "MyTools.hxx"
#include "HandPose.hxx"
#include "HandModel.hxx"
#include "DataDriven.hxx"

using namespace std;
using namespace SK;
using namespace SK::Easii;
using namespace Eigen;

// for testing
const bool skel = false;

boost::shared_ptr<visualization::PCLVisualizer> viewer (new visualization::PCLVisualizer ("3D Viewer"));

void addHandModel(HandModel &handmodel, bool isdot)
{
	SK::Array<Sphere> sgroup = handmodel.getFullHand();
	if(isdot)
	{
		int jointspot[] = {12, 18, 20, 21, 13, 24, 26, 27, 14, 30, 32, 33, 15, 36, 38, 39, 40, 44, 46, 47};
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
		int jointspot[] = {12, 18, 20, 21, 13, 24, 26, 27, 14, 30, 32, 33, 15, 36, 38, 39, 40, 44, 46, 47};
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

int main(int argc, char **argv)
{

	// Viewer initialization
	viewer->setBackgroundColor (0, 0, 0);
	viewer->initCameraParameters();
	viewer->addCoordinateSystem (1.0);
	
	// get dataset
	int file_num;
	cout << "choose one trail: ";
	cin >> file_num;
	DataDriven data_driven = (file_num >= 0 && file_num < 17)? DataDriven(file_num) : DataDriven();
	SK::Array<SK::Array<float>> dataset = data_driven.getDataSet();
	cout << "loading dataset done, set size = " << data_driven.getDataSetSize() << endl;
	data_driven.startPCA(false);
//	data_driven.startPCA(10, false);
	MatrixXf transmat = data_driven.getTransMatrix();
	cout << endl << setprecision(3) << "trans matrix: " << endl << transmat << endl;
	cout << endl << "min vector: " << endl << data_driven.getMinVector() << endl;
	cout << endl << "max vector: " << endl << data_driven.getMaxVector() << endl;

	HandModel handmodel = HandModel();
	addHandModel(handmodel, skel);
	if(skel)	addHandSkeleton(handmodel);

	//test: transformT
/*	SK::Array<float> zero_para;
	for(int i = 0; i < 26; i++)	zero_para.pushBack(0.0f);
	SK::Array<Eigen::Matrix<float, 3, 1>> transT = handmodel.transformT(zero_para);
	for(size_t i = 0; i < transT.size(); i++)
		cout << "points " << (i + 1) << ": (" << transT[i].x() << ", "  << transT[i].y() << ", " << transT[i].z() << ")\n";
	cout << endl;*/

	// Main while
	int frame = 0;
	while(!viewer->wasStopped())
	{
//		cout << "frame = " << frame << endl;
		// Hand Model & pose initialization
		handmodel = HandModel();
		HandPose handpose = HandPose();

		// Apply the new parameters handpose (Animation)
		SK::Array<float> curr_para = dataset[frame];
/*		handpose.setAllParameters(curr_para, false);
		handpose.applyPose(handmodel);*/

		// Apply the new parameters by transformT (Animation)
		Eigen::Matrix<float, 1, 26> curr_mat;
		for(int i = 0; i < 26; i++)
			curr_mat(0, i) = curr_para[i];
		Eigen::Matrix<float, 3, SPHERE_NUM> transT = handmodel.transformT(curr_mat);
		handmodel.setAllCenter(transT);

		updateHandModel(handmodel, skel);
		if(skel)	updateHandSkeleton(handmodel);
		
		viewer->spinOnce(10);
		frame = (frame + 1) % dataset.size();
	}

	return 0;

}