#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/range_image/range_image_planar.h>
#include <EasiiSDK/Iisu.h>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "glog/logging.h"
#include "MyTools.hxx"
#include "HandPose.hxx"
#include "HandModel.hxx"
#include "DataDriven.hxx"
#include "PSO.hxx"
#include "AICP.hxx"
#include "ICPPCA.hxx"
#include "Initialization.hxx"

using namespace std;
using namespace SK;
using namespace SK::Easii;
using namespace Eigen;

const int HEIGHT = 240;
const int WIDTH = 320;
// for testing
const bool skel = true;

PointCloud<PointXYZRGB> cloud;
PointCloud<PointXYZRGB>::Ptr cloudptr(&cloud);
boost::shared_ptr<visualization::PCLVisualizer> viewer (new visualization::PCLVisualizer ("3D Viewer"));
boost::shared_ptr<visualization::RangeImageVisualizer > rviewer (new visualization::RangeImageVisualizer  ("range Viewer"));
boost::shared_ptr<RangeImagePlanar> planar (new RangeImagePlanar());

Vector3 getTipVector(int finger)
{
	float data[5][3] = {{31, 225, -32}, {-2, 235, -20}, {58, 214, -59}, {-56, 196, -98}, {-42, 231, -38}};
	return Vector3(data[finger][0], data[finger][1], data[finger][2]);
}

SK::Array<Vector3> getFingerDirection(int finger)
{
	float str[5][3] = {{31, 225, -32}, {-2, 235, -20}, {58, 214, -59}, {-56, 196, -98}, {-42, 231, -38}};
	float end[5][3] = {{28, 277, -117}, {18, 280, -107}, {61, 249, -152}, {3, 255, -153}, {-16, 273, -125}};
	SK::Array<Vector3> direction;
	direction.pushBack(Vector3(str[finger][0], str[finger][1], str[finger][2]));
	direction.pushBack(Vector3(end[finger][0], end[finger][1], end[finger][2]));
	return direction;
}

SK::Array<Vector3> getPalmDirection()
{
	SK::Array<Vector3> direction;
	direction.pushBack(Vector3(16, 241, -123));
	direction.pushBack(Vector3(31, 142, -118));
	return direction;
}

Vector3 getFingerColor(int finger)
{
	// 0: red, 1: green, 2: blue, 3: yello, 4: pink
	int color[5][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {255, 0, 255}};
	return Vector3(color[finger][0], color[finger][1], color[finger][2]);
}

void addNPCFeature(SK::Array<Vector3> &pc_tips, SK::Array<Vector3> &pc_dirs, SK::Array<int> &match) 
{
	SK::Array<Vector3> palm = getPalmDirection();

	viewer->addArrow(palm[0], palm[1], 200, 200, 200, false, "plam orientation");
	pc_tips.resize(5);
	pc_dirs.resize(5);
	for(int f = 0; f < 5; f++)
	{
		Vector3 tips = getTipVector(f);
		Vector3 color = getFingerColor(f);
		SK::Array<Vector3> fdir = getFingerDirection(f);
		// no finger condition
		if(tips == Vector3(0, 0, 0))	continue;

		cout  << match[f] << " fingers, tips = " << tips << ", ori = " << (fdir[1] - fdir[0]) << endl;
		pc_tips[match[f]] = tips;
		pc_dirs[match[f]] = (fdir[1] - fdir[0]);

		string sname = "tips ";
		sname = sname + to_string(static_cast<long long>(match[f]));
		string dname = "finger direction ";
		dname = dname + to_string(static_cast<long long>(match[f]));
		viewer->addSphere(tips, 5.0, (int)color[0], (int)color[1], (int)color[2], sname);
		viewer->addArrow(fdir[0], fdir[1], 200, 200, 200, false, dname);
	}
}

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

	if (io::loadPCDFile<PointXYZRGB> ("data/test_pcd_3.pcd", *cloudptr) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file test_pcd_3.pcd \n");
		return -1;
	}
	cout << "Loaded " << cloudptr->width * cloudptr->height << " data points from test_pcd_3.pcd." <<  endl;

	// Viewer initialization
	viewer->setBackgroundColor (0, 0, 0);
	viewer->initCameraParameters();
	viewer->addCoordinateSystem (1.0);

	
	rviewer->setSize(WIDTH, HEIGHT);
	rviewer->setPosition(650, 300);

	// Camera initialization
	vector<visualization::Camera> camera;
	viewer->setCameraPosition(-14.4617, -171.208, 6.5311, 0, 0, 1);

	viewer->addPointCloud(cloudptr, "mycloud");
	
	// get dataset
	int file_num, length;
	cout << "choose one trail and pc space size: ";
	cin >> file_num >> length;
	DataDriven data_driven = (file_num >= 0 && file_num < 17)? DataDriven(file_num) : DataDriven();
	SK::Array<SK::Array<float>> dataset = data_driven.getDataSet();
	cout << "loading dataset done, set size = " << data_driven.getDataSetSize() << endl;
	if(length < 0)	data_driven.startPCA(false);
	else	data_driven.startPCA(length, false);
	MatrixXf transmat = data_driven.getTransMatrix();
	cout << endl << setprecision(3) << "trans matrix: " << endl << transmat << endl;

	// Get tips and plam normal (hand feature)
	SK::Array<Vector3> pc_tips, pc_dirs;
	SK::Array<int> finger_match;			// Special case!!!
	finger_match.pushBack(3);finger_match.pushBack(2);finger_match.pushBack(4);finger_match.pushBack(0);finger_match.pushBack(1);
	addNPCFeature(pc_tips, pc_dirs, finger_match);/**/

	// Hand Model & pose initialization
	HandModel handmodel = HandModel();
	HandPose handpose = HandPose();
	
	SK::Array<SK::Array<bool>> permlist = MyTools::fingerChoosing(5);
	double bestcost = 1000000;
	for(size_t i = 0; i < permlist.size(); i++)
	{
		Initialization in = Initialization(5, pc_tips, pc_dirs, handmodel);
		in.fingerExist(permlist[0]);
		// set global position & orientation
		handpose.setPosition(getPalmDirection()[0]);
		handpose.setOrientation(getPalmDirection()[1] - getPalmDirection()[0]);

		in.goFullInitail(handpose, false);
		if(bestcost > in.getCost())
		{
			bestcost = in.getCost();
			in.setFullResultPose(handpose);
		}
	}/**/
/*	handpose.applyPose(handmodel);*/

	// ICP Optimization
	clock_t sect_1 = clock();
	AICP aicp = AICP(10, 5, handpose);
	aicp.run_randomPara(cloud);
	HandPose bestpose1 = aicp.getBestPose();
	cout << "AICP done" << endl;/**/

	// PCA trans testing
/*	int index = 450;
	cout << "enter index: ";
	cin >> index;
	handpose.setAllParameters(data_driven.getSpecData(index), false);*/

	cout << "current length = " << data_driven.getPCSpaceSize() << endl;
	// ICPPCA Optimization
	clock_t sect_2 = clock();
	ICPPCA icppca = ICPPCA(4, 5, data_driven.getPCSpaceSize(), handpose, data_driven.getTransMatrix(), 
						   data_driven.getMeanVector(), data_driven.getMaxVector(), data_driven.getMinVector());
	icppca.run(cloud);
	HandPose bestpose2 = icppca.getBestPose();
	cout << "ICPPCA done" << endl;/**/

	// PCA trans testing
/*	HandPose tmppose = icppca.pureTrans(length);
	showParameter(handpose, tmppose);*/
	clock_t end = clock();
	cout << "section 1 = " << double(sect_2 - sect_1) << " ms, section 2 = " << double(end - sect_2) << " ms\n";

	// Show the model
	addHandModel(handmodel, skel);
	if(skel)	addHandSkeleton(handmodel);

	// Main while
	int frame = 0;
	while(!viewer->wasStopped())
	{
		handmodel = HandModel();
		if(frame % 3 == 0)		handpose.applyPose(handmodel);
		else if(frame % 3 == 1)	bestpose1.applyPose(handmodel);
		else					bestpose2.applyPose(handmodel);
/*		if(frame % 2 == 0)		handpose.applyPose(handmodel);
		else if(frame % 2 == 1)	tmppose.applyPose(handmodel);*/

		updateHandModel(handmodel, skel);
		if(skel)	updateHandSkeleton(handmodel);

//		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "mycloud");
		viewer->spinOnce(100);
		rviewer->spinOnce(100);
		frame++;
	}

	return 0;

}