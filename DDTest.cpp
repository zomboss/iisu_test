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
#include "Initialization.hxx"

using namespace std;
using namespace SK;
using namespace SK::Easii;
using namespace Eigen;

const int HEIGHT = 240;
const int WIDTH = 320;
// for testing
const bool skel = false;

PointCloud<PointXYZRGB> cloud;
PointCloud<PointXYZRGB>::Ptr cloudptr(&cloud);
boost::shared_ptr<visualization::PCLVisualizer> viewer (new visualization::PCLVisualizer ("3D Viewer"));
boost::shared_ptr<visualization::RangeImageVisualizer > rviewer (new visualization::RangeImageVisualizer  ("range Viewer"));
boost::shared_ptr<RangeImagePlanar> planar (new RangeImagePlanar());

Vector3 getTipVector(int finger)
{
	float data[5][3] = {{37.5714, 295.46, 77.926}, {-21.4612, 350.523, 80.8923}, {56.3304, 265.788, 35.05}, {-53.3599, 377.658, -5.33599}, {12.1443, 322.319, 91.0819}};
	return Vector3(data[finger][0], data[finger][1], data[finger][2]);
}

SK::Array<Vector3> getFingerDirection(int finger)
{
	float str[5][3] = {{37.5714, 295.46, 77.926}, {-21.4612, 350.523, 80.8923}, {56.3304, 265.788, 35.05}, {-53.3599, 377.658, -5.33599}, {12.1443, 322.319, 91.0819}};
	float end[5][3] = {{27.4743, 319.138, -18.7044}, {-1.4965, 343.182, -16.8192}, {31.7353, 326.943, -40.1506}, {27.6647, 339.774, -50.0551}, {23.2306, 319.797, -8.26963}};
	SK::Array<Vector3> direction;
	direction.pushBack(Vector3(str[finger][0], str[finger][1], str[finger][2]));
	direction.pushBack(Vector3(end[finger][0], end[finger][1], end[finger][2]));
	return direction;
}

SK::Array<Vector3> getPalmDirection()
{
	SK::Array<Vector3> direction;
	direction.pushBack(Vector3(19.2832, 341.86, -27.9837));
	direction.pushBack(Vector3(-45.5201, 270.517, -54.6425));
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

int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);

	if (io::loadPCDFile<PointXYZRGB> ("data/test_pcd_9.pcd", *cloudptr) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file test_pcd_9.pcd \n");
		return -1;
	}
	cout << "Loaded " << cloudptr->width * cloudptr->height << " data points from test_pcd_9.pcd." <<  endl;

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
	DataDriven data_driven = DataDriven(0);
	SK::Array<SK::Array<float>> dataset = data_driven.getDataSet();
	cout << "loading dataset done, set size = " << data_driven.getDataSetSize() << endl;
	data_driven.startPCA();
//	data_driven.startPCA(10);
	MatrixXf transmat = data_driven.getTransMatrix();
	cout << endl << setprecision(3) << "trans matrix: " << endl << transmat << endl;

	// Get tips and plam normal (hand feature)
	SK::Array<Vector3> pc_tips, pc_dirs;
	SK::Array<int> finger_match;			// Special case!!!
	finger_match.pushBack(3);finger_match.pushBack(1);finger_match.pushBack(4);finger_match.pushBack(0);finger_match.pushBack(2);
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

	// ICP-PSO Optimization
	PSO pso = PSO(15, 24, -10, 1);
	pso.generateParticles(handpose);
	cout << "PSO initial done..." << endl;
	pso.goGeneration_mp(cloud, handmodel, false, false);
//	pso.goGeneration_full(cloud, *planar.get(), handmodel, false, false);
	cout << "PSO optimizaiton done..." << endl;
	HandPose bestpose = pso.getBestPose();
	bestpose.applyPose(handmodel);
	cout << "show point: " << pso.getBestPoint() << endl;/**/


	// ICPPCA Optimization








	// Show the model
	addHandModel(handmodel, skel);
	if(skel)	addHandSkeleton(handmodel);

	// Main while
	while(!viewer->wasStopped())
	{
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "mycloud");
		viewer->spinOnce(100);
		rviewer->spinOnce(100);
	}

	return 0;

}