#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <EasiiSDK/Iisu.h>

#include <math.h> 
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <string>
#include <vector>
#include <windows.h>
#include "glog/logging.h"

#include "MyTools.hxx"
#include "DataDriven.hxx"
#include "Sphere.hxx"
#include "HandInfo.hxx"
#include "HandModel.hxx"
#include "HandPose.hxx"
#include "PSO.hxx"
#include "AICP.hxx"
#include "Initialization.hxx"


using namespace std;
using namespace pcl;
using namespace SK;
using namespace SK::Easii;

// for testing
const bool opti = true;
const bool skel = true;
const bool show = true;

const int HEIGHT = 240;
const int WIDTH = 320;

const char *name = "Sequences/Seq_test4/pcd_seq";
const char *type = ".pcd";
const int FILENUM = 58;

PointCloud<PointXYZRGB> cloud;
PointCloud<PointXYZRGB>::Ptr cloudptr(&cloud);
boost::shared_ptr<visualization::PCLVisualizer> viewer (new visualization::PCLVisualizer ("3D Viewer"));
boost::shared_ptr<visualization::RangeImageVisualizer > rviewer (new visualization::RangeImageVisualizer  ("Range Viewer"));
boost::shared_ptr<RangeImagePlanar> planar (new RangeImagePlanar());

void featurefromHandInfo(int &fin_num, SK::Array<Vector3> &pc_tips, SK::Array<Vector3> &pc_dirs, SK::Array<Vector3> &pc_palm)
{
	// We have to get HandInfo data first!!!
	fin_num = 5;
	float tips[5][3] = {{48.2747, 277.028, 41.7511}, {4.00454, 283.424, 81.4256}, {-32.2563, 285.37, 90.0489}, {-107.844, 279.245, -17.0972}, {-75.5988, 281.608, 70.2936}};
	float dirs[5][3] = {{-54.7152, 21.2096, -80.9716}, {-22.6027, 5.18344, -97.2741}, {15.6026, 12.9124, -97.9277}, {88.1873, 19.9759, -42.7078}, {54.9201, 30.5877, -77.77}};
	int order[5] = {3, 4, 2, 1, 0};
	pc_tips.resize(5);
	pc_dirs.resize(5);
	for(int i = 0; i < 5; i++)
	{
		pc_tips[i] = Vector3(tips[order[i]][0], tips[order[i]][1], tips[order[i]][2]);
		pc_dirs[i] = Vector3(dirs[order[i]][0], dirs[order[i]][1], dirs[order[i]][2]);

	}
	pc_palm.resize(2);
	pc_palm[0] = Vector3(-20.546, 305.868, -28.0197);
	pc_palm[1] = Vector3(-7.59871, 209.616, -51.8505);

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

void setDepthImage()
{
	int image_x = 320, image_y = 240;
	float center_x = 160.0, center_y = 120.0;
	float focal_x = 224.502, focal_y = 230.494;
	Affine3f sensorpose = Affine3f(Eigen::Translation3f(-14.4617, -171.208, 5.5311) * AngleAxisf(-0.5 * M_PI, Vector3f::UnitX()));
	planar->createFromPointCloudWithFixedSize(*cloudptr, image_x, image_y, center_x, center_y, focal_x, focal_y, sensorpose);
}


int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);
	
	// Viewer initialization
	viewer->setBackgroundColor (0, 0, 0);
	viewer->initCameraParameters();
	viewer->addCoordinateSystem (1.0);

/*	rviewer->setSize(WIDTH, HEIGHT);
	rviewer->setPosition(650, 300);*/

	// Camera initialization
	vector<visualization::Camera> camera;
	viewer->setCameraPosition(-14.4617, -171.208, 6.5311, 0, 0, 1);
	
	// Dataset initialization
	int file_num, length;
	cout << "choose one trail and pc space size: ";
	cin >> file_num >> length;
	DataDriven data_driven = (file_num >= 0 && file_num < 17)? DataDriven(file_num) : DataDriven();
	SK::Array<SK::Array<float>> dataset = data_driven.getDataSet();
	cout << "loading dataset done, set size = " << data_driven.getDataSetSize() << endl;
	if(length < 0 || length > 6)	data_driven.startPCA(false);
	else							data_driven.startPCA(length, false);
	MatrixXf transmat = data_driven.getTransMatrix();
	cout << endl << setprecision(3) << "trans matrix: " << endl << transmat << endl;
	
	// Point cloud Loading and random sampling (not yet now)
	viewer->addPointCloud(cloudptr, "mycloud");

	// Hand Model & Pose initialization
	HandModel handmodel = HandModel();
	HandPose poselist[FILENUM];

	if(show)	addHandModel(handmodel, skel);
	if(show && skel)	addHandSkeleton(handmodel);

	// For loop for computing
	for(int curr_data = 0; curr_data < FILENUM; curr_data++)
	{
		// Load the point cloud
		stringstream ss;
		ss << name << curr_data << type;
		if (io::loadPCDFile<PointXYZRGB>(ss.str(), *cloudptr) == -1) // load the file
		{
			PCL_ERROR ("Couldn't read file");
			break;
		}
		cout << "In computing, loaded " << cloudptr->width * cloudptr->height << " data points from " << ss.str() << endl;

		// Time stamp
		clock_t start = clock();

		// Set depth image
		setDepthImage();

		// Reset the Pose
		handmodel = HandModel();
		if(curr_data > 0)	poselist[curr_data] = poselist[curr_data - 1];
		
		if(curr_data == 0)	// Initialization
		{
			// get data from HandInfo(fake)
			int finger_num;
			SK::Array<Vector3> pc_tips, pc_dirs, pc_palm;
			featurefromHandInfo(finger_num, pc_tips, pc_dirs, pc_palm);

			poselist[curr_data] = HandPose();
			SK::Array<SK::Array<bool>> permlist = MyTools::fingerChoosing(finger_num);
			double bestcost = 1000000;
			for(size_t i = 0; i < permlist.size(); i++)
			{
				Initialization in = Initialization(finger_num, pc_tips, pc_dirs, handmodel);
				poselist[curr_data].setPosition(pc_palm[0]);
				poselist[curr_data].setOrientation(pc_palm[1] - pc_palm[0]);
				in.fingerExist(permlist[i]);
				in.goFullInitail(poselist[curr_data], false);
				if(bestcost > in.getCost())
				{
					bestcost = in.getCost();
					in.setFullResultPose(poselist[curr_data]);
				}
			}
			poselist[curr_data].applyPose(handmodel);/**/
			cout << "Initialization done" << endl;
		}
		
		// PSO Optimization
		PSO pso = PSO(20, 24, 4, 1);
		if(curr_data > 0)
			pso.generateParticles(poselist[curr_data - 1]);
		else
			pso.generateParticles(poselist[curr_data]);
		pso.goGeneration_datafull(cloud, *planar.get(), handmodel, data_driven, false, false);
//		pso.goGeneration_full(cloud, *planar.get(), handmodel, false, false);
		HandPose bestpose = pso.getBestPose();
		bestpose.applyPose(handmodel);
		poselist[curr_data] = bestpose;/**/
		cout << "best cost = " << pso.getBestPoint() << endl;

		// Time stamp
		clock_t end = clock();
		cout << "time comsumption in frame " << curr_data << ", time = " << double(end - start) << " ms\n";

	}
	cout << "all frame computation done" << endl;

	// While loop for display
	int frame = 0;
	while(!viewer->wasStopped())
	{
		// Load the point cloud
		stringstream ss;
		ss << name << frame << type;
		if (io::loadPCDFile<PointXYZRGB>(ss.str(), *cloudptr) == -1) // load the file
		{
			PCL_ERROR ("Couldn't read file");
			break;
		}
		cout << "In viewing, loaded " << cloudptr->width * cloudptr->height << " data points from " << ss.str() << endl;

		// Update hand model
		handmodel = HandModel();
		poselist[frame].applyPose(handmodel);
		if(show)	updateHandModel(handmodel, skel);
		if(show && skel)	updateHandSkeleton(handmodel);


		// Update viewer
		viewer->updatePointCloud(cloudptr, "mycloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "mycloud");
		viewer->spinOnce(100);
//		rviewer->spinOnce(100);
		frame = (frame + 1) % FILENUM;
	}

	return 0;
}