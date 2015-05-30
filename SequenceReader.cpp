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
#include <algorithm>
#include <string>
#include <vector>
#include <windows.h>
#include "glog/logging.h"

#include "MyTools.hxx"
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

const int HEIGHT = 240;
const int WIDTH = 320;
const int FILENUM = 137;

PointCloud<PointXYZRGB> cloud;
PointCloud<PointXYZRGB>::Ptr cloudptr(&cloud);
boost::shared_ptr<visualization::PCLVisualizer> viewer (new visualization::PCLVisualizer ("3D Viewer"));
boost::shared_ptr<visualization::RangeImageVisualizer > rviewer (new visualization::RangeImageVisualizer  ("range Viewer"));
boost::shared_ptr<RangeImagePlanar> planar (new RangeImagePlanar());

void featurefromHandInfo(int &fin_num, SK::Array<Vector3> &pc_tips, SK::Array<Vector3> &pc_dirs, SK::Array<Vector3> &pc_plam)
{
	// We have to get HandInfo data first!!!
	fin_num = 5;
	float tips[5][3] = {{21.0352, 343.565, 79.2866}, {-16.2998, 346.089, 89.6488}, {66.7937, 329.816, 40.3869}, {-60.8799, 340.17, 68.8904}, {-93.4081, 336.154, -22.1646}};
	float dirs[5][3] = {{3.94614, -7.9243, -99.6074}, {1.69563, 6.57322, -99.7693}, {-70.7422, 17.9615, -68.3588}, {23.2782, 8.0039, -96.923}, {66.3913, 23.308, -71.0558}};
	int order[5] = {4, 3, 1, 0, 2};
	pc_tips.resize(5);
	pc_dirs.resize(5);
	for(int i = 0; i < 5; i++)
	{
		pc_tips[i] = Vector3(tips[order[i]][0], tips[order[i]][1], tips[order[i]][2]);
		pc_dirs[i] = Vector3(dirs[order[i]][0], dirs[order[i]][1], dirs[order[i]][2]);

	}
	pc_plam.resize(2);
	pc_plam[0] = Vector3(-1.64406, 357.054, -32.2172);
	pc_plam[0] = Vector3(4.56499, 258.474, -47.8225);

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
	
	// Point cloud Loading and random sampling
	viewer->addPointCloud(cloudptr, "mycloud");

	// For loop for computing
	for(int curr_data = 0; curr_data < FILENUM; curr_data++)
	{




	}
	cout << "all frame computation done" << endl;

	// While loop for display
	int frame = 0;
	while(!viewer->wasStopped())
	{
		// Load the point cloud
		char *name = "Sequences/Seq_test1/pcd_seq", *type = ".pcd";
		stringstream ss;
		ss << name << frame << type;
		if (io::loadPCDFile<PointXYZRGB>(ss.str(), *cloudptr) == -1) // load the file
		{
			PCL_ERROR ("Couldn't read file");
			break;
		}
		cout << "Loaded " << cloudptr->width * cloudptr->height << " data points from " << ss.str() << endl;

		viewer->updatePointCloud(cloudptr, "mycloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "mycloud");
		viewer->spinOnce(100);
//		rviewer->spinOnce(100);
		frame = (frame + 1) % FILENUM;
	}

	return 0;
}