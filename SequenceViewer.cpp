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
#include <fstream>
#include <sstream>
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

// Processing & show
const bool opti = true;
const bool skel = false;
bool show = true;
bool reshow = false;

const int HEIGHT = 240;
const int WIDTH = 320;

// Data cames from
const char *posname = "PoseData/Seq_mov9_ICPPSO_ICPimp_4.txt";
const char *infoname = "InfoData/info_seq_mov9.txt";
const char *seqname = "Sequences/Seq_mov9/pcd_seq";
const char *type = ".pcd";
const int FILENUM = 29;

// camera pose
double camera_front[] = {-14.4617, -171.208, 6.5311, 0, 0, 1};
double camera_right[] = {662, -224, 369, -0.5, 0, 1};
double camera_left[] = {-722, -10.8, -322, -0.5, 0, 1};

PointCloud<PointXYZRGB> cloud;
PointCloud<PointXYZRGB>::Ptr cloudptr(&cloud);
boost::shared_ptr<visualization::PCLVisualizer> viewer (new visualization::PCLVisualizer ("3D Viewer"));
boost::shared_ptr<visualization::RangeImageVisualizer > rviewer (new visualization::RangeImageVisualizer  ("Range Viewer"));
boost::shared_ptr<RangeImagePlanar> planar (new RangeImagePlanar());

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

// change camera view point
void chCameraViewPoint (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	boost::shared_ptr<visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<visualization::PCLVisualizer> *> (viewer_void);
	if (event.getKeySym () == "a" && event.keyDown ())
		viewer->setCameraPosition(camera_left[0], camera_left[1], camera_left[2], camera_left[3], camera_left[4], camera_left[5]);
	if (event.getKeySym () == "s" && event.keyDown ())
		viewer->setCameraPosition(camera_front[0], camera_front[1], camera_front[2], camera_front[3], camera_front[4], camera_front[5]);
	if (event.getKeySym () == "d" && event.keyDown ())
		viewer->setCameraPosition(camera_right[0], camera_right[1], camera_right[2], camera_right[3], camera_right[4], camera_right[5]);
}

void showModel(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	boost::shared_ptr<visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<visualization::PCLVisualizer> *> (viewer_void);
	if (event.getKeySym () == "m" && event.keyDown ())
	{
		if(show == true)
		{
			viewer->removeAllShapes();
			show = false;
			reshow = false;
		}
		else
		{
			show = true;
			reshow = true;
		}
	}
}

int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);
	
	// Viewer initialization
	viewer->setBackgroundColor (0, 0, 0);
	viewer->initCameraParameters();
	viewer->addCoordinateSystem (1.0);
	viewer->registerKeyboardCallback(chCameraViewPoint, (void*)&viewer);
	viewer->registerKeyboardCallback(showModel, (void*)&viewer);
	viewer->addText("Info...", 0, 450, "info_text");

/*	rviewer->setSize(WIDTH, HEIGHT);
	rviewer->setPosition(650, 300);*/

	// Camera initialization
	viewer->setCameraPosition(-14.4617, -171.208, 6.5311, 0, 0, 1);
	
	// File initialization
	fstream file;
	file.open(posname, ios::in);
	if(!file)
	{
		cout << "cannot open file " << posname << "!!!" << endl;
		return -1;
	}
	
	// Point cloud Loading and random sampling (not yet now)
	viewer->addPointCloud(cloudptr, "mycloud");

	// Hand Model & Pose initialization
	HandModel handmodel = HandModel();
	HandPose poselist[FILENUM];

	if(opti && show)	addHandModel(handmodel, skel);
	if(opti && show && skel)	addHandSkeleton(handmodel);

	// Load parameter from PoseData
	SK::Array<float> pose_para;
	pose_para.resize(26);
	SK::Vector3 ori_para = SK::Vector3();
	int count = 0;
	while(file >> pose_para[0] >> pose_para[1] >> pose_para[2] >> pose_para[3] >> pose_para[4] >> pose_para[5] >> pose_para[6] >> pose_para[7]
			   >> pose_para[8] >> pose_para[9] >> pose_para[10] >> pose_para[11] >> pose_para[12] >> pose_para[13] >> pose_para[14] >> pose_para[15]
			   >> pose_para[16] >> pose_para[17] >> pose_para[18] >> pose_para[19] >> pose_para[20] >> pose_para[21] >> pose_para[22] >> pose_para[23]
			   >> pose_para[24] >> pose_para[25] >> ori_para[0] >> ori_para[1] >> ori_para[2])
	{
		poselist[count].setAllParameters(pose_para);
		poselist[count].setOrientation(ori_para);
		count++;
	}

	file.close();

	// While loop for display
	int frame = 0;
	while(!viewer->wasStopped())
	{
		// Load the point cloud
		stringstream ss;
		ss << seqname << frame << type;
		if (io::loadPCDFile<PointXYZRGB>(ss.str(), *cloudptr) == -1) // load the file
		{
			PCL_ERROR ("Couldn't read file");
			break;
		}
//		cout << "In viewing, loaded " << cloudptr->width * cloudptr->height << " data points from " << ss.str() << endl;
		stringstream infoss;
		infoss << "In viewing, loaded " << cloudptr->width * cloudptr->height << " data points from " << ss.str() << endl;
		
		// set up planar
		setDepthImage();

		// reshow the model?
		if(reshow)
		{
			viewer->addText("Info...", 0, 450, "info_text");
			if(opti && show)	addHandModel(handmodel, skel);
			if(opti && show && skel)	addHandSkeleton(handmodel);
			reshow = false;
		}

		// Update hand model
		if(opti)
		{
			handmodel = HandModel();
			poselist[frame].applyPose(handmodel);
			if(show)	updateHandModel(handmodel, skel);
			if(show && skel)	updateHandSkeleton(handmodel);

			// Compute costs
			vector<float> pure_vec;
			float pix_meter;
			Index<flann::L2<float>> index = MyTools::buildDatasetIndex(*planar.get(), pure_vec, pix_meter);
			MyCostFunction costf = MyCostFunction(cloud, handmodel, *planar.get(), pix_meter, pure_vec);
			if(frame > 1)	costf.calculate(index, poselist[frame - 1], poselist[frame - 2]);
			else		costf.calculate(index);
			infoss << "cost in D-term = " << costf.getDTerm() << ", F-term = " << costf.getFTerm() << ", L-term = " << costf.getLTerm() << ", M-term = " << costf.getMTerm() << endl;
			infoss << "total cost = " << costf.getCost() << endl;

		}

		// Update viewer
		viewer->updateText(infoss.str(), 0, 450, "info_text");
		viewer->updatePointCloud(cloudptr, "mycloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "mycloud");
		viewer->spinOnce(50);
//		rviewer->spinOnce(50);
		frame = (frame + 1) % FILENUM;
	}

	return 0;
}