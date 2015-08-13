/**
Main Program
Purpose:
	(1)	The core program to handle hand pose tracking
	(2)	Record pose data to PoseData directory
Usage:
<before compile>
	(1)	Modify the variable seqname to select the sequence data
	(2) Modify the variable infoname to select the info data (the sequence has to be consistent with the sequence data)
	(3)	Modify the variable posename to select the pose output you want to save (recommend store in PoseData directory)
	(4)	Modify FILENUM to the number of sequence
<in runtime>
	(1) When crashing, re-run the program
	(2)	After processing, the operation is same as inSequenceViewer.cpp
*/

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
const bool skel = true;
const bool show = true;

const int HEIGHT = 240;
const int WIDTH = 320;

// Data cames from
const char *posname = "PoseData/Seq_mov9_PSOonly_2.txt";
const char *infoname = "InfoData/info_seq_mov9.txt";
const char *seqname = "Sequences/Seq_mov9/pcd_seq";
const char *type = ".pcd";
const int FILENUM = 66;

// camera pose
double camera_front[] = {-14.4617, -171.208, 6.5311, 0, 0, 1};
double camera_right[] = {662, -224, 369, -0.5, 0, 1};
double camera_left[] = {-722, -10.8, -322, -0.5, 0, 1};

PointCloud<PointXYZRGB> cloud;
PointCloud<PointXYZRGB>::Ptr cloudptr(&cloud);
boost::shared_ptr<visualization::PCLVisualizer> viewer (new visualization::PCLVisualizer ("3D Viewer"));
//boost::shared_ptr<visualization::RangeImageVisualizer > rviewer (new visualization::RangeImageVisualizer  ("Range Viewer"));
boost::shared_ptr<RangeImagePlanar> planar (new RangeImagePlanar());

int posefromPoseData(SK::Array<HandPose> &arr)
{
	// Load pose file if needed
	fstream file_pose_in;
	file_pose_in.open(posname, ios::in);
	if(!file_pose_in)
	{
		cout << "cannot open file " << posname << "!!!" << endl;
		return 0;
	}

	// Load parameter from PoseData
	SK::Array<float> pose_para;
	pose_para.resize(26);
	SK::Vector3 ori_para = SK::Vector3();
	int count = 0;
	while( file_pose_in >> pose_para[0] >> pose_para[1] >> pose_para[2] >> pose_para[3] >> pose_para[4] >> pose_para[5] >> pose_para[6] >> pose_para[7]
						>> pose_para[8] >> pose_para[9] >> pose_para[10] >> pose_para[11] >> pose_para[12] >> pose_para[13] >> pose_para[14] >> pose_para[15]
						>> pose_para[16] >> pose_para[17] >> pose_para[18] >> pose_para[19] >> pose_para[20] >> pose_para[21] >> pose_para[22] >> pose_para[23]
						>> pose_para[24] >> pose_para[25] >> ori_para[0] >> ori_para[1] >> ori_para[2])
	{
		HandPose tmppose = HandPose();
		tmppose.setAllParameters(pose_para);
		tmppose.setOrientation(ori_para);
		arr.pushBack(tmppose);

		count++;
	}

	return count;
}

void featurefromHandInfo(int frame, int &fin_num, SK::Array<Vector3> &pc_tips, SK::Array<Vector3> &pc_dirs, SK::Array<Vector3> &pc_palm)
{
	// Get Info from data
	fstream file;
	file.open(infoname, ios::in);
	if(!file)
	{
		cout << "cannot open file " << infoname << "!!!" << endl;
		return;
	}
	
	float tips[5][3];
	float dirs[5][3];
	float palm[2][3];
	int order[5];
	string line;
	int count = 0;
	while(getline(file, line))
	{
		// find the correct data
		if(count < frame * 5)
		{
			count++;
			continue;
		}

		istringstream iss(line);
		switch(count % 5)
		{
		case 0:
			if (!(iss >> fin_num))
			{ 
				cout << "error occur in line " << count << "!!!" << endl;
				return;
			}
			break;
		case 1:
			if (!(iss >> tips[0][0] >> tips[0][1] >> tips[0][2] >> tips[1][0] >> tips[1][1] >> tips[1][2]
					  >> tips[2][0] >> tips[2][1] >> tips[2][2] >> tips[3][0] >> tips[3][1] >> tips[3][2]
					  >> tips[4][0] >> tips[4][1] >> tips[4][2]))
			{ 
				cout << "error occur in line " << count << "!!!" << endl;
				return;
			}
			break;
		case 2:
			if (!(iss >> dirs[0][0] >> dirs[0][1] >> dirs[0][2] >> dirs[1][0] >> dirs[1][1] >> dirs[1][2]
					  >> dirs[2][0] >> dirs[2][1] >> dirs[2][2] >> dirs[3][0] >> dirs[3][1] >> dirs[3][2]
					  >> dirs[4][0] >> dirs[4][1] >> dirs[4][2]))
			{ 
				cout << "error occur in line " << count << "!!!" << endl;
				return;
			}
			break;
		case 3:
			if (!(iss >> palm[0][0] >> palm[0][1] >> palm[0][2] >> palm[1][0] >> palm[1][1] >> palm[1][2]))
			{ 
				cout << "error occur in line " << count << "!!!" << endl;
				return;
			}
			break;
		case 4:
			if (!(iss >> order[0] >> order[1] >> order[2] >> order[3] >> order[4]))
			{ 
				cout << "error occur in line " << count << "!!!" << endl;
				return;
			}
			break;
		}
		
		count++;
		if(count >= (frame + 1) * 5)	break;
	}

/*	cout << "In fun, num = " << fin_num << endl;
	cout << "In fun, tips = ";
	for(int i = 0; i < 5; i++)
		for(int j = 0; j < 3; j++)
			cout << tips[i][j] << " ";
	cout << endl << "In fun, dirs = ";
	for(int i = 0; i < 5; i++)
		for(int j = 0; j < 3; j++)
			cout << dirs[i][j] << " ";
	cout << endl << "In fun, palm = ";
	for(int i = 0; i < 2; i++)
		for(int j = 0; j < 3; j++)
			cout << palm[i][j] << " ";
	cout << endl << "In fun, order = ";
	for(int i = 0; i < 5; i++)
			cout << order[i] << " ";
	cout << endl;*/

	pc_tips.resize(5);
	pc_dirs.resize(5);
	for(int i = 0; i < 5; i++)
	{
		pc_tips[i] = Vector3(tips[order[i]][0], tips[order[i]][1], tips[order[i]][2]);
		pc_dirs[i] = Vector3(dirs[order[i]][0], dirs[order[i]][1], dirs[order[i]][2]);

	}
	pc_palm.resize(2);
	pc_palm[0] = Vector3(palm[0][0], palm[0][1], palm[0][2]);
	pc_palm[1] = Vector3(palm[1][0], palm[1][1], palm[1][2]);
	
	file.close();
}

SK::Array<int> getFingerNumfromHandInfo()
{
	SK::Array<int> num_list;
	num_list.resize(FILENUM);
	
	fstream file;
	file.open(infoname, ios::in);
	if(!file)
	{
		cout << "cannot open file " << infoname << "!!!" << endl;
		return num_list;
	}
	
	string line;
	int count = 0;
	while(getline(file, line))
	{
		if(count % 5 == 0)
		{
			istringstream iss(line);
			if (!(iss >> num_list[count / 5]))
				cout << "error occur in line " << count << "!!!" << endl;
		}
		count++;
		if(count >= FILENUM * 5)	break;	// For sake
	}

	file.close();
	return num_list;
}

void getPalmCenterfromHandInfo(int frame, Eigen::Matrix<float, 3, 1> &curr, Eigen::Matrix<float, 3, 1> &prev)
{
	fstream file;
	file.open(infoname, ios::in);
	if(!file)
	{
		cout << "cannot open file " << infoname << "!!!" << endl;
		return ;
	}
	
	string line;
	int count = 0;
	while(getline(file, line))
	{
		istringstream iss(line);
		float useless[3];
		
		// find the previous data
		if(count == ((frame - 1) * 5 + 3))
		{
			if (!(iss >> prev(0, 0) >> prev(1, 0) >> prev(2, 0) >> useless[0] >> useless[1] >> useless[2]))
				cout << "error occur in line " << count << "!!!" << endl;
		}
		
		// find the current data
		if(count == (frame * 5 + 3))
		{	
			if (!(iss >> curr(0, 0) >> curr(1, 0) >> curr(2, 0) >> useless[0] >> useless[1] >> useless[2]))
				cout << "error occur in line " << count << "!!!" << endl;
			if(frame == 0)	prev = curr;
			break;
		}

		count++;
	}

}

bool initChoosing(int frame, double *costlist, SK::Array<int> numlist)
{
	if(frame == 0) return true;
	return (costlist[frame - 1] > costlist[0] * 2.0) && (numlist[frame] == 5 || numlist[frame] == 0);
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
//	cout << "size of coeff = " << coeff.size() << endl;
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

int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);
	
	// Viewer initialization
	viewer->setBackgroundColor (0, 0, 0);
	viewer->initCameraParameters();
	viewer->addCoordinateSystem (1.0);
	viewer->addText("info...", 0, 480, "info_text");
	viewer->registerKeyboardCallback(chCameraViewPoint, (void*)&viewer);

	// Camera initialization
	viewer->setCameraPosition(-14.4617, -171.208, 6.5311, 0, 0, 1);
	
	// File initialization
	fstream file_pose_out;
	file_pose_out.open(posname, ios::app);
	if(!file_pose_out)
	{
		cout << "cannot open file " << posname << "!!!" << endl;
		return -1;
	}
	
	// Point cloud Loading and random sampling (not yet now)
	viewer->addPointCloud(cloudptr, "mycloud");

	// Hand Model & Pose initialization
	HandModel handmodel = HandModel();
	HandPose poselist[FILENUM];
	SK::Array<HandPose> testlist;

	// best point list
	double costlist[FILENUM];
	SK::Array<int> numlist = getFingerNumfromHandInfo();

	SK::Array<HandPose> posearray;
	int str_point = posefromPoseData(posearray);
	cout << "start at " << str_point << endl;
	cout << "show data: " << endl;
	for(int i = 0; i < str_point; i++)
		cout << posearray[i].getAllParametersT(1.0f) << endl;/**/
	for(size_t i = 0; i < posearray.size(); i++)
		poselist[i] = posearray[i];

	// For loop for computing
	for(int curr_data = str_point; curr_data < FILENUM; curr_data++)
	{
		if(!opti)	break;
		
		// Load the point cloud
		stringstream ss;
		ss << seqname << curr_data << type;
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
		
		if(curr_data == 0 /*|| initChoosing(curr_data, costlist, numlist)*/)	// Initialization
		{
			// get data from HandInfo
			int finger_num;
			SK::Array<Vector3> pc_tips, pc_dirs, pc_palm;
			featurefromHandInfo(curr_data, finger_num, pc_tips, pc_dirs, pc_palm);

			// testing...
			/*cout << "finger_num = " << finger_num << endl;
			cout << "pc_tips: ";
			for(int i = 0; i < 5; i++)
				cout << "(" << pc_tips[i][0] << ", " << pc_tips[i][1] << ", " << pc_tips[i][2] << ") ";
			cout << endl << "pc_dirs: ";
			for(int i = 0; i < 5; i++)
				cout << "(" << pc_dirs[i][0] << ", " << pc_dirs[i][1] << ", " << pc_dirs[i][2] << ") ";
			cout << endl << "palm: (" << pc_palm[0][0] << ", " << pc_palm[0][1] << ", " << pc_palm[0][2] << ") -> (" 
									  << pc_palm[1][0] << ", " << pc_palm[1][1] << ", " << pc_palm[1][2] << ")\n";*/

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

/*			poselist[curr_data].applyPose(handmodel);*/
			cout << "Initialization done" << endl;
		}
		
		// Get the center to define is moving or not
		Eigen::Matrix<float, 3, 1> prev_center, curr_center;
		getPalmCenterfromHandInfo(curr_data, curr_center, prev_center);
		cout << "show previous center: (" << prev_center.transpose() << "), current center: (" << curr_center.transpose() << ")\n";

		// PSO Optimization
		PSO pso = PSO(30, 30, -5, 1);
		if(curr_data > 1)
			pso.setPrevPose(poselist[curr_data - 1], poselist[curr_data - 2]);/**/
		if(curr_data > 0)
			pso.generateParticles(poselist[curr_data - 1]);
		else
			pso.generateParticles(poselist[curr_data]);
		pso.checkMoving(curr_center, prev_center);
		cout << "show is moving: " << pso.getIsMov() << endl;
//		pso.goGeneration_loop(cloud, *planar.get(), handmodel, false, false);
		pso.goGeneration_triv(cloud, *planar.get(), handmodel, false, false);
//		pso.goGeneration_test(cloud, *planar.get(), handmodel, false, false);
//		cout << "testlist size = " << testlist.size() << endl;
		HandPose bestpose = pso.getBestPose();
		bestpose.applyPose(handmodel);
//		showParameter(poselist[curr_data], bestpose);
		poselist[curr_data] = bestpose;/**/
		costlist[curr_data] = pso.getBestPoint();
		cout << "best cost = " << costlist[curr_data] << endl;

		// Time stamp
		clock_t end = clock();
		cout << "time comsumption in frame " << curr_data << ", time = " << double(end - start) << " ms\n";

		// write pose into data
		SK::Array<float> pose_para = poselist[curr_data].getAllParameters();
		pose_para.pushBack(poselist[curr_data].getOrientation()[0]);
		pose_para.pushBack(poselist[curr_data].getOrientation()[1]);
		pose_para.pushBack(poselist[curr_data].getOrientation()[2]);
		for(int i = 0; i < 29; i++)
			file_pose_out << pose_para[i] << " ";
		file_pose_out << endl;

	}
	cout << "all frame computation done" << endl;
	file_pose_out.close();

	// While loop for display
	if(opti && show)	addHandModel(handmodel, skel);
	if(opti && show && skel)	addHandSkeleton(handmodel);

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

		// Show info on screen
		stringstream info;
		info << "In viewing, loaded " << cloudptr->width * cloudptr->height << " data points from " << ss.str() << endl;
		info << "frame " << frame << ", cost = " << costlist[frame] << endl;
		viewer->updateText(info.str(), 0, 480, "info_text");

		// Update hand model
		if(opti)
		{
			handmodel = HandModel();
			poselist[frame].applyPose(handmodel);
//			testlist[frame].applyPose(handmodel);
			if(show)	updateHandModel(handmodel, skel);
			if(show && skel)	updateHandSkeleton(handmodel);
		}


		// Update viewer
		viewer->updatePointCloud(cloudptr, "mycloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "mycloud");
		viewer->spinOnce(50);
		frame = (frame + 1) % FILENUM;
	}

	return 0;
}