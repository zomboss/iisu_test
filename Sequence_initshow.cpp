#include <iostream>
#include <sstream>
#include <ctime>
#include <string>
#include <math.h>
#include <omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/range_image/range_image_planar.h>
#include <flann/flann.hpp>
#include "glog/logging.h"
#include "MyTools.hxx"
#include "Sphere.hxx"
#include "HandModel.hxx"
#include "HandPose.hxx"
#include "CostFunction.hxx"
#include "PSO.hxx"
#include "AICP.hxx"
#include "Initialization.hxx"


using namespace std;
using namespace pcl;
using namespace SK;
using namespace SK::Easii;

bool isinit = false;
bool isori = true;
int str_frame = 17;
int fin_ch = -1;
bool is_upper = false;
const bool isskel = true;
bool show = false;
bool reshow = false;

const int HEIGHT = 240;
const int WIDTH = 320;

const char *posname = "PoseData/Seq_mov9_ICPPSO_onlyA_2.txt";
const char *infoname = "InfoData/info_seq_mov9.txt";
const char *seqname = "Sequences/Seq_mov9/pcd_seq";
const char *type = ".pcd";

PointCloud<PointXYZRGB> cloud;
PointCloud<PointXYZRGB>::Ptr cloudptr(&cloud);
boost::shared_ptr<visualization::PCLVisualizer> viewer (new visualization::PCLVisualizer ("3D Viewer"));
boost::shared_ptr<visualization::RangeImageVisualizer > rviewer (new visualization::RangeImageVisualizer  ("range Viewer"));
boost::shared_ptr<RangeImagePlanar> planar (new RangeImagePlanar());

int* getFingerColor(int index)
{
	int color[3] = {0, 0, 0};
	switch(index)
	{
		case 0:
			color[0] = 255;
			color[1] = 0;
			color[2] = 0;
			break;
		case 1:
			color[0] = 0;
			color[1] = 255;
			color[2] = 0;
			break;
		case 2:
			color[0] = 0;
			color[1] = 255;
			color[2] = 255;
			break;
		case 3:
			color[0] = 255;
			color[1] = 255;
			color[2] = 0;
			break;
		case 4:
			color[0] = 255;
			color[1] = 0;
			color[2] = 255;
			break;
		default:
			color[0] = 255;
			color[1] = 255;
			color[2] = 255;
			break;
	}
	return color;
}

void featurefromHandInfo(int frame, int &fin_num, SK::Array<Vector3> &pc_tips, SK::Array<Vector3> &pc_dirs, SK::Array<Vector3> &pc_palm, bool show = true)
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
		if(count > (frame + 1) * 5)	break;
	}

	pc_tips.resize(5);
	pc_dirs.resize(5);
	for(int i = 0; i < 5; i++)
	{
		pc_tips[i] = Vector3(tips[order[i]][0], tips[order[i]][1], tips[order[i]][2]);
		pc_dirs[i] = Vector3(dirs[order[i]][0], dirs[order[i]][1], dirs[order[i]][2]);

		// set features into viewer
		if(show)
		{
			int *color = getFingerColor(order[i]);
			string sname = "tips ";
			sname = sname + to_string(static_cast<long long>(order[i]));
			string dname = "finger direction ";
			dname = dname + to_string(static_cast<long long>(order[i]));
			viewer->addSphere(pc_tips[i], 5.0, (int)color[0], (int)color[1], (int)color[2], sname);
//			viewer->addArrow(pc_tips[i], (pc_tips[i] + pc_dirs[i]), (double)(color[0] / 255.0), (double)(color[1] / 255.0), (double)(color[2] / 255.0), false, dname);
		}
	}
	pc_palm.resize(2);
	pc_palm[0] = Vector3(palm[0][0], palm[0][1], palm[0][2]);
	pc_palm[1] = Vector3(palm[1][0], palm[1][1], palm[1][2]);

	// set features into viewer
	if(show)
		viewer->addArrow(pc_palm[0], pc_palm[1], 1, 1, 1, false, "plam orientation");
	
	file.close();
}

int getFingerNumfromHandInfo(int frame)
{
	int fin_num = -1;
	
	fstream file;
	file.open(infoname, ios::in);
	if(!file)
	{
		cout << "cannot open file " << infoname << "!!!" << endl;
		return fin_num;
	}
	
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
		
		if(count % 5 == 0)
		{
			istringstream iss(line);
			if (!(iss >> fin_num))
				cout << "error occur in line " << count << "!!!" << endl;
			break;
		}
	}
	
	return fin_num;
}

Eigen::Matrix<float, 3, 1> showPalmCenterList()
{
	Eigen::Matrix<float, 3, 1> info_pos;
	
	fstream file;
	file.open(infoname, ios::in);
	if(!file)
	{
		cout << "cannot open file " << infoname << "!!!" << endl;
		return info_pos;
	}
	
	string line;
	int count = 0;
	while(getline(file, line))
	{
		Eigen::Matrix<float, 3, 1> palm_pos, pre_pos;
		float useless[3];
		if(count % 5 == 3)
		{
			pre_pos = palm_pos;
			istringstream iss(line);
			if (!(iss >> palm_pos(0, 0) >> palm_pos(1, 0) >> palm_pos(2, 0) >> useless[0] >> useless[1] >> useless[2]))
				cout << "error occur in line " << count << "!!!" << endl;
/*			cout << "in frame " << (count / 5) << ", center: (" << palm_pos(0, 0) << ", " << palm_pos(1, 0) << ", " << palm_pos(2, 0) << ")";
			if(count / 5 > 0)	cout << ", distance = " << (palm_pos - pre_pos).norm() << endl;
			else				cout << endl;*/
			if(count / 5 == str_frame)	info_pos = palm_pos;
		}
		count++;
	}
	return info_pos;
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

void showDepthImage()
{
	int image_x = WIDTH, image_y = HEIGHT;
	float center_x = 160.0, center_y = 120.0;
	float focal_x = 224.502, focal_y = 230.494;
	Eigen::Affine3f sensorpose = Eigen::Affine3f(Eigen::Translation3f(-14.4617, -171.208, 5.5311) * Eigen::AngleAxisf(-0.5 * M_PI, Eigen::Vector3f::UnitX()));
	planar->createFromPointCloudWithFixedSize(*cloudptr, image_x, image_y, center_x, center_y, focal_x, focal_y, sensorpose);
	//planar->createFromPointCloudWithFixedSize(*cloudptr, image_x, image_y, center_x, center_y, focal_x, focal_y, sensorpose);
	rviewer->showRangeImage(*planar.get());
	cout << "Depth map setting done" << endl;
}

void projectDepthImage(HandModel &handmodel)
{

	for(int i = 0; i < handmodel.getSphereSize(); i++)
	{
		Eigen::Vector3f point = MyTools::SKtoEigenVector(handmodel.getFullHand()[i].getCenter());
		int image_x, image_y;
		float range;
		planar->getImagePoint(point, image_x, image_y, range);
		rviewer->addCircle(image_x, (240 - image_y), 5, handmodel.getFullHand()[i].getName());
	}
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

void loadTryPose(HandPose &trypose)
{
	switch(fin_ch)
	{
	case 0:
		if(is_upper)trypose.setFingerPose(0, SK::Vector3(0, THUMB_MCP_FE_UPPERBOUND, trypose.getThumbPose()[0][2]), trypose.getThumbPose()[1], trypose.getThumbPose()[2]);
		else		trypose.setFingerPose(0, SK::Vector3(0, THUMB_MCP_FE_LOWERBOUND, trypose.getThumbPose()[0][2]), trypose.getThumbPose()[1], trypose.getThumbPose()[2]);
		break;
	case 1:
		if(is_upper)trypose.setFingerPose(0, SK::Vector3(0, trypose.getThumbPose()[0][1], THUMB_MCP_AA_UPPERBOUND), trypose.getThumbPose()[1], trypose.getThumbPose()[2]);
		else		trypose.setFingerPose(0, SK::Vector3(0, trypose.getThumbPose()[0][1], THUMB_MCP_AA_LOWERBOUND), trypose.getThumbPose()[1], trypose.getThumbPose()[2]);
		break;
	case 2:
		if(is_upper)trypose.setFingerPose(0, trypose.getThumbPose()[0], SK::Vector3(THUMB_PIP_FE_UPPERBOUND, 0, 0), trypose.getThumbPose()[2]);
		else		trypose.setFingerPose(0, trypose.getThumbPose()[0], SK::Vector3(THUMB_PIP_FE_LOWERBOUND, 0, 0), trypose.getThumbPose()[2]);
		break;
	case 3:
		if(is_upper)trypose.setFingerPose(0, trypose.getThumbPose()[0], trypose.getThumbPose()[1], SK::Vector3(THUMB_DIP_FE_UPPERBOUND, 0, 0));
		else		trypose.setFingerPose(0, trypose.getThumbPose()[0], trypose.getThumbPose()[1], SK::Vector3(THUMB_DIP_FE_LOWERBOUND, 0, 0));
		break;
	case 4:
		if(is_upper)trypose.setFingerPose(1, SK::Vector3(INDEX_MCP_FE_UPPERBOUND, 0, trypose.getIndexPose()[0][2]), trypose.getIndexPose()[1], trypose.getIndexPose()[2]);
		else		trypose.setFingerPose(1, SK::Vector3(INDEX_MCP_FE_LOWERBOUND, 0, trypose.getIndexPose()[0][2]), trypose.getIndexPose()[1], trypose.getIndexPose()[2]);
		break;
	case 5:
		if(is_upper)trypose.setFingerPose(1, SK::Vector3(trypose.getIndexPose()[0][0], 0, INDEX_MCP_AA_UPPERBOUND), trypose.getIndexPose()[1], trypose.getIndexPose()[2]);
		else		trypose.setFingerPose(1, SK::Vector3(trypose.getIndexPose()[0][0], 0, INDEX_MCP_AA_LOWERBOUND), trypose.getIndexPose()[1], trypose.getIndexPose()[2]);
		break;
	case 6:
		if(is_upper)trypose.setFingerPose(1, trypose.getIndexPose()[0], SK::Vector3(INDEX_PIP_FE_UPPERBOUND, 0, 0), trypose.getIndexPose()[2]);
		else		trypose.setFingerPose(1, trypose.getIndexPose()[0], SK::Vector3(INDEX_PIP_FE_LOWERBOUND, 0, 0), trypose.getIndexPose()[2]);
		break;
	case 7:
		if(is_upper)trypose.setFingerPose(1, trypose.getIndexPose()[0], trypose.getIndexPose()[1], SK::Vector3(INDEX_DIP_FE_UPPERBOUND, 0, 0));
		else		trypose.setFingerPose(1, trypose.getIndexPose()[0], trypose.getIndexPose()[1], SK::Vector3(INDEX_DIP_FE_LOWERBOUND, 0, 0));
		break;
	case 8:
		if(is_upper)trypose.setFingerPose(2, SK::Vector3(MIDDLE_MCP_FE_UPPERBOUND, 0, trypose.getMiddlePose()[0][2]), trypose.getMiddlePose()[1], trypose.getMiddlePose()[2]);
		else		trypose.setFingerPose(2, SK::Vector3(MIDDLE_MCP_FE_LOWERBOUND, 0, trypose.getMiddlePose()[0][2]), trypose.getMiddlePose()[1], trypose.getMiddlePose()[2]);
		break;
	case 9:
		if(is_upper)trypose.setFingerPose(2, SK::Vector3(trypose.getMiddlePose()[0][0], 0, MIDDLE_MCP_AA_UPPERBOUND), trypose.getMiddlePose()[1], trypose.getMiddlePose()[2]);
		else		trypose.setFingerPose(2, SK::Vector3(trypose.getMiddlePose()[0][0], 0, MIDDLE_MCP_AA_LOWERBOUND), trypose.getMiddlePose()[1], trypose.getMiddlePose()[2]);
		break;
	case 10:
		if(is_upper)trypose.setFingerPose(2, trypose.getMiddlePose()[0], SK::Vector3(MIDDLE_PIP_FE_UPPERBOUND, 0, 0), trypose.getMiddlePose()[2]);
		else		trypose.setFingerPose(2, trypose.getMiddlePose()[0], SK::Vector3(MIDDLE_PIP_FE_LOWERBOUND, 0, 0), trypose.getMiddlePose()[2]);
		break;
	case 11:
		if(is_upper)trypose.setFingerPose(2, trypose.getMiddlePose()[0], trypose.getMiddlePose()[1], SK::Vector3(MIDDLE_DIP_FE_UPPERBOUND, 0, 0));
		else		trypose.setFingerPose(2, trypose.getMiddlePose()[0], trypose.getMiddlePose()[1], SK::Vector3(MIDDLE_DIP_FE_LOWERBOUND, 0, 0));
		break;
	case 12:
		if(is_upper)trypose.setFingerPose(3, SK::Vector3(RING_MCP_FE_UPPERBOUND, 0, trypose.getRingPose()[0][2]), trypose.getRingPose()[1], trypose.getRingPose()[2]);
		else		trypose.setFingerPose(3, SK::Vector3(RING_MCP_FE_LOWERBOUND, 0, trypose.getRingPose()[0][2]), trypose.getRingPose()[1], trypose.getRingPose()[2]);
		break;
	case 13:
		if(is_upper)trypose.setFingerPose(3, SK::Vector3(trypose.getRingPose()[0][0], 0, RING_MCP_AA_UPPERBOUND), trypose.getRingPose()[1], trypose.getRingPose()[2]);
		else		trypose.setFingerPose(3, SK::Vector3(trypose.getRingPose()[0][0], 0, RING_MCP_AA_LOWERBOUND), trypose.getRingPose()[1], trypose.getRingPose()[2]);
		break;
	case 14:
		if(is_upper)trypose.setFingerPose(3, trypose.getRingPose()[0], SK::Vector3(RING_PIP_FE_UPPERBOUND, 0, 0), trypose.getRingPose()[2]);
		else		trypose.setFingerPose(3, trypose.getRingPose()[0], SK::Vector3(RING_PIP_FE_LOWERBOUND, 0, 0), trypose.getRingPose()[2]);
		break;
	case 15:
		if(is_upper)trypose.setFingerPose(3, trypose.getRingPose()[0], trypose.getRingPose()[1], SK::Vector3(RING_DIP_FE_UPPERBOUND, 0, 0));
		else		trypose.setFingerPose(3, trypose.getRingPose()[0], trypose.getRingPose()[1], SK::Vector3(RING_DIP_FE_LOWERBOUND, 0, 0));
		break;
	case 16:
		if(is_upper)trypose.setFingerPose(4, SK::Vector3(LITTLE_MCP_FE_UPPERBOUND, 0, trypose.getLittlePose()[0][2]), trypose.getLittlePose()[1], trypose.getLittlePose()[2]);
		else		trypose.setFingerPose(4, SK::Vector3(LITTLE_MCP_FE_LOWERBOUND, 0, trypose.getLittlePose()[0][2]), trypose.getLittlePose()[1], trypose.getLittlePose()[2]);
		break;
	case 17:
		if(is_upper)trypose.setFingerPose(4, SK::Vector3(trypose.getLittlePose()[0][0], 0, LITTLE_MCP_AA_UPPERBOUND), trypose.getLittlePose()[1], trypose.getLittlePose()[2]);
		else		trypose.setFingerPose(4, SK::Vector3(trypose.getLittlePose()[0][0], 0, LITTLE_MCP_AA_LOWERBOUND), trypose.getLittlePose()[1], trypose.getLittlePose()[2]);
		break;
	case 18:
		if(is_upper)trypose.setFingerPose(4, trypose.getLittlePose()[0], SK::Vector3(LITTLE_PIP_FE_UPPERBOUND, 0, 0), trypose.getLittlePose()[2]);
		else		trypose.setFingerPose(4, trypose.getLittlePose()[0], SK::Vector3(LITTLE_PIP_FE_LOWERBOUND, 0, 0), trypose.getLittlePose()[2]);
		break;
	case 19:
		if(is_upper)trypose.setFingerPose(4, trypose.getLittlePose()[0], trypose.getLittlePose()[1], SK::Vector3(LITTLE_DIP_FE_UPPERBOUND, 0, 0));
		else		trypose.setFingerPose(4, trypose.getLittlePose()[0], trypose.getLittlePose()[1], SK::Vector3(LITTLE_DIP_FE_LOWERBOUND, 0, 0));
		break;
	default:
		// do nothing
		break;
	}

}

int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);
	
	// Viewer initialization
	viewer->setBackgroundColor (0, 0, 0);
	viewer->initCameraParameters();
	viewer->addCoordinateSystem (1.0);
	viewer->addText("case", 0, 450, "num_text");
	viewer->registerKeyboardCallback(showModel, (void*)&viewer);

	rviewer->setSize(WIDTH, HEIGHT);
	rviewer->setPosition(650, 300);

	// Camera initialization
	vector<visualization::Camera> camera;
	viewer->setCameraPosition(-14.4617, -171.208, 6.5311, 0, 0, 1);

	cout << "need initialization? ";
	cin >> isinit;
	cout << "enter frame: ";
	cin >> str_frame;
	cout << "show origin? ";
	cin >> isori;

	// testing...
	Eigen::Matrix<float, 3, 1> info_pos = showPalmCenterList();
	cout << "show info_pos = " << info_pos.transpose() << endl;

	// Load the point cloud
	stringstream ss;
	ss << seqname << str_frame << type;
	if (io::loadPCDFile<PointXYZRGB>(ss.str(), *cloudptr) == -1) // load the file
	{
		PCL_ERROR ("Couldn't read file");
		return -1;
	}

	// Point cloud Loading and random sampling
//	cloud = MyTools::downsampling(cloud);
	viewer->addPointCloud(cloudptr, "mycloud");

	// Depth map setting
	showDepthImage();

	// Hand Model & hand pose initialization
	HandModel handmodel = HandModel();
	HandPose handpose = HandPose();
	HandPose initpose = HandPose();
	HandPose prev_pose1 = HandPose();
	HandPose prev_pose2 = HandPose();

	// Load the pose data
	fstream pos_file;
	pos_file.open(posname, ios::in);
	if(!pos_file)
	{
		cout << "cannot open file " << posname << "!!!" << endl;
		return -1;
	}
		
	// Load previous parameter from PoseData
	SK::Array<float> pose_para;
	pose_para.resize(26);
	SK::Vector3 ori_para = SK::Vector3();
	int count = 0;
	while( pos_file >> pose_para[0] >> pose_para[1] >> pose_para[2] >> pose_para[3] >> pose_para[4] >> pose_para[5] >> pose_para[6] >> pose_para[7]
					>> pose_para[8] >> pose_para[9] >> pose_para[10] >> pose_para[11] >> pose_para[12] >> pose_para[13] >> pose_para[14] >> pose_para[15]
					>> pose_para[16] >> pose_para[17] >> pose_para[18] >> pose_para[19] >> pose_para[20] >> pose_para[21] >> pose_para[22] >> pose_para[23]
					>> pose_para[24] >> pose_para[25] >> ori_para[0] >> ori_para[1] >> ori_para[2] )
	{
		if(str_frame > 1)
		{
			if(count == (str_frame - 2))
			{
				prev_pose2.setAllParameters(pose_para);
				prev_pose2.setOrientation(ori_para);
			}
			if(count == (str_frame - 1))
			{
				prev_pose1.setAllParameters(pose_para);
				prev_pose1.setOrientation(ori_para);
				handpose.setAllParameters(pose_para);
				handpose.setOrientation(ori_para);
				break;
			}
		}
		else	break;
		count++;
	}
	cout << "File loading done." << endl;
	pos_file.close();
	showParameter(prev_pose1, prev_pose2);

	if(isinit || str_frame == 0)
	{
		// get data from HandInfo
		int finger_num;
		SK::Array<Vector3> pc_tips, pc_dirs, pc_palm;
		featurefromHandInfo(str_frame, finger_num, pc_tips, pc_dirs, pc_palm);

		// Try to follow last frame global position
		SK::Vector3 tmpdir = pc_palm[1] - pc_palm[0];
/*		pc_palm[0] = prev_pose1.getPosition();
		pc_palm[1] = prev_pose1.getPosition() + tmpdir;*/

		// Initailization
		SK::Array<SK::Array<bool>> permlist = MyTools::fingerChoosing(finger_num);
		double bestcost = 1000000;
		for(size_t i = 0; i < permlist.size(); i++)
		{
			Initialization in = Initialization(finger_num, pc_tips, pc_dirs, handmodel);
			
			// set global position & orientation
			initpose.setPosition(pc_palm[0]);
			initpose.setOrientation(pc_palm[1] - pc_palm[0]);

			in.fingerExist(permlist[0]);
			in.goFullInitail(initpose, false);
			if(bestcost > in.getCost())
			{
				bestcost = in.getCost();
				in.setFullResultPose(initpose);
			}/**/
		}

		/*	initpose.applyPose(handmodel);*/
		cout << "Initialzation done." << endl;
	}

	// ICP-PSO Optimization
	PSO pso = PSO(24, 30, 8, 1);
	if(str_frame > 1)	pso.setPrevPose(prev_pose1, prev_pose2);
//	if(isinit)	pso.generateParticles(initpose, handpose);
	if(isinit)	pso.generateParticles(initpose);
	else if(str_frame == 0)	pso.generateParticles(initpose);
	else		pso.generateParticles(handpose);
//	else		pso.generateLParticles(handpose, info_pos);
	SK::Array<HandPose> particles;
	particles = pso.getAllParticles();

	if(show)	addHandModel(handmodel, isskel);
	if(show && isskel)	addHandSkeleton(handmodel);
	projectDepthImage(handmodel);

	// main while
	int frame = 0;
	while(!viewer->wasStopped())
	{
		handmodel = HandModel();
		if(isori)	particles[frame %  particles.size()].applyPose(handmodel);
		else		initpose.applyPose(handmodel);
	
		// reshow the model?
		if(reshow)
		{
			viewer->addText("Info...", 0, 450, "num_text");
			if(show)	addHandModel(handmodel, isskel);
			if(show && isskel)	addHandSkeleton(handmodel);
			reshow = false;
		}

//		bestpose.applyPose(handmodel);
		if(show)	updateHandModel(handmodel, isskel);
		if(show && isskel)	updateHandSkeleton(handmodel);

/*		showParameter(handpose, particles[(frame % particles.size())]);
		cout << endl;*/

		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "mycloud");
		viewer->spinOnce(100);
		rviewer->spinOnce(100);
		frame++;
	}

	return 0;
}