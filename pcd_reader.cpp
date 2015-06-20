#include <iostream>
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

const int HEIGHT = 240;
const int WIDTH = 320;

PointCloud<PointXYZRGB> cloud;
PointCloud<PointXYZRGB>::Ptr cloudptr(&cloud);
boost::shared_ptr<visualization::PCLVisualizer> viewer (new visualization::PCLVisualizer ("3D Viewer"));
boost::shared_ptr<visualization::RangeImageVisualizer > rviewer (new visualization::RangeImageVisualizer  ("range Viewer"));
boost::shared_ptr<RangeImagePlanar> planar (new RangeImagePlanar());

Vector3 getTipVector(int finger)
{
	float data[5][3] = {{31, 225, -32}, {-2, 235, -20}, {58, 214, -59}, {-56, 196, -98}, {-42, 231, -38}};;
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

Vector3 getHandPose(int finger)	// useless now!!!
{
	float pose[5][3] = {{0, -0.174533, 0.349066}, {-0.0593308, 0, -0.0362806}, {-0.104054, 0, -0.344614}, {0.0797768, 0, -0.349066}, {0.734575, 0, -0.349066}};
	if(finger >=5)
		return Vector3(0.0, 0.0, 0.0);
	if(finger < 0)
		return Vector3(1.57, 0.0, 0.0);
	return Vector3(pose[finger][0], pose[finger][1], pose[finger][2]);
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

void addModifiedPointCloud()
{
	SK::Array<Vector3> palm = getPalmDirection();
	Matrix4 rot = MyTools::transVector(palm[0], (palm[1] - palm[0]), Vector3(0, 0, 0), Vector3(0, 0, 1));
//	cout << "show rot: " << rot << endl;
	for(size_t i = 0; i < cloudptr->points.size(); i++)
		cloudptr->points[i] = MyTools::rotatedPoint(rot, cloudptr->points[i]);
	viewer->addPointCloud(cloudptr, "mycloud");
}

void addMPCFeature(SK::Array<Vector3> &pc_tips, SK::Array<Vector3> &pc_dirs, SK::Array<int> &match) 
{
	SK::Array<Vector3> palm = getPalmDirection();
	Matrix4 rot = MyTools::transVector(palm[0], (palm[1] - palm[0]), Vector3(0, 0, 0), Vector3(0, 0, 1));
	// rotation term
	palm[0] = rot.multiplyByPoint(palm[0]);
	palm[1] = rot.multiplyByPoint(palm[1]);

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
		
		// rotation term
		tips = rot.multiplyByPoint(tips);
		fdir[0] = rot.multiplyByPoint(fdir[0]);
		fdir[1] = rot.multiplyByPoint(fdir[1]);
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

void addHandModel(HandModel &handmodel)
{
	SK::Array<Sphere> sgroup = handmodel.getFullHand();
	Vector3 model_str = handmodel.getGlobalpos();
	Vector3 model_trans = handmodel.getGlobaltrans();
	Vector3 model_end = Vector3((model_str[0] + model_trans[0] * 40), (model_str[1] + model_trans[1] * 40), (model_str[2] + model_trans[2] * 40));
	viewer->addArrow(model_str, model_end, 200, 200, 200, false, "model orientation");

	int jointspot[] = {16, 18, 20, 21, 22, 24, 26, 27, 28, 30, 32, 33, 34, 36, 38, 39, 40, 44, 46, 47};
	for(size_t i = 0; i < 20; i++)
		viewer->addSphere(MyTools::vectortoPointXYZ(sgroup[jointspot[i]].getCenter()), 3, 
						  sgroup[jointspot[i]].getColor()[0], sgroup[jointspot[i]].getColor()[1], sgroup[jointspot[i]].getColor()[2], 
						  sgroup[jointspot[i]].getName());
	string dirname[5] = {"direction 0", "direction 1", "direction 2", "direction 3", "direction 4"};
	for(int i = 0; i < 5; i++)
		viewer->addArrow(handmodel.getFingerTips(i), handmodel.getFingerBaseJoint(i), 0, 0, 200, false, dirname[i]);
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

int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);
	
	// Load the point cloud
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
	
	// Point cloud Loading and random sampling
//	cloud = MyTools::downsampling(cloud);
	clock_t begin = clock();
//	addModifiedPointCloud();
	viewer->addPointCloud(cloudptr, "mycloud");

	// Get tips and plam normal (hand feature)
	SK::Array<Vector3> pc_tips, pc_dirs;
	SK::Array<int> finger_match;			// Special case!!!
	finger_match.pushBack(3);finger_match.pushBack(2);finger_match.pushBack(4);finger_match.pushBack(0);finger_match.pushBack(1);
//	addMPCFeature(pc_tips, pc_dirs, finger_match);
	addNPCFeature(pc_tips, pc_dirs, finger_match);

	// Depth map setting
	showDepthImage();

	// Hand Model initialization
	HandModel handmodel = HandModel();

	// Hand Pose initialization
	HandPose handpose = HandPose();

/*	Initialization in = Initialization(5, pc_tips, pc_dirs, handmodel);
	SK::Array<bool> isfinger;				// Special case!!!
	isfinger.pushBack(true);isfinger.pushBack(true);isfinger.pushBack(true);isfinger.pushBack(true);isfinger.pushBack(true);
	in.fingerExist(isfinger);
	in.goFullInitail(handpose, false);
	in.setFullResultPose(handpose);
	in.goInitail(handpose);
	in.goInitail();
	in.setResultPose(handpose);*/
/*	handpose.applyPose(handmodel);
	cout << "Initialzation done." << endl;*/

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
//		cout << "Term " << i << ", cost = " << in.getCost() << endl;
		if(bestcost > in.getCost())
		{
			bestcost = in.getCost();
			in.setFullResultPose(handpose);
		}/**/
	}
	handpose.applyPose(handmodel);/**/
	cout << "Initialzation done." << endl;
	clock_t mid = clock();
//	showParameter(handpose);

	MyCostFunction costf = MyCostFunction(cloud, handmodel);
	costf.calculate();
	cout << "before point = " << costf.getCost() << endl;

	handmodel = HandModel();

	// ICP-PSO Optimization
	PSO pso = PSO(40, 24, -10, 1);
	pso.generateParticles(handpose);
	cout << "PSO initial done..." << endl;
//	pso.goGeneration_mp(cloud, handmodel, false, true);
	pso.goGeneration_full(cloud, *planar.get(), handmodel, false, false);
	cout << "PSO optimizaiton done..." << endl;
	HandPose bestpose = pso.getBestPose();
	bestpose.applyPose(handmodel);
	cout << "show point: " << pso.getBestPoint() << endl;/**/

	// F-term information
/*	float pix_meter = pso.getPixmeter();
	vector<float> pure_vector;
	Index<flann::L2<float>> index = pso.buildDatasetIndex(*planar.get(), pure_vector);*/


	// ICP Optimization
/*	AICP aicp = AICP(10, 5, handpose);
	aicp.run_randomPara(cloud);
//	aicp.run_randomJoint(cloud);
//	aicp.run_randomPara(cloud, *planar.get(), pix_meter, pure_vector, index);
//	aicp.run_specPara(cloud, 24);
//	aicp.run_cyclePara(cloud);
	HandPose bestpose = aicp.getBestPose();
	bestpose.applyPose(handmodel);*/
	
	costf = MyCostFunction(cloud, handmodel);
	costf.calculate();
	cout << "after point = " << costf.getCost() << endl;

	// For better recognition
//	handmodel.moveHand(Vector3(200, 0, 0));
	
	// Time consumption
/*	clock_t end = clock();
	double init_t = double(mid - begin);
	double opt_t = double(end - mid);
	double total_t = double(end - begin);
	cout << "init comsume =  " << init_t << " ms, opt comsume = " << opt_t << " ms, total comsume = " << total_t  << " ms" << endl;
	showParameter(handpose, bestpose);*/

	addHandModel(handmodel);
	addHandSkeleton(handmodel);
	projectDepthImage(handmodel);

	// main while
	while(!viewer->wasStopped())
	{
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "mycloud");
		viewer->spinOnce(100);
		rviewer->spinOnce(100);
	}

	return 0;
}