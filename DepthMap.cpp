#include <iostream>
#include <ctime>
#include <string>
#include <vector>
#include <math.h>
#include <omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/range_image/range_image_planar.h>
#include "glog/logging.h"
#include "MyTools.hxx"
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
using namespace flann;

const float BACKGROUND = 2000.0;
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

void addModifiedPointCloud()
{
	SK::Array<Vector3> palm = getPalmDirection();
	Matrix4 rot = MyTools::transVector(palm[0], (palm[1] - palm[0]), Vector3(0, 0, 0), Vector3(0, 0, 1));
//	cout << "show rot: " << rot << endl;
	for(size_t i = 0; i < cloudptr->points.size(); i++)
		cloudptr->points[i] = MyTools::rotatedPoint(rot, cloudptr->points[i]);
	viewer->addPointCloud(cloudptr, "mycloud");
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
	Vector3 model_str = handmodel.getGlobalpos();
	Vector3 model_trans = handmodel.getGlobaltrans();
	Vector3 model_end = Vector3((model_str[0] + model_trans[0] * 40), (model_str[1] + model_trans[1] * 40), (model_str[2] + model_trans[2] * 40));
	viewer->addArrow(model_str, model_end, 200, 200, 200, false, "model orientation");

	int jointspot[] = {16, 18, 20, 21, 22, 24, 26, 27, 28, 30, 32, 33, 34, 36, 38, 39, 40, 44, 46, 47};
	if(isdot)
		for(size_t i = 0; i < 20; i++)
			viewer->addSphere(MyTools::vectortoPointXYZ(sgroup[jointspot[i]].getCenter()), 3, 
							  sgroup[jointspot[i]].getColor()[0], sgroup[jointspot[i]].getColor()[1], sgroup[jointspot[i]].getColor()[2], 
							  sgroup[jointspot[i]].getName());
	else
		for(size_t i = 0; i < sgroup.size(); i++)
			viewer->addSphere(MyTools::vectortoPointXYZ(sgroup[i].getCenter()), sgroup[i].getRadius(), 
							  sgroup[i].getColor()[0], sgroup[i].getColor()[1], sgroup[i].getColor()[2], 
							  sgroup[i].getName());
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

flann::Index<flann::L2<float> > buildDataset(vector<float> &data, float &pix_meter)
{
	// Load valid index (x,y)
	bool prev = false;
	int counter = 0;
	for(int j = 0; j < HEIGHT; j++)
	{
		for(int i = 0; i < WIDTH; i++)
		{
			if(planar->isValid(i, j))
			{
				// get the distance between one pixel
				if(prev)
				{
					PointWithRange prev_point = planar->getPoint(i - 1, j);
					PointWithRange curr_point = planar->getPoint(i, j);
					pix_meter += squaredEuclideanDistance(prev_point, curr_point);
					counter++;
				}
				data.push_back((float)i);
				data.push_back((float)j);
				prev = true;
			}
			else prev = false;
		}
		prev = false;
	}
	pix_meter /= (float)counter;
//	cout << "distance in one pixel = " << pix_meter << endl;

	int pure_size = (int)data.size() / 2;
	float *pure_data = data.data();

	// Initial flann
	flann::Matrix<float> dataset(pure_data, pure_size, 2);
	
	// Construct an randomized kd-tree index using 4 kd-trees
	flann::Index<flann::L2<float> > index(dataset, flann::KDTreeIndexParams(4));
    index.buildIndex();

	return index;
}

void getNearestNeighborNeighbor(flann::Index<flann::L2<float> > &index, vector<float> &pure_data, int query_x, int query_y)
{
	float *query_data = new float[2];
	query_data[0] = (float)query_x; query_data[1] = (float)query_y;
//	cout << "show pure size = " << pure_size << endl;

	// Initial flann
    flann::Matrix<float> query(query_data, 1, 2);
	flann::Matrix<int> indices(new int[query.rows], query.rows, 1);
    flann::Matrix<float> dists(new float[query.rows], query.rows, 1);
	
	// Do a knn search, using 128 checks
    index.knnSearch(query, indices, dists, 1, flann::SearchParams(128));

	// Checking
	int tar = indices[0][0];	// first for how many points you query, second for how many ndighbors you want
//	cout << "index = " << tar << ", distance = " << dists[0][0];
//	cout << ", position: (" << pure_data[tar * 2] << ", " << pure_data[tar * 2 + 1] << ")" << endl;

}

void getNearestNeighborNeighbor(vector<float> &pure_vec, int query_x, int query_y)
{
	
	int pure_size = (int)pure_vec.size();
	float *pure_data = pure_vec.data();
	float *query_data = new float[2];
	query_data[0] = (float)query_x; query_data[1] = (float)query_y;
//	cout << "show pure size = " << pure_size << endl;

	// Initial flann
	flann::Matrix<float> dataset(pure_data, pure_size, 2);
    flann::Matrix<float> query(query_data, 1, 2);
	flann::Matrix<int> indices(new int[query.rows], query.rows, 1);
    flann::Matrix<float> dists(new float[query.rows], query.rows, 1);
	
	// Construct an randomized kd-tree index using 4 kd-trees
	flann::Index<flann::L2<float> > index(dataset, flann::KDTreeIndexParams(4));
    index.buildIndex();
	
	// Do a knn search, using 128 checks
    index.knnSearch(query, indices, dists, 1, flann::SearchParams(128));

	// Checking
	int tar = indices[0][0];	// first for how many points you query, second for how many ndighbors you want
	cout << "index = " << tar << ", distance = " << dists[0][0];
	cout << ", position: (" << pure_data[tar * 2] << ", " << pure_data[tar * 2 + 1] << ")" << endl;

}

void projectDepthImage(HandModel &handmodel)
{
	vector<float> pure_vec;
	float pix_meter = 0.0;
	flann::Index<flann::L2<float> > index = buildDataset(pure_vec, pix_meter);
	
	for(int i = 0; i < handmodel.getSphereSize(); i++)
	{
		Eigen::Vector3f point = MyTools::SKtoEigenVector(handmodel.getFullHand()[i].getCenter());
		int image_x, image_y;
		float range;
		planar->getImagePoint(point, image_x, image_y, range);
//		cout << "show " << handmodel.getFullHand()[i].getName() << " located in: (" << image_x << ", " <<image_y << ")"
//			 << ", range comp: " << range << "<---->" << planar->getPoint(image_x, image_y).range << ", valid? " << planar->isValid(image_x, image_y) << endl;
		//if(!planar->isValid(image_x, image_y))	getNearestNeighborNeighbor(pure_vec, image_x, image_y);
		if(!planar->isValid(image_x, image_y))	getNearestNeighborNeighbor(index, pure_vec, image_x, image_y);

		rviewer->addCircle(image_x, (240 - image_y), 5, handmodel.getFullHand()[i].getName());
	}
}

int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);
	
	// Load the point cloud
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

	//addModifiedPointCloud();
	viewer->addPointCloud(cloudptr, "mycloud");

	// Depth map setting
	showDepthImage();

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

	clock_t begin = clock();
	// Point cloud Loading and random sampling
//	cloud = MyTools::downsampling(cloud);
	cout << "cloud size = " << cloud.size() << endl;

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

	
	// Time consumption
	clock_t end = clock();
	double total_t = double(end - begin);
	cout << "total comsume = " << total_t  << " ms" << endl;/**/
	
	// Show th model
	addHandModel(handmodel, skel);
	if(skel)	addHandSkeleton(handmodel);

	// show the spot on planar
	projectDepthImage(handmodel);/**/


	// main while
	while(!viewer->wasStopped() && !rviewer->wasStopped())
	{
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "mycloud");
		viewer->spinOnce(100);
		rviewer->spinOnce(100);
	}

	return 0;
}