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

// for testing
const bool opti = true;
const bool skel = true;
const bool show = true;
const bool fixo = false;
const int spin_time = 200;

PointCloud<PointXYZRGB> cloud;
PointCloud<PointXYZRGB>::Ptr cloudptr(&cloud);
boost::shared_ptr<RangeImagePlanar> planar (new RangeImagePlanar());
boost::shared_ptr<visualization::PCLVisualizer> viewer (new visualization::PCLVisualizer ("3D Viewer"));
boost::shared_ptr<visualization::ImageViewer > iviewer (new visualization::ImageViewer ("image Viewer"));
//boost::shared_ptr<visualization::RangeImageVisualizer > rviewer (new visualization::RangeImageVisualizer ("range Viewer"));

// save clouds into pcd file
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
  boost::shared_ptr<visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "s" && event.keyDown ())
  {
    cout << "s was pressed => save " << cloud.points.size() << " data points to test_pcd.pcd." << endl;
	try
	{
		cloud.width = cloud.points.size();
		cloud.height = 1;
		cloud.is_dense = false;
		cout << "cloud width * height: " << cloud.width << " * " << cloud.height << " = " << (cloud.width * cloud.height) << endl;
		io::savePCDFileASCII("test_pcd.pcd", cloud);
		system("pause");
	}
	catch(IOException &e)
	{
		cout << "Cannot save because" << e.detailedMessage() << endl;
	}
  }
}

void setFeatures(SK::Array<Vector3> &pc_tips, SK::Array<Vector3> &pc_dirs, SK::Array<int> &index)
{
	SK::Array<Vector3> ori_tips = pc_tips;
	SK::Array<Vector3> ori_dirs = pc_dirs;
	for(int i = 0; i < 5; i++)
	{
		pc_tips[i] = ori_tips[index[i]];
		pc_dirs[i] = ori_dirs[index[i]];
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

void showImage(Image &image)
{

//	cout << "image size, width = " << image.getImageInfos().width << ", height = " << image.getImageInfos().height << endl;
	
	// Update image viewer
	unsigned char data[320 * 240];
	for(int i = 0; i < 240; i++)
		for(int j = 0; j < 320; j++)
			data[i * 320 + j] = (unsigned char)image.readPixel<uint16_t>(j, i);
	iviewer->showMonoImage(data, image.getImageInfos().width, image.getImageInfos().height, "color image", 1.0);/**/
//	iviewer->showMonoImage((unsigned char *)image.getRAW(), image.getImageInfos().width, image.getImageInfos().height, "color image", 1.0);
	iviewer->spinOnce(spin_time);

}

void setDepthImage()
{
//	planar->reset();
	int image_x = 320, image_y = 240;
	float center_x = 160.0, center_y = 120.0;
	float focal_x = 224.502, focal_y = 230.494;
	Affine3f sensorpose = Affine3f(Eigen::Translation3f(-14.4617, -171.208, 5.5311) * AngleAxisf(-0.5 * M_PI, Vector3f::UnitX()));
	planar->createFromPointCloudWithFixedSize(*cloudptr, image_x, image_y, center_x, center_y, focal_x, focal_y, sensorpose);
}

int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);

	// iisu initialization
	Iisu &iisu = Iisu::instance();
	Result result;
	cout << "Initializing..." << endl;
	result = iisu.init();
	if(!result.succeeded())
	{
		cout << "Init error: " << result.getDescription().ptr() << endl;
		return result.getErrorCode();
	}
	else cout << "Init done." << endl << "Starting..." << endl;
    result = iisu.start();
	if(!result.succeeded())
	{
		cout << "Start error: " << result.getDescription().ptr() << endl;
		iisu.close();
		return result.getErrorCode();
	}
	else cout << "Start done." << endl;

    // Viewer initialization
	viewer->setBackgroundColor (0, 0, 0);
	viewer->initCameraParameters();
	viewer->addCoordinateSystem (1.0);
	viewer->addPointCloud(cloudptr, "mycloud");
	viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);

	iviewer->setSize(320, 240);
	iviewer->setPosition(650, 300);

/*	rviewer->setSize(320, 240);
	rviewer->setPosition(0, 300);*/

	// Camera initialization
	vector<visualization::Camera> camera;
	if(!fixo)	viewer->setCameraPosition(-14.4617, -171.208, 6.5311, 0, 0, 1);

	// Hand Model & Pose initialization
	HandModel handmodel = HandModel();
	HandPose curr_pose, prev_pose;
	bool ispre = false;	// first time
	if(show)	addHandModel(handmodel, skel);
	if(show && skel)	addHandSkeleton(handmodel);
	
	// main while
	int frame = 0;
    while (iisu.update() && !viewer->wasStopped())
    {
		// Get hand info from iisu
		iisu.acquire();
		frame = iisu.getScene().getSource().getFrame();
		Image refimage = iisu.getScene().getSource().getDepthImage();
		CameraInfo &camerainfo = iisu.getScene().getSource().getCameraInfo();
		Hand &hand = iisu.getScene().getHand(0);	// Assume one hand here
		Calibration &cal = iisu.getScene().getCalibration();

		// Hand Info initialization
		cloudptr->points.resize(hand.getFingerTipPositions3D().size() + hand.getMeshPoints3D().size());
		HandInfo handinfo = HandInfo(hand.getFingerCount(), hand.getMeshPoints3D(), hand.getFingerTipPositions3D(), 
									 hand.getPalmPosition3D(), hand.getPalmNormal3D());
		handinfo.setHandPointCloud(cloudptr, fixo);

		// set planar
		setDepthImage();

		if(!handinfo.isEmpty() && opti)
		{
			// Get tips and plam normal (hand feature)
			SK::Array<Vector3> pc_tips = handinfo.getAllFingerTips();
			SK::Array<Vector3>	pc_dirs = handinfo.getAllFingerDirs();
			setFeatures(pc_tips, pc_dirs, handinfo.getOrderedFingers());
			
			// Reset the Pose
			handmodel = HandModel();/**/

			// Hand Pose initialization if needed
			prev_pose = curr_pose;
			if(!ispre)
			{
				curr_pose = HandPose();
				SK::Array<SK::Array<bool>> permlist = MyTools::fingerChoosing(handinfo.getFingerNum());
				double bestcost = 1000000;
				for(size_t i = 0; i < permlist.size(); i++)
				{
					Initialization in = Initialization(handinfo.getFingerNum(), pc_tips, pc_dirs, handmodel);
					curr_pose.setPosition(handinfo.getpalmOri()[0]);
					curr_pose.setOrientation(handinfo.getpalmOri()[1] - handinfo.getpalmOri()[0]);
					in.fingerExist(permlist[i]);
					in.goFullInitail(curr_pose, false);
					if(bestcost > in.getCost())
					{
						bestcost = in.getCost();
						in.setFullResultPose(curr_pose);
					}
				}
				curr_pose.applyPose(handmodel);/**/
			}

			// ICP-PSO Optimization
			PSO pso = PSO(15, 24, -10, 1);
			if(ispre)
				pso.generateParticles(prev_pose);
			else
				pso.generateParticles(curr_pose);
			pso.goGeneration_mp(cloud, handmodel, false, false);
//			pso.goGeneration_full(cloud, *planar.get(), handmodel, false, false);
			HandPose bestpose = pso.getBestPose();
			bestpose.applyPose(handmodel);
			curr_pose = bestpose;/**/

/*			handmodel.moveHand(Vector3(200, 0, 0));*/
			if(show)	updateHandModel(handmodel, skel);
			if(show && skel)	updateHandSkeleton(handmodel);
			ispre = true;
		}
		else	ispre = false;

		// Update camera
		viewer->getCameras(camera);
/*		cout << "camera: " << endl 
			 << "position: (" << camera[0].pos[0] << ", " <<  camera[0].pos[1] << ", " << camera[0].pos[2] << ")\n"
			 << "view: (" << camera[0].view[0] << ", " <<  camera[0].view[1] << ", " << camera[0].view[2] << ")\n";*/

		// Update viewers
		showImage(refimage);
		handinfo.showInfo(viewer);
		viewer->updatePointCloud(cloudptr, "mycloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "mycloud");
		viewer->spinOnce(spin_time);
/*		rviewer->showRangeImage(*planar.get());
		rviewer->spinOnce(spin_time);*/
		iisu.release();
		
    }

    iisu.stop();
    iisu.shutdown();
    return EXIT_SUCCESS;
}
