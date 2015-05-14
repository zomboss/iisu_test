#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <EasiiSDK/Iisu.h>
#include <math.h> 
#include <iostream>
#include <algorithm>
#include <vector>
#include <random>
#include <windows.h>
#include "HandPose.hxx"
#include "MyTools.hxx"
#include "Sphere.hxx"
#include "HandModel.hxx"
#include "Finger.hxx"

#define JUMP_RATIO 0.65
#define CROUCH_RATIO 0.40

using namespace std;
using namespace SK;
using namespace SK::Easii;
using namespace pcl;

PointCloud<PointXYZRGB> cloud;
PointCloud<PointXYZRGB>::Ptr cloudptr(&cloud);
//visualization::CloudViewer viewer("Simple Cloud Viewer");
boost::shared_ptr<visualization::PCLVisualizer> viewer2 (new visualization::PCLVisualizer ("3D Viewer"));


const int c_PIXEL_COUNT = 76800; // 320x240
const int c_MIN_Z = 100; // discard points closer than this
const int c_MAX_Z = 800; // discard points farther than this
const int fingerRadius = 16;
const int iteration = 3;

bool distance(PointXYZRGB &tips, PointXYZRGB &point, int radius)
{
	return radius > sqrt(pow((tips.x - point.x), 2) + pow((tips.y - point.y), 2) + pow((tips.z - point.z), 2));
}

bool distance(Vector3 tips, PointXYZRGB &point, int radius)
{
	return radius > sqrt(pow((tips[0] - point.x), 2) + pow((tips[1] - point.y), 2) + pow((tips[2] - point.z), 2));
}

int* colorChoose(int index)
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
			color[1] = 0;
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

// save clouds into pcd file
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
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
	}
	catch(IOException &e)
	{
		cout << "Cannot save because" << e.detailedMessage() << endl;
	}
  }
}

int main(int argc, char **argv)
{
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
	viewer2->setBackgroundColor (0, 0, 0);
	viewer2->initCameraParameters();
	viewer2->addCoordinateSystem (1.0);
	viewer2->addPointCloud(cloudptr, "mycloud");
	viewer2->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer2);

	// Hand Model initialization
	cout << "Building model..." << endl;
	HandModel handmodel = HandModel();
	Array<Sphere> testG = handmodel.getFullHand();
	cout << "Building model done, size = " << testG.size() << endl;

	// Hand Pose initialization
	HandPose handpose = HandPose(Vector3(0.0,-0.21238,1.11737), Vector3(0,0,0), Vector3(0,0,0), 
								 Vector3(-0.045404,0,0.114452), Vector3(0,0,0), Vector3(0,0,0), 
								 Vector3(-0.174533,0,-0.00280097), Vector3(0,0,0), Vector3(0,0,0), 
								 Vector3(-0.0267493,0,-0.265335), Vector3(0,0,0), Vector3(0,0,0), 
								 Vector3(0.40864,0,-0.523599), Vector3(0,0,0), Vector3(0,0,0));
	handpose.applyPose(handmodel);
	handmodel.rotateHand(Vector3(0,0,0.185863));

	Array<float> curr_para = handpose.getAllParameters();
	Array<float> new_para = MyTools::perturbParameter(curr_para);
	cout << "from " << endl;
	for(size_t i = 0; i < curr_para.size(); i++)
		cout << curr_para[i] << "\t";
	cout << endl << "to " << endl;
	for(size_t i = 0; i < new_para.size(); i++)
		cout << new_para[i] << "\t";
	cout << endl;	
	HandPose tmppose = handpose;
	tmppose.setAllParameters(new_para);
	handmodel.rotateHand(-Vector3(0,0,0.185863));
	tmppose.applyPose(handmodel);
	handmodel.rotateHand(Vector3(0,0,0.185863));

	// Set test plam and fingers
	Vector3 pos1 = Vector3(8,245,-105), pos2 = handmodel.getGlobalpos(), ori1 = Vector3(-4,-100,8), ori2 = handmodel.getGlobaltrans(),
			testpos1 = Vector3(-84,232,-107), testpos2 = Vector3(55,232,-18), testpos3 = Vector3(-30,269,5), testpos4 = Vector3(11,256,8), testpos5 = Vector3(-67,262,-18), 
			testori1 = Vector3(96,7,-28), testori2 = Vector3(-51,19,-84), testori3 = Vector3(22,-22,-95), testori4 = Vector3(-1,-18,-99), testori5 = Vector3(45,-18,-88);
	Matrix4 rot = MyTools::transVector(pos1, ori1, pos2, ori2);
	cout << "now pos2 = " << pos2 << endl;
	cout << "now ori2 = " << ori2 << endl;
	pos1 = rot.multiplyByPoint(pos1);
	testpos1 = rot.multiplyByPoint(testpos1); //(0, 255, 0) -> green ()
	testpos2 = rot.multiplyByPoint(testpos2); //(127, 127, 0) -> yellow ()
	testpos3 = rot.multiplyByPoint(testpos3); //(0, 127, 127) -> cyan ()
	testpos4 = rot.multiplyByPoint(testpos4); //(127, 0, 127) -> purple ()
	testpos5 = rot.multiplyByPoint(testpos5); //(84, 84, 84) -> white ()
	ori1 = rot.multiplyByVector(ori1);
	testori1 = rot.multiplyByVector(testori1);
	testori2 = rot.multiplyByVector(testori2);
	testori3 = rot.multiplyByVector(testori3);
	testori4 = rot.multiplyByVector(testori4);
	testori5 = rot.multiplyByVector(testori5);
	cout << "now pos1 = " << pos1 << endl;
	cout << "testpos1 = " << testpos1 << ", testori1 = " << testori1 << endl;
	cout << "testpos2 = " << testpos2 << ", testori2 = " << testori2 << endl;
	cout << "testpos3 = " << testpos3 << ", testori3 = " << testori3 << endl;
	cout << "testpos4 = " << testpos4 << ", testori4 = " << testori4 << endl;
	cout << "testpos5 = " << testpos5 << ", testori5 = " << testori5 << endl;/**/

	int frame = 0;
    while (iisu.update() && !viewer2->wasStopped())
    {
        iisu.acquire();
        frame = iisu.getScene().getSource().getFrame();
		CameraInfo &camerainfo = iisu.getScene().getSource().getCameraInfo();
		// Assume one hand here
        Hand &hand = iisu.getScene().getHand(0);
		Calibration &cal = iisu.getScene().getCalibration();
		Array<Vector3> handpoints = hand.getMeshPoints3D();
		Array<Vector3> fingerspoints = hand.getFingerTipPositions3D();
		Vector3 plampoint = cal.worldToCamera(hand.getPalmPosition3D());
		Vector3 plamnormal = cal.worldToCamera(hand.getPalmNormal3D());
/*		{
			cout << "Camera Position:(camera) " << cal.worldToCamera(camerainfo.getPosition()) << endl;
			cout << "Camera Up:(camera) " << cal.worldToCamera(camerainfo.getUp()) << endl;
			cout << "Camera Left:(camera) " << cal.worldToCamera(camerainfo.getLeft()) << endl;
			cout << "Camera Front:(camera) " << cal.worldToCamera(camerainfo.getFront()) << endl;
		}*/
			
		PointXYZ plamori_str, plamori_end;
		plamori_str = PointXYZ(plampoint[0] * 1000, plampoint[1] * 1000, plampoint[2] * 1000);
		plamori_end = PointXYZ((plamori_str.x + plamnormal[0] * 100), (plamori_str.y + plamnormal[1] * 100), (plamori_str.z + plamnormal[2] * 100));

		cloudptr->points.resize(fingerspoints.size() + handpoints.size());
/*		cout << "data size: " << handpoints.size() << endl;*/

		if(handpoints.size() != 0)
			cout << fixed << setprecision(0) << "plam vector: " << plamori_str << " -> " << plamori_end << endl;

		// Update viewer
		if(handpoints.size() != 0)	cout << "tips: ";
		for(int i = 0; i < (int)fingerspoints.size(); i++)
		{
			Vector3 camerapoint = cal.worldToCamera(fingerspoints[i]);
			cloudptr->points[i].x = camerapoint[0] * 1000;
			cloudptr->points[i].y = camerapoint[1] * 1000;
			cloudptr->points[i].z = camerapoint[2] * 1000;
			int *color = colorChoose(i);
			cloudptr->points[i].r = color[0];
			cloudptr->points[i].g = color[1];
			cloudptr->points[i].b = color[2];
//			if(handpoints.size() != 0)
//				cout << fixed << setprecision(0)  << "(" << cloudptr->points[i].x << ", " << cloudptr->points[i].y << ", " << cloudptr->points[i].z << ") ";
		}
		if(handpoints.size() != 0)	cout << endl;

		// array for finger initialization
		Array<Vector3> fingersPoint[5];
		Array<int> fingersPointIndex[5];
		for(int i = 0; i < (int)handpoints.size(); i++)
		{
			Vector3 camerapoint = cal.worldToCamera(handpoints[i]);
			cloudptr->points[fingerspoints.size() + i].x = camerapoint[0] * 1000;
			cloudptr->points[fingerspoints.size() + i].y = camerapoint[1] * 1000;
			cloudptr->points[fingerspoints.size() + i].z = camerapoint[2] * 1000;
			cloudptr->points[fingerspoints.size() + i].r = 255;
			cloudptr->points[fingerspoints.size() + i].g = 255;
			cloudptr->points[fingerspoints.size() + i].b = 255;
				
			// find finger points for initialization
			for(size_t j = 0; j < fingerspoints.size(); j++)
			{
				if(distance(cloudptr->points[j], cloudptr->points[fingerspoints.size() + i], fingerRadius))
				{
					int *color = colorChoose(j);
					cloudptr->points[fingerspoints.size() + i].r = color[0];
					cloudptr->points[fingerspoints.size() + i].g = color[1];
					cloudptr->points[fingerspoints.size() + i].b = color[2];
						
					// add to fingersPoint
					Vector3 tmpPoint;
					tmpPoint[0] = camerapoint[0] * 1000;
					tmpPoint[1] = camerapoint[1] * 1000;
					tmpPoint[2] = camerapoint[2] * 1000;
					fingersPoint[j].pushBack(tmpPoint);
					fingersPointIndex[j].pushBack(i);
				}

			}
		}

		//for fingers detection
		vector<Finger> fingers;
		for(size_t i = 0; i < fingerspoints.size(); i++)
		{
			Vector3 cameraFpoint = cal.worldToCamera(fingerspoints[i]);
			cameraFpoint[0] *= 1000;
			cameraFpoint[1] *= 1000;
			cameraFpoint[2] *= 1000;
			Finger finger = Finger(i, cameraFpoint, fingersPoint[i], fingersPointIndex[i]);
			fingers.push_back(finger);
		}

		// Advance: Iterative term
		for(int t = 0; t < iteration; t++)
		{
			Array<Vector3> fingersPoint_Iter[5];
			Array<int> fingersPointIndex_Iter[5];
			for(size_t i = 0; i < handpoints.size(); i++)
			{
				Vector3 camerapoint = cal.worldToCamera(handpoints[i]);
				for(size_t j = 0; j < fingerspoints.size(); j++)
				{
					// get the farest point
					Vector3 maxp = fingers[j].getmaxPoint();
					if(distance(maxp, cloudptr->points[fingerspoints.size() + i], fingerRadius))
					{
						int *color = colorChoose(j);
						cloudptr->points[fingerspoints.size() + i].r = color[0];
						cloudptr->points[fingerspoints.size() + i].g = color[1];
						cloudptr->points[fingerspoints.size() + i].b = color[2];
						// add to fingersPoint
						Vector3 tmpPoint;
						tmpPoint[0] = camerapoint[0] * 1000;
						tmpPoint[1] = camerapoint[1] * 1000;
						tmpPoint[2] = camerapoint[2] * 1000;
						fingersPoint_Iter[j].pushBack(tmpPoint);
						fingersPointIndex_Iter[j].pushBack(i);
					}
				}
			}
			for(size_t i = 0; i < fingerspoints.size(); i++)
			{
				Vector3 cameraFpoint = cal.worldToCamera(fingerspoints[i]);
				cameraFpoint[0] *= 1000;
				cameraFpoint[1] *= 1000;
				cameraFpoint[2] *= 1000;
				fingers[i].addPointCloud(fingersPoint_Iter[i], fingersPointIndex_Iter[i]);
			}
		}

		if(handpoints.size() != 0 && !fingers.empty())
		{
			for(size_t i = 0; i < fingers.size(); i++)
				cout << "finger " << i << " : (" << fingers[i].gettipPoint()[0] << ", " << fingers[i].gettipPoint()[1] << ", " << fingers[i].gettipPoint()[2] << ") ";
			cout << endl;
		}/**/

		// show PCA eigenvector
		//if(handpoints.size() != 0 && !fingers.empty())	cout << fixed << setprecision(0)  << "vector X value: \n" << fingers[0].getDirection() << endl;
		//if(handpoints.size() != 0 && !fingers.empty())	cout << fixed << setprecision(0)  << "vector X coeff: \n" << fingers[0].getDirection_f() << endl;

		// add plam normal
		viewer2->removeAllShapes();
		viewer2->addArrow(plamori_str, plamori_end, 100, 100, 100, false, "plam orientation");
			
		// Test PCA
		string pcaname[5] = {"PCA Vector 0", "PCA Vector 1", "PCA Vector 2", "PCA Vector 3", "PCA Vector 4"};
		if(handpoints.size() != 0 && !fingers.empty())
		{
			for(size_t i = 0; i < fingers.size(); i++)
			{
				Vector3 pcaori = fingers[i].getDirection().normalizedCopy() * 100;
				viewer2->addArrow(fingers[i].gettipPoint(), (fingers[i].gettipPoint() + pcaori), 200, 200, 200, false, pcaname[i]);
				cout << "finger " << i << " direction: " << fingers[i].gettipPoint() << " -> " << (fingers[i].gettipPoint() + pcaori) << endl;
			}
		}/**/

		// hand model (spheres)
/*		Array<Sphere> sgroup = handmodel.getFullHand();
		Vector3 model_str = handmodel.getGlobalpos();
		Vector3 model_trans = handmodel.getGlobaltrans();
		Vector3 model_end = Vector3((model_str[0] + model_trans[0] * 40), (model_str[1] + model_trans[1] * 40), (model_str[2] + model_trans[2] * 40));
		viewer2->addArrow(model_str, model_end, 200, 200, 200, false, "model orientation");

		viewer2->addSphere(MyTools::vectortoPointXYZ(testpos1), 5.0, 0, 255, 0, "testpos1");
		viewer2->addSphere(MyTools::vectortoPointXYZ(testpos2), 5.0, 127, 127, 0, "testpos2");
		viewer2->addSphere(MyTools::vectortoPointXYZ(testpos3), 5.0, 0, 127, 127, "testpos3");
		viewer2->addSphere(MyTools::vectortoPointXYZ(testpos4), 5.0, 127, 0, 127, "testpos4");
		viewer2->addSphere(MyTools::vectortoPointXYZ(testpos5), 5.0, 84, 84, 84, "testpos5");
		viewer2->addSphere(MyTools::vectortoPointXYZ(pos1), 5.0, 0, 0, 255, "testpos");
		viewer2->addArrow(testpos1, (testpos1 + testori1), 0, 200, 0, false, "testori1");
		viewer2->addArrow(testpos2, (testpos2 + testori2), 0, 200, 0, false, "testori2");
		viewer2->addArrow(testpos3, (testpos3 + testori3), 0, 200, 0, false, "testori3");
		viewer2->addArrow(testpos4, (testpos4 + testori4), 0, 200, 0, false, "testori4");
		viewer2->addArrow(testpos5, (testpos5 + testori5), 0, 200, 0, false, "testori5");
		viewer2->addArrow(pos1, (pos1 + ori1), 200, 200, 200, false, "test orientation");*/
			
/*		for(size_t i = 0; i < sgroup.size(); i++)
			viewer2->addSphere(MyTools::vectortoPointXYZ(sgroup[i].getCenter()), sgroup[i].getRadius(), sgroup[i].getColor()[0], sgroup[i].getColor()[1], sgroup[i].getColor()[2], sgroup[i].getName());
		string dirname[5] = {"direction 0", "direction 1", "direction 2", "direction 3", "direction 4"};
		for(int i = 0; i < 5; i++)
			viewer2->addArrow(handmodel.getFingerTips(i), handmodel.getFingerBaseJoint(i), 0, 0, 200, false, dirname[i]);*/

		// viewer.showCloud(cloud);
		viewer2->updatePointCloud(cloudptr, "mycloud");
		viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "mycloud");/**/
			
		viewer2->spinOnce(100);
		iisu.release();
		
    }

    iisu.stop();
    iisu.shutdown();
    return EXIT_SUCCESS;
}
