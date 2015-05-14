//============================================================
//
// Easii SDK C++ Tutorial : Step 3 - Read Data & Get/Set Parameters
//
// In this tutorial you will learn how to :
//  - initialize iisu with appropriate configuration 
//  - start/stop iisu processing loop 
//  - clean iisu execution context when shutting down iisu
//  - register specific event listeners
//  - get informed when the camera gets occluded/deoccluded
//  - get informed of asynchronous iisu errors
//  
//  - get informed when iisu has finished processing a camera frame
//	- register data handles
//	- register paremeter handles
//	- get/set parameters
//	- access data handles contents and use them in your application
//	- notify iisu you start/finished processing a data frame
//============================================================



//EasiiSDK global header (includes all iisu functionalities)
#include <pcl/visualization/cloud_viewer.h>
#include <EasiiSDK/Iisu.h>
#include <iostream>
#include <windows.h>
#define JUMP_RATIO 0.65
#define CROUCH_RATIO 0.40

using namespace std;
using namespace SK;
using namespace SK::Easii;
using namespace pcl;

PointCloud<PointXYZRGB>::Ptr cloud(new pcl::PointCloud<PointXYZRGB>);
//visualization::CloudViewer viewer("Simple Cloud Viewer");

const int c_PIXEL_COUNT = 76800; // 320x240
const int c_MIN_Z = 100; // discard points closer than this
const int c_MAX_Z = 800; // discard points farther than this


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

    int frame = 0;
    while (true)
    {
        if (iisu.update())
        {
            iisu.acquire();
            frame = iisu.getScene().getSource().getFrame();
            Hand &hand = iisu.getScene().getHand(0);
			/*int32_t rl = -1;
			if(hand.getStatus() != 0)
			{
				int32_t rl_old = rl;
				rl = hand.getHandedness();
				if(rl == rl_old)
					;
			    else if(rl == 0)
					cout << "unknown" << endl;
				else
					cout << ((rl == 1)? "left" : "right") << endl;
			}*/
			Array<Vector3> handpoints = hand.getMeshPoints3D();
			Array<Vector3> fingerspoints = hand.getFingerTipPositions3D();
			Vector3 plampoint = hand.getPalmPosition3D();
			cout << "Array hand size: " << handpoints.size() << "\t";
			cout << "Array finger size: " << fingerspoints.size() << "\t";
			cout << fixed << setprecision(2) << "plam pos: " << plampoint[0] << ", " << plampoint[1] << ", " << plampoint[2]<< endl;
			cloud->points.resize(handpoints.size() + fingerspoints.size() + 1);
			// Update viewer
			for(int i = 0; i < (int)handpoints.size(); i++)
			{
				cloud->points[i].x = handpoints[i][0];
				cloud->points[i].y = handpoints[i][1];
				cloud->points[i].z = handpoints[i][2];
				cloud->points[i].r = 255;
				cloud->points[i].g = 255;
				cloud->points[i].b = 255;
			}
			for(int i = 0; i < (int)fingerspoints.size(); i++)
			{
				cloud->points[handpoints.size() + i].x = fingerspoints[i][0];
				cloud->points[handpoints.size() + i].y = fingerspoints[i][1];
				cloud->points[handpoints.size() + i].z = fingerspoints[i][2];
				cloud->points[handpoints.size() + i].r = 255;
				cloud->points[handpoints.size() + i].g = 0;
				cloud->points[handpoints.size() + i].b = 0;
			}
			{
				cloud->points[handpoints.size() + fingerspoints.size()].x = plampoint[0];
				cloud->points[handpoints.size() + fingerspoints.size()].y = plampoint[1];
				cloud->points[handpoints.size() + fingerspoints.size()].z = plampoint[2];
				cloud->points[handpoints.size() + fingerspoints.size()].r = 0;
				cloud->points[handpoints.size() + fingerspoints.size()].g = 255;
				cloud->points[handpoints.size() + fingerspoints.size()].b = 0;
			}
			iisu.release();
        }
    }

    iisu.stop();
    iisu.shutdown();
    return EXIT_SUCCESS;
}
