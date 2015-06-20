#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <EasiiSDK/Iisu.h>
#include <math.h> 
#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <random>
#include <windows.h>
#include "MyTools.hxx"
#include "HandInfo.hxx"
#include "Finger.hxx"

using namespace std;
using namespace SK;
using namespace SK::Easii;
using namespace pcl;

bool isrecord = false;
bool start_time = false;
int rec_index = 0;

const char *seqname = "Sequences/Seq/pcd_seq";
const char *seqtype = ".pcd";
const char *movname = "Movies/movie";
const char *movtype = ".skv";

PointCloud<PointXYZRGB> cloud;
PointCloud<PointXYZRGB>::Ptr cloudptr(&cloud);
boost::shared_ptr<visualization::PCLVisualizer> viewer (new visualization::PCLVisualizer ("3D Viewer"));

void save(string filename)
{
	cout << "Save " << cloud.points.size() << " data points to " << filename << endl;
	try
	{
		cloud.width = cloud.points.size();
		cloud.height = 1;
		cloud.is_dense = false;
		cout << "cloud width * height: " << cloud.width << " * " << cloud.height << " = " << (cloud.width * cloud.height) << endl;
		io::savePCDFileASCII(filename, cloud);
	}
	catch(IOException &e)
	{
		cout << "Cannot save because" << e.detailedMessage() << endl;
	}
}

// save clouds into pcd file
void savePCDFile (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	boost::shared_ptr<visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<visualization::PCLVisualizer> *> (viewer_void);
	if (event.getKeySym () == "s" && event.keyDown ())
	{
		cout << "Press s..." << endl;
		save("test_pcd.pcd");
		system("pause");
	}
}

// save clouds into pcd file
void savePCDSequence (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	boost::shared_ptr<visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<visualization::PCLVisualizer> *> (viewer_void);
	if (event.getKeySym () == "r" && event.keyDown ())
	{
		if(!isrecord)
		{
			cout << "Press r, start to record sequence..." << endl;
			isrecord = true;
			rec_index = 0;
		}
		else
		{
			cout << "Press r again, total " << rec_index << " frames, stop..." << endl;
			isrecord = false;
			start_time = false;
//			system("pause");
		}
	}
}

int main(int argc, char **argv)
{
	Iisu &iisu = Iisu::instance();
	Result result;

	// iisu initialization
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
	viewer->registerKeyboardCallback (savePCDFile, (void*)&viewer);
	viewer->registerKeyboardCallback (savePCDSequence, (void*)&viewer);

	// Camera initialization
	vector<visualization::Camera> camera;
	viewer->setCameraPosition(-14.4617, -171.208, 6.5311, 0, 0, 1);

	// Recorder initialization
	Recorder &recorder = iisu.getScene().getRecorder();

	int frame = 0;
    while (iisu.update() && !viewer->wasStopped())
    {
        iisu.acquire();
        frame = iisu.getScene().getSource().getFrame();
//		recorder = iisu.getScene().getRecorder();
		CameraInfo &camerainfo = iisu.getScene().getSource().getCameraInfo();
		
		// Assume one hand here
        Hand &hand = iisu.getScene().getHand(0);
		Calibration &cal = iisu.getScene().getCalibration();

		cloudptr->points.resize(hand.getFingerTipPositions3D().size() + hand.getMeshPoints3D().size());
		HandInfo handinfo = HandInfo(hand.getFingerCount(), hand.getMeshPoints3D(), hand.getFingerTipPositions3D(), 
									 hand.getPalmPosition3D(), hand.getPalmNormal3D());
		handinfo.setHandPointCloud(cloudptr, cal);

		// Show info if recording
		if(isrecord && !start_time)
		{
			Array<int> order = handinfo.getOrderedFingers();
			if(handinfo.isFingers() && !handinfo.isEmpty())
			{
				cout << "order: ";
				for(int i = 0; i < handinfo.getFingerNum(); i++)
					cout << order[i] << " ";
				cout << endl;
			}
			handinfo.showInfo(viewer);
			stringstream ss;
			ss << movname << movtype;
			const std::string tmp = ss.str();
			const char *name = tmp.c_str();
			cout << recorder.start(name) << endl;
			start_time = true;
		}

		// Update viewer
		viewer->updatePointCloud(cloudptr, "mycloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "mycloud");/**/
		
		// Recording
		if(isrecord)
		{
			stringstream ss;
			ss << seqname << rec_index << seqtype;
			save(ss.str());
			rec_index++;
		}
		else if(recorder.isRecording())
			recorder.stop();

		viewer->spinOnce(50);
		iisu.release();
		
    }

    iisu.stop();
    iisu.shutdown();
    return EXIT_SUCCESS;
}
