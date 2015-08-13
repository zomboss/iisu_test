/**
Main Program
Purpose:
	(1)	Record pose data (.pcd file, Sequences/Seq directory is needed), info data(.txt file) and film (.skv file)
Usage:
<in runtime>
	(1) Key "r" to start/end recording
	(2) Key "s" to store a single pcd file
<postprocessing>
	(1)	Change names of Sequences/Seq directory, Movies/movie.skv and InfoData/info_seq.txt to prevent overwriting
*/

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <EasiiSDK/Iisu.h>
#include <math.h> 
#include <iostream>
#include <fstream>
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
const char *infoname = "InfoData/info_seq.txt";
const char *seqtype = ".pcd";
const char *movname = "Movies/movie";
const char *movtype = ".skv";

PointCloud<PointXYZRGB> cloud;
PointCloud<PointXYZRGB>::Ptr cloudptr(&cloud);
boost::shared_ptr<visualization::PCLVisualizer> viewer (new visualization::PCLVisualizer ("3D Viewer"));

void saveCloud(string filename)
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

void saveInfo(fstream &outputfile, HandInfo &curr_info)
{
	// Write finger number
	outputfile << curr_info.getFingerNum() << endl;
	
	// Write 5 tips into file
	SK::Array<SK::Vector3> tips = curr_info.getAllFingerTips();
	for(size_t i = 0; i < tips.size(); i++)
		outputfile << tips[i][0] << " " << tips[i][1] << " " << tips[i][2] << " ";
	outputfile << endl;

	// Write 5 directions into file
	SK::Array<SK::Vector3> dirs = curr_info.getAllFingerDirs();
	for(size_t i = 0; i < dirs.size(); i++)
		outputfile << dirs[i][0] << " " << dirs[i][1] << " " << dirs[i][2] << " ";
	outputfile << endl;

	// Write palm vector into file
	SK::Array<SK::Vector3> palm = curr_info.getpalmOri();
	outputfile << palm[0][0] << " " << palm[0][1] << " " << palm[0][2] << " " << palm[1][0] << " " << palm[1][1] << " " << palm[1][2] << endl;

	// Write order for fingers
	SK::Array<int> order = curr_info.getOrderedFingers();
	outputfile << order[0] << " " << order[1] << " " << order[2] << " " << order[3] << " " << order[4] << endl;

}

// save clouds into pcd file
void savePCDFile (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	boost::shared_ptr<visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<visualization::PCLVisualizer> *> (viewer_void);
	if (event.getKeySym () == "s" && event.keyDown ())
	{
		cout << "Press s..." << endl;
		saveCloud("test_pcd.pcd");
		system("pause");
	}
}

// save clouds into pcd file sequence
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

	// File initialization
	fstream file;
	file.open(infoname, ios::out);
	if(!file)
	{
		cout << "cannot open file " << infoname << "!!!" << endl;
		return -1;
	}

	// Recorder initialization
/**/Recorder &recorder = iisu.getScene().getRecorder();

	int frame = 0;
    while (iisu.update() && !viewer->wasStopped())
    {
        iisu.acquire();
        frame = iisu.getScene().getSource().getFrame();
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
			
			// Start to take a film
			stringstream ss;
			ss << movname << movtype;
			const std::string tmp = ss.str();
			const char *name = tmp.c_str();
/**/		cout << recorder.start(name) << endl;
			start_time = true;
		}

		// Update viewer
		viewer->updatePointCloud(cloudptr, "mycloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "mycloud");/**/
		
		// Recording
		if(isrecord)
		{
			stringstream ss;
			ss << seqname << rec_index << seqtype;
			saveCloud(ss.str());
			saveInfo(file, handinfo);
			rec_index++;
		}
		else if(recorder.isRecording())
		{
			recorder.stop();
			file.close();
		}/**/
		else if(start_time && file.is_open())
			file.close();

		viewer->spinOnce(50);
		iisu.release();
		
    }

    iisu.stop();
    iisu.shutdown();
    return EXIT_SUCCESS;
}
