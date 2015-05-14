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
#include "MyTools.hxx"
#include "HandInfo.hxx"
#include "Finger.hxx"

using namespace std;
using namespace SK;
using namespace SK::Easii;
using namespace pcl;

PointCloud<PointXYZRGB> cloud;
PointCloud<PointXYZRGB>::Ptr cloudptr(&cloud);
boost::shared_ptr<visualization::PCLVisualizer> viewer (new visualization::PCLVisualizer ("3D Viewer"));

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
	viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);

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
//		handinfo.setHandPointCloud(cloudptr);

		// test: show order...
		Array<int> order = handinfo.getOrderedFingers();
		if(handinfo.isFingers() && !handinfo.isEmpty())
		{
			cout << "order: ";
			for(int i = 0; i < handinfo.getFingerNum(); i++)
				cout << order[i] << " ";
			cout << endl;
		}

		// Update viewer
		viewer->removeAllShapes();
		handinfo.showInfo(viewer);
		viewer->updatePointCloud(cloudptr, "mycloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "mycloud");/**/
			
		viewer->spinOnce(100);
		iisu.release();
		
    }

    iisu.stop();
    iisu.shutdown();
    return EXIT_SUCCESS;
}
