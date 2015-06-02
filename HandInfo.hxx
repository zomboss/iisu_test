#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <EasiiSDK/Iisu.h>
#include <math.h> 
#include <iostream>
#include <algorithm>
#include <windows.h>
#include "Finger.hxx"

#define ITER 3
#define RADIUS 16

using namespace std;
using namespace pcl;
using namespace SK;
using namespace SK::Easii;

class HandInfo
{
public:
	HandInfo();
	HandInfo(int , SK::Array<Vector3> , SK::Array<Vector3> , Vector3 &, Vector3 &);
	SK::Array<Finger> getAllFingers() {return fingers;}
	SK::Array<Vector3> getpalmOri() {return palm_ori;}
	SK::Array<Vector3> getAllFingerTips();
	SK::Array<Vector3> getAllFingerDirs();
	SK::Array<int> getOrderedFingers();
	int getFingerNum();
	
	// old version
	void setHandPointCloud(PointCloud<PointXYZRGB> &,Calibration &);
	void initFinger(PointCloud<PointXYZRGB> &, Calibration &);
	void iterFinger(PointCloud<PointXYZRGB> &, Calibration &);

	void setHandPointCloud(PointCloud<PointXYZRGB>::Ptr &,Calibration &);
	void fingerDetection(PointCloud<PointXYZRGB>::Ptr &,Calibration &);
	void switchtoOrigin();
	
	// including switch to origin
	void setHandPointCloud(PointCloud<PointXYZRGB>::Ptr &, bool);
	void fingerDetection(PointCloud<PointXYZRGB>::Ptr &);

	bool isEmpty();
	bool isFingers();
	bool isFingers(int );

	void showInfo();
	void showInfo(boost::shared_ptr<visualization::PCLVisualizer> &);

private:
	int finger_num;
	SK::Array<Vector3> handpoints;
	SK::Array<Vector3> fingerspoints;
	SK::Array<Finger> fingers;
	SK::Array<Vector3> palm_ori;
	Vector3 palm_vec;
	Vector3 zero_point;
	
	
};