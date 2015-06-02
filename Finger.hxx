#pragma once

#include <EasiiSDK/Iisu.h>
#include <math.h> 
#include <iostream>
#include <algorithm>
#include <windows.h>

using namespace std;
using namespace SK;
using namespace SK::Easii;

class Finger
{
public:
	Finger();
	Finger(int, const Vector3 &, const SK::Array<Vector3> &,  const SK::Array<int> &);
	
	void addPointCloud( const SK::Array<Vector3>&, const SK::Array<int> &);
	void addPoint(Vector3);
	SK::Array<Vector3> getPointCloud(){return pointCloud;}
	Vector3 getmaxPoint(){return maxPoint;}
	int getmaxIndex(){return maxIndex;}
	int getSize(){return pointCloud.size();}
	Vector3 getmeanPoint(){return meanPoint;}
	Vector3 gettipPoint(){return tipPoint;}
	Vector3 getDirection();
	Eigen::Matrix3f getDirection_f();

private:
	SK::Array<Vector3> pointCloud;
	SK::Array<int> pointCloudIndex;
	Vector3 maxPoint;
	Vector3 meanPoint;
	Vector3 tipPoint;
	int maxIndex;
	int fingerIndex;
	int length;
	int width;
	float ratio;

	float maxdist;
};


