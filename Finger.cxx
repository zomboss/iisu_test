#include <pcl/common/pca.h>
#include "Finger.hxx"

using namespace pcl;

Finger::Finger()
{

}

Finger::Finger(int I, const Vector3 &tips, const Array<Vector3> &points, const Array<int> &pointsIndex)
{
	fingerIndex = I;
	tipPoint = tips;
	pointCloud = points;
	pointCloudIndex = pointsIndex;
	float max[3] = {0.0, 0.0, 0.0}, mean[3] = {0, 0, 0};
	maxdist = 0.0;
	for(int i = 0; i < pointCloud.size(); i++)
	{
		mean[0] += pointCloud[i].x;
		mean[1] += pointCloud[i].y;
		mean[2] += pointCloud[i].z;
		if(maxdist < pow((tips.x - pointCloud[i].x), 2) + pow((tips.y - pointCloud[i].y), 2) + pow((tips.z - pointCloud[i].z), 2) )
		{
			maxdist = pow((tips.x - pointCloud[i].x), 2) + pow((tips.y - pointCloud[i].y), 2) + pow((tips.z - pointCloud[i].z), 2);
			max[0] = pointCloud[i].x;
			max[1] = pointCloud[i].y;
			max[2] = pointCloud[i].z;
			maxIndex = pointCloudIndex[i];
		}
	}
	maxPoint = Vector3(max[0], max[1], max[2]);
	meanPoint = Vector3(mean[0] / pointCloud.size(), mean[1] / pointCloud.size(), mean[2] / pointCloud.size());
}

void Finger::addPointCloud( const Array<Vector3>& newpoints, const Array<int>& newpointsIndex)
{
	float max[3] = {0.0, 0.0, 0.0}, mean[3] = {0.0, 0.0, 0.0};
	float orimaxdist = maxdist;
	int oripcsize = pointCloud.size();
	for(int i = 0; i < newpoints.size(); i++)
	{
		pointCloud.pushBack(newpoints[i]);
		pointCloudIndex.pushBack(newpointsIndex[i]);
		mean[0] += newpoints[i].x;
		mean[1] += newpoints[i].y;
		mean[2] += newpoints[i].z;
		if(maxdist < pow((tipPoint.x - newpoints[i].x), 2) + pow((tipPoint.y - newpoints[i].y), 2) + pow((tipPoint.z - newpoints[i].z), 2) )
		{
			maxdist = pow((tipPoint.x - newpoints[i].x), 2) + pow((tipPoint.y - newpoints[i].y), 2) + pow((tipPoint.z - newpoints[i].z), 2);
			max[0] = newpoints[i].x;
			max[1] = newpoints[i].y;
			max[2] = newpoints[i].z;
			maxIndex = newpointsIndex[i];
		}
	}
	if(orimaxdist != maxdist)
		maxPoint = Vector3(max[0], max[1], max[2]);
	//meanPoint = Vector3(((oripcsize * meanPoint[0] +  mean[0]) / pointCloud.size()), ((oripcsize * meanPoint[1] +  mean[1]) / pointCloud.size()), ((oripcsize * meanPoint[2] +  mean[2]) / pointCloud.size()));

}

void Finger::addPoint(Vector3)
{
	// useless now

}

Vector3 Finger::getDirection()
{
	// rebuild the finger point cloud
	PointCloud<PointXYZ>::Ptr fingercloud(new PointCloud<PointXYZ>);
	fingercloud->points.resize(pointCloud.size());
	for(int i = 0; i < pointCloud.size(); i++)
	{
		fingercloud->points[i].x = pointCloud[i][0];
		fingercloud->points[i].y = pointCloud[i][1];
		fingercloud->points[i].z = pointCloud[i][2];
	}
	PCA<PointXYZ> fingerPCA;
	fingerPCA.setInputCloud(fingercloud);
	Vector3 result = Vector3(0,0,0);
	try
	{
		Eigen::Vector3f tmp = fingerPCA.getEigenValues();
		tmp = fingerPCA.getEigenVectors() * tmp;
		result = Vector3(tmp[0], tmp[1], tmp[2]);
		// compare with mean point
		Vector3 rough = tipPoint - meanPoint;
		if(rough.dot(result) > 0)
			result = -result;
	}
	catch(InitFailedException &)
	{
//		cout << "PCA failed..." << endl;
	}
	return result;
}

Eigen::Matrix3f Finger::getDirection_f()
{
	// rebuild the finger point cloud
	PointCloud<PointXYZ>::Ptr fingercloud(new PointCloud<PointXYZ>);
	fingercloud->points.resize(pointCloud.size());
	for(int i = 0; i < pointCloud.size(); i++)
	{
		fingercloud->points[i].x = pointCloud[i][0];
		fingercloud->points[i].y = pointCloud[i][1];
		fingercloud->points[i].z = pointCloud[i][2];
	}
	PCA<PointXYZ> fingerPCA;
	fingerPCA.setInputCloud(fingercloud);
	Eigen::Matrix3f result = Eigen::Matrix3f::Identity();
	try
	{
		result = fingerPCA.getEigenVectors() * fingerPCA.getCoefficients();
	}
	catch(InitFailedException &)
	{
		cout << "PCA failed..." << endl;
	}
	return result;
}