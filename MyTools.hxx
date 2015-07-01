#pragma once

#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image_planar.h>
#include <flann/flann.hpp>
#include <EasiiSDK/Iisu.h>
#include <math.h> 
#include <iostream>
#include <algorithm>
#include <random>
#include <vector>
#include <ctime>        // std::time
#include <cstdlib>      // std::rand, std::srand
#include <windows.h>

#define SAMPLE_NUM 256

using namespace std;
using namespace SK;
using namespace SK::Easii;
using namespace pcl;
using namespace flann;

class MyTools
{
public:
	// Data type transformation
	static SK::Vector3 PointtoVector3(PointXYZ &point);
	static SK::Vector3 PointtoVector3(PointXYZRGB &point);
	static PointXYZ vectortoPointXYZ(SK::Vector3 vec);
	static PointXYZRGB vectortoPointXYZRGB(SK::Vector3 vec);
	static SK::Vector3 EigentoSKVector(Eigen::Vector3f &vec_e);
	static SK::Array<float> EigentoSKVector(Eigen::VectorXf &vec_e);
	static Eigen::Vector3f SKtoEigenVector(SK::Vector3 &vec_s);
	static Eigen::VectorXf SKtoEigenVector(SK::Array<float> &vec_s);
	static SK::Vector4 vector3to4(SK::Vector3 vec);
	static SK::Vector3 vector4to3(SK::Vector4 vec);
	static SK::Matrix4 matrix3to4(SK::Matrix3 mat);
	static SK::Matrix3 matrix4to3(SK::Matrix4 mat);
	static SK::Matrix4 translation(SK::Vector3 pos);
	static SK::Matrix4 transVector(SK::Vector3 pos1, SK::Vector3 ori1, SK::Vector3 pos2, SK::Vector3 ori2);
	static PointXYZRGB rotatedPoint(Matrix4 rot, const PointXYZRGB &oldpoint);
	template <typename T>
	static Eigen::Matrix<T, 3, Eigen::Dynamic> PointCloudtoMatrix(const PointCloud<PointXYZRGB> &cloud, T typevar)
	{
		static Eigen::Matrix<T, 3, Eigen::Dynamic> mat;
		mat.resize(3, cloud.size());
		for(int i = 0; i < cloud.size(); i++)
			mat.col(i) = Eigen::Matrix<T, 3, 1>(T(cloud[i].x), T(cloud[i].y), T(cloud[i].z));
		return mat;
	}
	
	// Permutaion with order fixed
	static SK::Array<SK::Array<bool>> fingerChoosing(int num);

	// Random Sampling
	static SK::Array<float> perturbParameter(SK::Array<float> );
	static PointCloud<PointXYZRGB> downsampling(PointCloud<PointXYZRGB> &);
	
	// Array operations
	static SK::Array<float> subArray(const SK::Array<float> &, const SK::Array<float> &);
	static SK::Array<float> addArray(const SK::Array<float> &, const SK::Array<float> &);
	static SK::Array<float> scaArray(const SK::Array<float> &, float);

	// flann index
	static Index<flann::L2<float>> buildDatasetIndex(const RangeImagePlanar &planar, vector<float> &data, float &pix_meter);

};