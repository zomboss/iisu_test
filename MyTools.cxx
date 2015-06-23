#pragma once

#include "MyTools.hxx"
#include "HandPose.hxx"

SK::Vector3 MyTools::PointtoVector3(PointXYZ &point)
{
	return Vector3(point.x, point.y, point.z);
}

SK::Vector3 MyTools::PointtoVector3(PointXYZRGB &point)
{
	return Vector3(point.x, point.y, point.z);
}

PointXYZ MyTools::vectortoPointXYZ(SK::Vector3 vec)
{
	return PointXYZ(vec[0], vec[1], vec[2]);
}

PointXYZRGB MyTools::vectortoPointXYZRGB(SK::Vector3 vec)
{
	PointXYZRGB tmpRGB = PointXYZRGB(255, 255, 255);
	tmpRGB.x = vec[0];
	tmpRGB.y = vec[1];
	tmpRGB.z = vec[2];
	return tmpRGB;
}

SK::Vector3 MyTools::EigentoSKVector(Eigen::Vector3f &vec_e)
{
	SK::Vector3 vec_s = SK::Vector3(vec_e[0], vec_e[1], vec_e[2]);
	return vec_s;
}

SK::Array<float> MyTools::EigentoSKVector(Eigen::VectorXf &vec_e)
{
	SK::Array<float> vec_s;
	for(size_t i = 0; i < vec_e.size(); i++)
		vec_s.pushBack(vec_e[i]);
	return vec_s;
}

Eigen::Vector3f MyTools::SKtoEigenVector(SK::Vector3 &vec_s)
{
	Eigen::Vector3f vec_e = Eigen::Vector3f();
	vec_e[0] = vec_s[0];
	vec_e[1] = vec_s[1];
	vec_e[2] = vec_s[2];
	return vec_e;

}

Eigen::VectorXf MyTools::SKtoEigenVector(SK::Array<float> &vec_s)
{
	Eigen::VectorXf vec_e;
	vec_e.resize(vec_s.size());
	for(size_t i = 0; i < vec_s.size(); i++)
		vec_e[i] = vec_s[i];
	
	return vec_e;
}

SK::Vector4 MyTools::vector3to4(SK::Vector3 vec)
{
	return Vector4(vec[0], vec[1], vec[2], 1);
}

SK::Vector3 MyTools::vector4to3(SK::Vector4 vec)
{
	if(vec[3] == 1)
		return Vector3(vec[0], vec[1], vec[2]);
	else
		return Vector3(0,0,0); // error happened!!!
}

SK::Matrix4 MyTools::matrix3to4(SK::Matrix3 mat)
{
	return Matrix4(mat._11, mat._21, mat._31, 0, 
				   mat._12, mat._22, mat._32, 0, 
				   mat._13, mat._23, mat._33, 0, 
				   0, 0, 0, 1);
}

SK::Matrix3 MyTools::matrix4to3(SK::Matrix4 mat)
{
	return Matrix3(mat._11, mat._21, mat._31, 
				   mat._12, mat._22, mat._32, 
				   mat._13, mat._23, mat._33);
}

SK::Matrix4 MyTools::translation(SK::Vector3 pos)
{
	return Matrix4(1, 0, 0, pos[0], 
				   0, 1, 0, pos[1], 
				   0, 0, 1, pos[2], 
				   0, 0, 0, 1);
}

SK::Matrix4 MyTools::transVector(SK::Vector3 pos1, SK::Vector3 ori1, SK::Vector3 pos2, SK::Vector3 ori2)
{
	// from 1 to 2
	ori1 = ori1.normalizedCopy();
	ori2 = ori2.normalizedCopy();
	Vector3 normal = ori2.cross(ori1);
	float len = normal.length();
	float d = ori1.dot(ori2);
	Matrix3 skewv = Matrix3(0, -normal[2], normal[1], normal[2], 0, -normal[0], -normal[1], normal[0], 0);
	Matrix3 rot = Matrix3(1,0,0,0,1,0,0,0,1) + skewv + ((1 - d) / len) * skewv * skewv;
	Matrix4 trans = MyTools::translation(pos2 - pos1) * MyTools::translation(pos1) * MyTools::matrix3to4(rot) *  MyTools::translation(-pos1);
	return trans;
}

PointXYZRGB MyTools::rotatedPoint(SK::Matrix4 rot, const PointXYZRGB &oldpoint)
{
	Vector3 tmppoint = SK::Vector3(oldpoint.x, oldpoint.y, oldpoint.z);
	tmppoint = rot.multiplyByPoint(tmppoint);
	PointXYZRGB newpoint = oldpoint;
	newpoint.x = tmppoint[0];
	newpoint.y = tmppoint[1];
	newpoint.z = tmppoint[2];
	return newpoint;
}

SK::Array<SK::Array<bool>> MyTools::fingerChoosing(int num)
{
	SK::Array<SK::Array<bool>> list;
	SK::Array<bool> option;
	switch(num)
	{
	case 5:	// 1 option
		option.assign(5, true);
		list.pushBack(option);
		break;
	case 4:	// 5 option
		for(int i = 0; i < 5; i++)
		{
			option.assign(5, true);
			option[i] = false;
			list.pushBack(option);
		}
		break;
	case 3:
		for(int i = 0; i < 4; i++)
		{
			for(int j = i + 1; j < 5; j++)
			{
				option.assign(5, true);
				option[i] = false;
				option[j] = false;
				list.pushBack(option);
			}
		}
		break;
	case 2:
		for(int i = 0; i < 4; i++)
		{
			for(int j = i + 1; j < 5; j++)
			{
				option.assign(5, false);
				option[i] = true;
				option[j] = true;
				list.pushBack(option);
			}
		}
		break;
	case 1:
		for(int i = 0; i < 5; i++)
		{
			option.assign(5, false);
			option[i] = true;
			list.pushBack(option);
		}
		break;
	case 0:
		option.assign(5, false);
		list.pushBack(option);
		break;
	default:
		cout << "Error occur in Finger Choosing" << endl;
	}

	return list;
}

Array<float> MyTools::perturbParameter(Array<float> curr_para)
{
	random_device rd;
    mt19937 gen(rd());
	Array<float> rand_para;
	for(size_t i = 0; i < curr_para.size(); i++)
	{
		if(i < 20 && i % 4 != 1)		// x-joint
		{
			normal_distribution<float> randnormal(curr_para[i], 0.4);	// angle: 0.2 arc degree ~= 5 degree
			// Check angles limitation
			float value = HandPose::validFingers((i / 5), (i % 4), randnormal(gen));
			// Concern not too far from original
			rand_para.pushBack(value);
		}
		else if(i < 20)		// z-joint
		{
			normal_distribution<float> randnormal(curr_para[i], 0.2);	// angle: 0.2 arc degree ~= 5 degree
			// Check angles limitation
			float value = HandPose::validFingers((i / 5), (i % 4), randnormal(gen));
			// Concern not too far from original
			rand_para.pushBack(value);
		}
		else if(i < 23)	// position
		{
			normal_distribution<float> randnormal(curr_para[i], 8);	// distance: 15mm
			// Check angles limitation
			float value = randnormal(gen);
			// Concern not too far from original
			rand_para.pushBack(value);
		}
		else			// rotation
		{
			normal_distribution<float> randnormal(curr_para[i], 0.2);	// angle: 0.2 arc degree ~= 5 degree
			// Check angles limitation
			float value = randnormal(gen);
			// Concern not too far from original
			rand_para.pushBack(value);
		}
	}
	return rand_para;
}

int myrandom(int i)
{
	return std::rand() % i;
}

PointCloud<PointXYZRGB> MyTools::downsampling(PointCloud<PointXYZRGB> &pointcloud)
{
	if(pointcloud.points.size() <= SAMPLE_NUM)
		return pointcloud;
	
	PointCloud<PointXYZRGB> downpointcloud;
	downpointcloud.resize((size_t)SAMPLE_NUM);

	srand(unsigned(time(0)));
	vector<int> randomseed;
	for (size_t i = 0; i < pointcloud.points.size(); ++i) randomseed.push_back(i);
	// using built-in random generator:
	random_shuffle(randomseed.begin(), randomseed.end());
	// using myrandom:
	random_shuffle(randomseed.begin(), randomseed.end(), myrandom);
	
	// Assign new point cloud
	for(size_t i = 0; i < (size_t)SAMPLE_NUM; i++)
		downpointcloud.points[i] = pointcloud.points[randomseed[i]];
	return downpointcloud;
}

Array<float> MyTools::subArray(const Array<float> &arr1, const Array<float> &arr2)
{
	Array<float> sub;
	if(arr1.size() != arr2.size())
	{
		sub.assign(1, -1);
		return sub;
	}
	for(size_t i = 0; i < arr1.size(); i++)
	{
		sub.pushBack(arr1[i] - arr2[i]);
	}
	return sub;
}

Array<float> MyTools::addArray(const Array<float> &arr1, const Array<float> &arr2)
{
	Array<float> add;
	if(arr1.size() != arr2.size())
	{
		add.assign(1, -1);
		return add;
	}
	for(size_t i = 0; i < arr1.size(); i++)
	{
		add.pushBack(arr1[i] + arr2[i]);
	}
	return add;
}

Array<float> MyTools::scaArray(const Array<float> &arr, float scalar)
{
	Array<float> sca;
	for(size_t i = 0; i < arr.size(); i++)
	{
		sca.pushBack(arr[i] * scalar);
	}
	return sca;
}