#pragma once

#include <pcl/common/transforms.h>
#include <EasiiSDK/Iisu.h>
#include <math.h> 
#include <iostream>
#include <algorithm>
#include <vector>
#include "MyTools.hxx"
#include "Sphere.hxx"
#include "HandModel.hxx"
#include "HandPose.hxx"
#include "ceres/ceres.h"
#include "Eigen"

using namespace SK;
using namespace SK::Easii;
using namespace ceres;
using namespace Eigen;

class InitializationT
{
public:
	InitializationT();
	InitializationT(int , Eigen::Matrix<float, 3, 5> , Eigen::Matrix<float, 3, 5> );
	double* getGlobalArray(){return global_array;}
	double* getThetaArray() {return theta_array;}
	double* getPhiArray() {return phi_array;}
	double getCost() {return cost;}
	
	SK::Vector3 getFingerParameter(int );
	void setPCFeature(Eigen::Matrix<float, 3, 5> , Eigen::Matrix<float, 3, 5> );
	void setFullResultPose(HandPose &);
	void fingerExist(SK::Array<bool>);

	void goInitail(HandPose &, bool );


private:
	int finger_num;
	bool finger_exist[5];
	// <Order> 0:thumb, 1:index, 2:middle, 3:ring, 4:little
	Eigen::Matrix<float, 3, 5> pc_tips;
	Eigen::Matrix<float, 3, 5> pc_dirs;
	double theta_array[5];
	double phi_array[5];
	double global_array[3];
	double cost;

};

class CF_tips_T
{
public:
	CF_tips_T();
	CF_tips_T(int f, Eigen::Matrix<float, 3, 1> p, HandPose po):
		finger(f), point(p), pose(po){}
	template <typename T>
	bool operator()(const T* const theta, const T* const phi, const T* const rot_x, const T* const rot_y, const T* const rot_z, T* residuals) const
	{
		// what!?
		Eigen::Quaternion<T> test(T(1.0), T(1.0), T(1.0), T(1.0));
		
		HandPose tmppose = pose;
		Eigen::Matrix<T, 3, 1> m_tips;
		Eigen::Matrix<T, 3, 1> m_dirs;
		Eigen::Matrix<T, 1, 26> pose_mat = tmppose.getAllParametersT(T(0));
		switch(finger)
		{
		case 0:
			pose_mat(0, 0) = *phi;
			pose_mat(0, 1) = *theta;
			break;
		case 1:
			pose_mat(0, 4) = *theta;
			pose_mat(0, 5) = *phi;
			break;
		case 2:
			pose_mat(0, 8) = *theta;
			pose_mat(0, 9) = *phi;
			break;
		case 3:
			pose_mat(0, 12) = *theta;
			pose_mat(0, 13) = *phi;
			break;
		case 4:
			pose_mat(0, 16) = *theta;
			pose_mat(0, 17) = *phi;
			break;
		}
		pose_mat(0, 23) = *rot_x;
		pose_mat(0, 24) = *rot_y;
		pose_mat(0, 25) = *rot_z;

		HandModel model = HandModel();
		model.getSpecFingerStatus(finger, pose_mat, tmppose.getOrientation(T(0)), m_tips, m_dirs);
		Eigen::Matrix<T, 3, 1> point_T = point.cast<T>();
		
		residuals[0] = (m_tips - point_T).norm();
		return true;
	}

private:
	int finger;
	Eigen::Matrix<float, 3, 1> point;
	HandPose pose;

};

class CF_dirs_T
{
public:
	CF_dirs_T();
	CF_dirs_T(int f, Eigen::Matrix<float, 3, 1> d, HandPose po):
		finger(f), direction(d.normalized()), pose(po){}
	template <typename T>
	bool operator()(const T* const theta, const T* const phi, const T* const rot_x, const T* const rot_y, const T* const rot_z, T* residuals) const
	{
		// what!?
		Eigen::Quaternion<T> test(T(1.0), T(1.0), T(1.0), T(1.0));
		
		HandPose tmppose = pose;
		Eigen::Matrix<T, 3, 1> m_tips;
		Eigen::Matrix<T, 3, 1> m_dirs;
		Eigen::Matrix<T, 1, 26> pose_mat = tmppose.getAllParametersT(T(0));
		switch(finger)
		{
		case 0:
			pose_mat(0, 0) = *phi;
			pose_mat(0, 1) = *theta;
			break;
		case 1:
			pose_mat(0, 4) = *theta;
			pose_mat(0, 5) = *phi;
			break;
		case 2:
			pose_mat(0, 8) = *theta;
			pose_mat(0, 9) = *phi;
			break;
		case 3:
			pose_mat(0, 12) = *theta;
			pose_mat(0, 13) = *phi;
			break;
		case 4:
			pose_mat(0, 16) = *theta;
			pose_mat(0, 17) = *phi;
			break;
		}
		pose_mat(0, 23) = *rot_x;
		pose_mat(0, 24) = *rot_y;
		pose_mat(0, 25) = *rot_z;

		HandModel model = HandModel();
		model.getSpecFingerStatus(finger, pose_mat, tmppose.getOrientation(T(0)), m_tips, m_dirs);
		m_dirs.normalize();
		Eigen::Matrix<T, 3, 1> direction_T = direction.cast<T>();

		residuals[0] = ceres::acos(m_dirs.dot(direction_T));
		return true;
	}

private:
	int finger;
	Eigen::Matrix<float, 3, 1> direction;
	HandPose pose;

};