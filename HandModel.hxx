#pragma once

#include <pcl/ModelCoefficients.h>
#include <Eigen/Geometry>
#include <EasiiSDK/Iisu.h>
#include <math.h> 
#include <iostream>
#include <algorithm>
#include <windows.h>
#include <Eigen/src/StlSupport/StdVector.h>
#include "ceres/jet.h"
#include "Sphere.hxx"

#define SPHERE_NUM 48

using namespace std;
using namespace SK;
using namespace SK::Easii;
using namespace pcl;

template <typename T>
Eigen::Matrix<T, 1, 4> getQuaternionfromVec(Eigen::Matrix<T, 3, 1> &vec)
{
	T sqrAngle = vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2];
	T angle = ceres::sqrt(sqrAngle);
	Eigen::Matrix<T, 1, 4> para;
	
	if (ceres::abs(angle) < T(0.00001))
	{
		para(0, 0) = T(1.0f);
		para(0, 1) = T(0.0f);
		para(0, 2) = T(0.0f);
		para(0, 3) = T(0.0f);
	}
	else
	{
		T halfAngle = T(0.5) * angle;
		T sinHalfAngle = ceres::sin(halfAngle);

		para(0, 0) = ceres::cos(halfAngle);
		para(0, 1) = sinHalfAngle * vec[0] / angle;
		para(0, 2) = sinHalfAngle * vec[1] / angle;
		para(0, 3) = sinHalfAngle * vec[2] / angle;
	}

	return para;
}

template <typename T>
Eigen::Transform<T, 3, Eigen::Affine> getGlobalMovement(Eigen::Matrix<T, 1, 26> &para, Eigen::Matrix<T, 3, 1> &ori)
{
	// get tranlation and retation from parameter
	Eigen::Translation<T, 3> global_trans = Eigen::Translation<T, 3>(para(0, 20), para(0, 21), para(0, 22));
	Eigen::Matrix<T, 1, 4> q_para = getQuaternionfromVec(Eigen::Matrix<T, 3, 1>(para(0, 23), para(0, 24), para(0, 25)));
	Eigen::Quaternion<T> global_rot = Eigen::Quaternion<T>(q_para(0, 0), q_para(0, 1), q_para(0, 2), q_para(0, 3));

	// move with orientation
	Eigen::Matrix<T, 3, 1> origin;
	origin(0, 0) = T(0); origin(1, 0) = T(0); origin(2, 0) = T(1); 
	ori.normalize();
	Eigen::Matrix<T, 3, 1> normal = ori.cross(origin);
	T len = normal.norm();
	Eigen::Transform<T, 3, Eigen::Affine> global_go;
	if(len != T(0))
	{
		T d = origin.dot(ori);
		Eigen::Matrix<T, 3, 3> skewv; 
		skewv << T(0), T(-normal(2, 0)), T(normal(1, 0)), 
					T(normal(2, 0)), T(0), T(-normal(0, 0)), 
					T(-normal(1, 0)), T(normal(0 ,0)), T(0);
		Eigen::Matrix<T, 3, 3> id;
		id.setIdentity();
		Eigen::Matrix<T, 3, 3> rot = id + skewv + ((T(1) + (d * T(-1))) / len) * skewv * skewv;
		global_go = global_trans * rot;
	}
	else
		global_go = global_trans;
	Eigen::Transform<T, 3, Eigen::Affine> global_move = global_go * global_rot;

	return global_move;
}

template <typename T>
void getProcessFingerArray(int fin, Eigen::Matrix<T, 1, 26> &para, int (&r_array)[3], int (&j_array)[3], int &tip, SK::Array<Eigen::Matrix<T, 3, 1>> &angle, T typevar)
{
	// index spheres: 16~21, 12: first joint, 18: mid joint, 20: last joint, 21:tips
	// middle spheres: 22~27, 13: first joint, 24: mid joint, 26: last joint, 27:tips
	// ring spheres: 28~33, 14: first joint, 30: mid joint, 32: last joint, 33:tips
	// little spheres: 34~39, 15: first joint, 36: mid joint, 38: last joint, 39:tips
	switch(fin)
	{
	case 1:	// index
		r_array[0] = 16; r_array[1] = 18; r_array[2] = 20; tip = 21;
		j_array[0] = 12; j_array[1] = 18; j_array[2] = 20;
		angle.pushBack(Eigen::Matrix<T, 3, 1>(para(0, 4), T(0), para(0, 5)));
		angle.pushBack(Eigen::Matrix<T, 3, 1>(para(0, 6), T(0), T(0)));
		angle.pushBack(Eigen::Matrix<T, 3, 1>(para(0, 7), T(0), T(0)));
		break;
	case 2:	// middle
		r_array[0] = 22; r_array[1] = 24; r_array[2] = 26; tip = 27;
		j_array[0] = 13; j_array[1] = 24; j_array[2] = 26;
		angle.pushBack(Eigen::Matrix<T, 3, 1>(para(0, 8), T(0), para(0, 9)));
		angle.pushBack(Eigen::Matrix<T, 3, 1>(para(0, 10), T(0), T(0)));
		angle.pushBack(Eigen::Matrix<T, 3, 1>(para(0, 11), T(0), T(0)));
		break;
	case 3:	// ring
		r_array[0] = 28; r_array[1] = 30; r_array[2] = 32; tip = 33;
		j_array[0] = 14; j_array[1] = 30; j_array[2] = 32;
		angle.pushBack(Eigen::Matrix<T, 3, 1>(para(0, 12), T(0), para(0, 13)));
		angle.pushBack(Eigen::Matrix<T, 3, 1>(para(0, 14), T(0), T(0)));
		angle.pushBack(Eigen::Matrix<T, 3, 1>(para(0, 15), T(0), T(0)));
		break;
	case 4:	// little
		r_array[0] = 34; r_array[1] = 36; r_array[2] = 38; tip = 39;
		j_array[0] = 15; j_array[1] = 36; j_array[2] = 38;
		angle.pushBack(Eigen::Matrix<T, 3, 1>(para(0, 16), T(0), para(0, 17)));
		angle.pushBack(Eigen::Matrix<T, 3, 1>(para(0, 18), T(0), T(0)));
		angle.pushBack(Eigen::Matrix<T, 3, 1>(para(0, 19), T(0), T(0)));
		break;
	}
}

class HandModel
{
public:
	HandModel();

//	void changeHandPose(HandPose );
	Matrix4 movetoGlobal(Vector3, Vector3 );
	Matrix4 moveHand(Vector3, Vector3 );
	Matrix4 moveHand(Vector3 );
	void rotateHand(Vector3 );
	void bentThumb(SK::Array<Vector3> );
	void bentFinger(int , SK::Array<Vector3> );	//int fin-> 2:index, 3:middle, 4:ring, 5:little
	void WorldtoObject();
	void ObjecttoWorld(Vector3 pos, Vector3 ori, Vector3 rot);
	SK::Array<Sphere> getFullHand(){return models;}
	SK::Array<int> getAllFingersIndex();
	SK::Array<Sphere> getNeighbors(int );
	Vector3 getFingerTips(int );
	Vector3 getFingerBaseJoint(int );
	Vector3 getFingerMiddleJoint(int );
	Vector3 getFingerTopJoint(int );
	Vector3 getGlobaltrans() {return globalori;}
	Vector3 getGlobalpos() {return globalpos;}
	Vector3 getGlobalup() {return globalup;}
	SK::Array<Vector3> getFingerRotation(int );
	int getSphereSize(){return models.size();}
	int getRelated(int index){return relatedmap[index];}


	SK::Array<ModelCoefficients> getSkeleton();

	template <typename T>
	Eigen::Matrix<T, 3, SPHERE_NUM> transformT(Eigen::Matrix<T, 1, 26> &para, Eigen::Matrix<T, 3, 1> &ori)
	{
		// return 48 new point positions
		Eigen::Matrix<T, 3, SPHERE_NUM> t_point_list;
		Eigen::Transform<T, 3, Eigen::Affine> global_move = getGlobalMovement(para, ori);

		// finger without thumb
		for(int fin = 0; fin < 4; fin++)
		{
			// index spheres: 16~21, 12: first joint, 18: mid joint, 20: last joint, 21:tips
			// middle spheres: 22~27, 13: first joint, 24: mid joint, 26: last joint, 27:tips
			// ring spheres: 28~33, 14: first joint, 30: mid joint, 32: last joint, 33:tips
			// little spheres: 34~39, 15: first joint, 36: mid joint, 38: last joint, 39:tips
			int rootarray[3], jointarray[3], tips;
			SK::Array<Eigen::Matrix<T, 3, 1>> angle;
			getProcessFingerArray((fin + 1), para, rootarray, jointarray, tips, angle, T(0.0));

			// initial spheres
			for(int i = 0; i < 6; i++)	// basic?
				t_point_list.col(16 + fin * 6 + i) = Eigen::Matrix<T, 3, 1>(T(models[16 + fin * 6 + i].getCenter()[0]), 
																		T(models[16 + fin * 6 + i].getCenter()[1]), 
																		T(models[16 + fin * 6 + i].getCenter()[2]));

			// Set last joint as root, rotate all spheres
			// And then set mid joint as root, rotate spheres
			// Finally, set the first joint as root, rotate all spheres (all movements have to move back)	
			for(int t = 2; t >= 0; t--)
			{
				// T and -T
				Eigen::Matrix<T, 3, 1> tmp_root(T(models[jointarray[t]].getCenter()[0]), T(models[jointarray[t]].getCenter()[1]), T(models[jointarray[t]].getCenter()[2])); 
				Eigen::Translation<T, 3> pre_trans = Eigen::Translation<T, 3>(-tmp_root);
				Eigen::Translation<T, 3> re_trans = Eigen::Translation<T, 3>(tmp_root);
				// R
				Eigen::Matrix<T, 1, 4> q_para_fin = getQuaternionfromVec(angle[t]);
				Eigen::Quaternion<T> rot = Eigen::Quaternion<T>(q_para_fin(0, 0), q_para_fin(0, 1), q_para_fin(0, 2), q_para_fin(0, 3));
			
				// transform
				for(int i = rootarray[t]; i <= tips; i++)
					t_point_list.col(i) = re_trans * rot * pre_trans * t_point_list.col(i);
			}
		}

		// thumb
		{
			// the main axis = (2,1,-2)
			// Quaternion form:, given axis A(Ax,Ay,Az) and theta, s = sin(theta/2), w = cos(theta/2), x = s*Ax, y = s*Ay, z = s*Az
			// angle form: (below), (theta1, 0, 0), (theta2, 0, 0)
			// thumb spheres: 40~47, 40: first joint, 44: mid joint, 47: last joint, 47:tips
			T theta[3] = {T(0), T(para(0, 2)), T(para(0, 3))};
			int rootarray_st[3] = {40, 42, 45}, tips = 47;

			// initial spheres
			for(int i = rootarray_st[0]; i <= tips; i++)
				t_point_list.col(i) = Eigen::Matrix<T, 3, 1>(T(models[i].getCenter()[0]), T(models[i].getCenter()[1]), T(models[i].getCenter()[2]));


			for(int t = 2; t >= 1; t--)		// 1!!!!
			{
				// T and -T
				Eigen::Matrix<T, 3, 1> tmp_root(T(models[rootarray_st[t]].getCenter()[0]), T(models[rootarray_st[t]].getCenter()[1]), T(models[rootarray_st[t]].getCenter()[2])); 
				Eigen::Translation<T, 3> pre_trans = Eigen::Translation<T, 3>(-tmp_root);
				Eigen::Translation<T, 3> re_trans = Eigen::Translation<T, 3>(tmp_root);
				// R (Quaternion building)
				T s = ceres::sin(theta[t] / T(2.0)), w = ceres::cos(theta[t] / T(2.0));
				T x =  s * T(2) / T(3), y = s / T(3), z =  s * T(2) / T(-3);
				Eigen::Quaternion<T> rot = Eigen::Quaternion<T>(w, x, y, z);
			
				// transform
				for(int i = rootarray_st[t]; i <= tips; i++)
					t_point_list.col(i) = re_trans * rot * pre_trans * t_point_list.col(i);
			}
		
			//the first joint rotatation axis: y and z (0, phi1, phi2)
			// T and -T
			Eigen::Matrix<T, 3, 1> tmp_root_st(T(models[rootarray_st[0]].getCenter()[0]), T(models[rootarray_st[0]].getCenter()[1]), T(models[rootarray_st[0]].getCenter()[2])); 
			Eigen::Translation<T, 3> pre_trans_st = Eigen::Translation<T, 3>(-tmp_root_st);
			Eigen::Translation<T, 3> re_trans_st = Eigen::Translation<T, 3>(tmp_root_st);
			// R
			Eigen::Matrix<T, 3, 1> root_angle(T(0), para(0, 0), para(0, 1));
			Eigen::Matrix<T, 1, 4> q_para_fin = getQuaternionfromVec(root_angle);
			Eigen::Quaternion<T> rot_st = Eigen::Quaternion<T>(q_para_fin(0, 0), q_para_fin(0, 1), q_para_fin(0, 2), q_para_fin(0, 3));

			// transform
			for(int i = rootarray_st[0]; i <= tips; i++)
				t_point_list.col(i) = re_trans_st * rot_st * pre_trans_st * t_point_list.col(i);
		}

		// palm and global transform
		for(int i = 0; i < SPHERE_NUM; i++)
		{
			// rotation first and the move
			if(i < 16)
				t_point_list.col(i) = Eigen::Matrix<T, 3, 1>(T(models[i].getCenter()[0]), T(models[i].getCenter()[1]), T(models[i].getCenter()[2])); 
			t_point_list.col(i) = global_move * t_point_list.col(i);
		}

		return t_point_list;
	}
	template <typename T>
	Eigen::Matrix<T, 3, SPHERE_NUM> getAllCenterMat(T typevar)
	{
		Eigen::Matrix<T, 3, SPHERE_NUM> center_mat;
		for(int i = 0; i < SPHERE_NUM; i++)
			center_mat.col(i) = Eigen::Matrix<T, 3, 1>(T(models[i].getCenter()[0]), T(models[i].getCenter()[1]), T(models[i].getCenter()[2]));
		return center_mat;
	}
	template <typename T>
	void setAllCenter(Eigen::Matrix<T, 3, SPHERE_NUM> new_center)
	{
		for(int i = 0; i < SPHERE_NUM; i++)
			models[i].setCenter(SK::Vector3(new_center(0, i), new_center(1, i), new_center(2, i)));
	}
	template <typename T>
	Eigen::Matrix<T, 1, SPHERE_NUM> getRadiusMat(T typevar)
	{
		Eigen::Matrix<T, 1, SPHERE_NUM> radius_mat;
		for(int i = 0; i < SPHERE_NUM; i++)
			radius_mat(0, i) = T(models[i].getRadius());
		return radius_mat;
	}
	template <typename T>
	void getSpecFingerStatus(int fin, Eigen::Matrix<T, 1, 26> &para, Eigen::Matrix<T, 3, 1> &ori, Eigen::Matrix<T, 3, 1> &m_tips, Eigen::Matrix<T, 3, 1> &m_dirs)
	{
		// Get global movement
		Eigen::Transform<T, 3, Eigen::Affine> global_move = getGlobalMovement(para, ori);
		m_tips(0, 0) = T(getFingerTips(fin)[0]);
		m_tips(1, 0) = T(getFingerTips(fin)[1]);
		m_tips(2, 0) = T(getFingerTips(fin)[2]);
		m_dirs(0, 0) = T(getFingerBaseJoint(fin)[0]);
		m_dirs(1, 0) = T(getFingerBaseJoint(fin)[1]);
		m_dirs(2, 0) = T(getFingerBaseJoint(fin)[2]);
		
		if(fin == 0)	// thumb
		{
			// the main axis = (2,1,-2)
			// Quaternion form:, given axis A(Ax,Ay,Az) and theta, s = sin(theta/2), w = cos(theta/2), x = s*Ax, y = s*Ay, z = s*Az
			// angle form: (below), (theta1, 0, 0), (theta2, 0, 0)
			// thumb spheres: 40~47, 40: first joint, 44: mid joint, 47: last joint, 47:tips
			T theta[3] = {T(0), T(para(0, 2)), T(para(0, 3))};
			int rootarray_st[3] = {40, 42, 45}, tips = 47;

			for(int t = 2; t >= 1; t--)
			{
				// T and -T
				Eigen::Matrix<T, 3, 1> tmp_root(T(models[rootarray_st[t]].getCenter()[0]), T(models[rootarray_st[t]].getCenter()[1]), T(models[rootarray_st[t]].getCenter()[2])); 
				Eigen::Translation<T, 3> pre_trans = Eigen::Translation<T, 3>(-tmp_root);
				Eigen::Translation<T, 3> re_trans = Eigen::Translation<T, 3>(tmp_root);
				// R (Quaternion building)
				T s = ceres::sin(theta[t] / T(2.0)), w = ceres::cos(theta[t] / T(2.0));
				T x =  s * T(2) / T(3), y = s / T(3), z =  s * T(2) / T(-3);
				Eigen::Quaternion<T> rot = Eigen::Quaternion<T>(w, x, y, z);
			
				// transform
				m_tips = re_trans * rot * pre_trans * m_tips;
			}
			//the first joint rotatation axis: y and z (0, phi1, phi2)
			// T and -T
			Eigen::Matrix<T, 3, 1> tmp_root_st(T(models[rootarray_st[0]].getCenter()[0]), T(models[rootarray_st[0]].getCenter()[1]), T(models[rootarray_st[0]].getCenter()[2])); 
			Eigen::Translation<T, 3> pre_trans_st = Eigen::Translation<T, 3>(-tmp_root_st);
			Eigen::Translation<T, 3> re_trans_st = Eigen::Translation<T, 3>(tmp_root_st);
			// R
			Eigen::Matrix<T, 3, 1> root_angle(T(0), para(0, 0), para(0, 1));
			Eigen::Matrix<T, 1, 4> q_para_fin = getQuaternionfromVec(root_angle);
			Eigen::Quaternion<T> rot_st = Eigen::Quaternion<T>(q_para_fin(0, 0), q_para_fin(0, 1), q_para_fin(0, 2), q_para_fin(0, 3));

			// transform
			m_tips = re_trans_st * rot_st * pre_trans_st * m_tips;

			// global transform
			m_tips = global_move * m_tips;
			m_dirs = global_move * m_dirs - m_tips;

		}
		else
		{
			// Set up the root joint
			int rootarray[3], jointarray[3], tips;
			SK::Array<Eigen::Matrix<T, 3, 1>> angle;
			getProcessFingerArray(fin, para, rootarray, jointarray, tips, angle, T(0.0));
			
			// Set last joint as root, rotate all spheres
			// And then set mid joint as root, rotate spheres
			// Finally, set the first joint as root, rotate all spheres (all movements have to move back)	
			for(int t = 2; t >= 0; t--)
			{
				// T and -T
				Eigen::Matrix<T, 3, 1> tmp_root(T(models[jointarray[t]].getCenter()[0]), T(models[jointarray[t]].getCenter()[1]), T(models[jointarray[t]].getCenter()[2])); 
				Eigen::Translation<T, 3> pre_trans = Eigen::Translation<T, 3>(-tmp_root);
				Eigen::Translation<T, 3> re_trans = Eigen::Translation<T, 3>(tmp_root);
				// R
				Eigen::Matrix<T, 1, 4> q_para_fin = getQuaternionfromVec(angle[t]);
				Eigen::Quaternion<T> rot = Eigen::Quaternion<T>(q_para_fin(0, 0), q_para_fin(0, 1), q_para_fin(0, 2), q_para_fin(0, 3));
			
				// transform
				m_tips = re_trans * rot * pre_trans * m_tips;
			}

			// global transform
			m_tips = global_move * m_tips;
			m_dirs = global_move * m_dirs - m_tips;

		}
	}

private:
	SK::Array<Sphere> models;
	int relatedmap[SPHERE_NUM];

	// 26 Degree of freedom
	Vector3 globalori;		// palm orientation in world space
	Vector3 globalpos;		// palm center position in world space
	Vector3 globalup;		// hand up in world space (useless now)
	SK::Array<Vector3> thumb;
	SK::Array<Vector3> index;
	SK::Array<Vector3> mid;
	SK::Array<Vector3> ring;
	SK::Array<Vector3> little;

};