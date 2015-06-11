#pragma once

#include <pcl/ModelCoefficients.h>
#include <EasiiSDK/Iisu.h>
#include <math.h> 
#include <iostream>
#include <algorithm>
#include <windows.h>
//#include "MyTools.hxx"
//#include "HandPose.hxx"
#include "Sphere.hxx"

#define SPHERE_NUM 48

using namespace std;
using namespace SK;
using namespace SK::Easii;
using namespace pcl;

template <typename T>
SK::Array<T> getQuaternionfromVec(Eigen::Matrix<T, 3, 1> &vec)
{
	T sqrAngle = vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2];
	T angle = sqrt(sqrAngle);
	SK::Array<T> para;
	
	if (abs(angle) < T(0.00001))
	{
		para.pushBack(T(1.0f));
		para.pushBack(T(0.0f));
		para.pushBack(T(0.0f));
		para.pushBack(T(0.0f));
	}
	else
	{
		T halfAngle = T(0.5) * angle;
		T sinHalfAngle = sin(halfAngle);

		para.pushBack(cos(halfAngle));
		para.pushBack(sinHalfAngle * vec[0] / angle);
		para.pushBack(sinHalfAngle * vec[1] / angle);
		para.pushBack(sinHalfAngle * vec[2] / angle);
	}

	return para;
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
	SK::Array<Eigen::Matrix<T, 3, 1>> transformT(SK::Array<T> &para)
	{
		// return 48 new point positions
		SK::Array<Eigen::Matrix<T, 3, 1>> t_point_list;
		t_point_list.resize(SPHERE_NUM);
		Eigen::Translation<T, 3> global_trans = Eigen::Translation<T, 3>(para[20], para[21], para[22]);
		SK::Array<T> q_para = getQuaternionfromVec(Eigen::Matrix<T, 3, 1>(para[23], para[24], para[25]));
		Eigen::Quaternion<T> global_rot = Eigen::Quaternion<T>(q_para[0], q_para[1], q_para[2], q_para[3]);
		Eigen::Transform<T, 3, Affine> global_move = global_trans * global_rot;

		// finger without thumb
		for(int fin = 0; fin < 4; fin++)
		{
			// index spheres: 16~21, 16: first joint, 18: mid joint, 20: last joint, 21:tips
			// middle spheres: 22~27, 22: first joint, 24: mid joint, 26: last joint, 27:tips
			// ring spheres: 28~33, 28: first joint, 30: mid joint, 32: last joint, 33:tips
			// little spheres: 34~39, 34: first joint, 36: mid joint, 38: last joint, 39:tips
			int rootarray[3], tips;
			SK::Array<Eigen::Matrix<T, 3, 1>> angle;
			switch(fin)
			{
			case 0:	// index
				rootarray[0] = 16;rootarray[1] = 18;rootarray[2] = 20;tips = 21;
				angle.pushBack(Eigen::Matrix<T, 3, 1>(para[4], T(0), para[5]));
				angle.pushBack(Eigen::Matrix<T, 3, 1>(para[6], T(0), T(0)));
				angle.pushBack(Eigen::Matrix<T, 3, 1>(para[7], T(0), T(0)));
				break;
			case 1:	// middle
				rootarray[0] = 22;rootarray[1] = 24;rootarray[2] = 26;tips = 27;
				angle.pushBack(Eigen::Matrix<T, 3, 1>(para[8], T(0), para[9]));
				angle.pushBack(Eigen::Matrix<T, 3, 1>(para[10], T(0), T(0)));
				angle.pushBack(Eigen::Matrix<T, 3, 1>(para[11], T(0), T(0)));
				break;
			case 2:	// ring
				rootarray[0] = 28;rootarray[1] = 30;rootarray[2] = 32;tips = 33;
				angle.pushBack(Eigen::Matrix<T, 3, 1>(para[12], T(0), para[13]));
				angle.pushBack(Eigen::Matrix<T, 3, 1>(para[14], T(0), T(0)));
				angle.pushBack(Eigen::Matrix<T, 3, 1>(para[15], T(0), T(0)));
				break;
			case 3:	// little
				rootarray[0] = 34;rootarray[1] = 36;rootarray[2] = 38;tips = 39;
				angle.pushBack(Eigen::Matrix<T, 3, 1>(para[16], T(0), para[17]));
				angle.pushBack(Eigen::Matrix<T, 3, 1>(para[18], T(0), T(0)));
				angle.pushBack(Eigen::Matrix<T, 3, 1>(para[19], T(0), T(0)));
				break;
			}

			// initial spheres
			for(int i = 0; i < 6; i++)	// basic?
				t_point_list[16 + fin * 6 + i] = Eigen::Matrix<T, 3, 1>(T(models[16 + fin * 6 + i].getCenter()[0]), 
																		T(models[16 + fin * 6 + i].getCenter()[1]), 
																		T(models[16 + fin * 6 + i].getCenter()[2]));

			// Set last joint as root, rotate all spheres
			// And then set mid joint as root, rotate spheres
			// Finally, set the first joint as root, rotate all spheres (all movements have to move back)	
			for(int t = 2; t >= 0; t--)
			{
				// T and -T
				Eigen::Matrix<T, 3, 1> tmp_root(T(models[rootarray[t]].getCenter()[0]), T(models[rootarray[t]].getCenter()[1]), T(models[rootarray[t]].getCenter()[2])); 
				Eigen::Translation<T, 3> pre_trans = Eigen::Translation<T, 3>(-tmp_root);
				Eigen::Translation<T, 3> re_trans = Eigen::Translation<T, 3>(tmp_root);
				// R
				SK::Array<T> q_para_fin = getQuaternionfromVec(angle[t]);
				Eigen::Quaternion<T> rot = Eigen::Quaternion<T>(q_para_fin[0], q_para_fin[1], q_para_fin[2], q_para_fin[3]);
			
				// transform
				for(int i = rootarray[t]; i <= tips; i++)
					t_point_list[i] = re_trans * rot * pre_trans * t_point_list[i];
			}
		}

		// thumb
		{
			// the main axis = (2,1,-2)
			// Quaternion form:, given axis A(Ax,Ay,Az) and theta, s = sin(theta/2), w = cos(theta/2), x = s*Ax, y = s*Ay, z = s*Az
			// angle form: (below), (theta1, 0, 0), (theta2, 0, 0)
			// thumb spheres: 40~47, 40: first joint, 44: mid joint, 47: last joint, 47:tips
			T theta[3] = {T(0), T(para[2]), T(para[3])};
			int rootarray_st[3] = {40, 44, 46}, tips = 47;

			// initial spheres
			for(int i = rootarray_st[0]; i <= tips; i++)
				t_point_list[i] = Eigen::Matrix<T, 3, 1>(T(models[i].getCenter()[0]), T(models[i].getCenter()[1]), T(models[i].getCenter()[2]));


			for(int t = 2; t >= 1; t--)		// 1!!!!
			{
				// T and -T
				Eigen::Matrix<T, 3, 1> tmp_root(T(models[rootarray_st[t]].getCenter()[0]), T(models[rootarray_st[t]].getCenter()[1]), T(models[rootarray_st[t]].getCenter()[2])); 
				Eigen::Translation<T, 3> pre_trans = Eigen::Translation<T, 3>(-tmp_root);
				Eigen::Translation<T, 3> re_trans = Eigen::Translation<T, 3>(tmp_root);
				// R (Quaternion building)
				T s = sin(theta[t] / T(2.0)), w = cos(theta[t] / T(2.0));
				T x =  s * T(2) / T(3), y = s / T(3), z =  s * T(2) / T(-3);
				Eigen::Quaternion<T> rot = Eigen::Quaternion<T>(w, x, y, z);
			
				// transform
				for(int i = rootarray_st[t]; i <= tips; i++)
					t_point_list[i] = re_trans * rot * pre_trans * t_point_list[i];
			}
		
			//the first joint rotatation axis: y and z (0, phi1, phi2)
			// T and -T
			Eigen::Matrix<T, 3, 1> tmp_root_st(T(models[rootarray_st[0]].getCenter()[0]), T(models[rootarray_st[0]].getCenter()[1]), T(models[rootarray_st[0]].getCenter()[2])); 
			Eigen::Translation<T, 3> pre_trans_st = Eigen::Translation<T, 3>(-tmp_root_st);
			Eigen::Translation<T, 3> re_trans_st = Eigen::Translation<T, 3>(tmp_root_st);
			// R
			Eigen::Matrix<T, 3, 1> root_angle(T(0), para[0], para[1]);
			SK::Array<T> q_para_fin = getQuaternionfromVec(root_angle);
			Eigen::Quaternion<T> rot_st = Eigen::Quaternion<T>(q_para_fin[0], q_para_fin[1], q_para_fin[2], q_para_fin[3]);

			// transform
			for(int i = rootarray_st[0]; i <= tips; i++)
				t_point_list[i] = re_trans_st * rot_st * pre_trans_st * t_point_list[i];/**/
		}

		// palm and global transform
		for(int i = 0; i < SPHERE_NUM; i++)
		{
			// rotation first and the move
			if(i < 16)
				t_point_list[i] = Eigen::Matrix<T, 3, 1>(T(models[i].getCenter()[0]), T(models[i].getCenter()[1]), T(models[i].getCenter()[2])); 
			t_point_list[i] = global_move * t_point_list[i];
		}

		return t_point_list;
	}

	template <typename T>
	void setAllCenter(SK::Array<Eigen::Matrix<T, 3, 1>> new_center)
	{
		for(int i = 0; i < SPHERE_NUM; i++)
			models[i].setCenter(SK::Vector3(new_center[i].x(), new_center[i].y(), new_center[i].z()));
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