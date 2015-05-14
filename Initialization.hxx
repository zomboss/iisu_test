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

using namespace std;
using namespace pcl;
using namespace SK;
using namespace SK::Easii;
using namespace ceres;
using namespace Eigen;

class Initialization
{
public:
	Initialization();
	Initialization(int , SK::Array<Vector3> , SK::Array<Vector3> , HandModel &);
	double getGlobal(){return global;}
	double* getThetaArray() {return theta_array;}
	double* getPhiArray() {return phi_array;}
	double getCost() {return cost;}
	Vector3 getFingerParameter(int );
	void chPCFeature(SK::Array<int> );
	void setPCFeature(SK::Array<Vector3> , SK::Array<Vector3>);
	void setResultPose(HandPose &);
	void setFullResultPose(HandPose &);
	void goInitail();
	void goInitail(HandPose &);
	void goFullInitail(HandPose &, bool );
	void fingerExist(SK::Array<bool>);

	friend class Problem;

private:
	int finger_num;
	bool finger_exist[5];
	HandModel handmodel;
	// <Order> 0:thumb, 1:index, 2:middle, 3:ring, 4:little
	SK::Array<Vector3> pc_tips;
	SK::Array<Vector3> pc_dirs;
	SK::Array<Vector3> md_tips;
	SK::Array<Vector3> md_joints;
	double theta_array[5];
	double phi_array[5];
	double cost;
	double global;
	double global_array[3];

};

class CF_dis_tips_full
{
public:
	CF_dis_tips_full(int f, Vector3 t): finger(f), point(t){global_pos = Vector3();global_ori = Vector3(0,0,1);}
	CF_dis_tips_full(int f, Vector3 t, Vector3 p, Vector3 o): finger(f), point(t), global_pos(p), global_ori(o){}
	bool operator()(const double* const theta, const double* const phi, const double* const rot_x, const double* const rot_y, const double* const rot_z, double* residual) const
	{
		HandModel model = HandModel();
		HandPose pose = HandPose();
		if(finger == 0)	// thumb
			pose.setFingerPose(finger, Vector3(0, *phi, *theta), Vector3(0,0,0), Vector3(0,0,0));
		else
			pose.setFingerPose(finger, Vector3(*theta, 0, *phi), Vector3(0,0,0), Vector3(0,0,0));
		// Add global position & orientation
		if(global_pos[0] != 0 && global_pos[1] != 0 && global_pos[2] != 0)
			pose.setPosition(global_pos);
		if(global_ori[0] != 0 && global_ori[1] != 0 && global_ori[2] != 1)
			pose.setOrientation(global_ori);
		pose.setRotation(Vector3(*rot_x, *rot_y, *rot_z));
		pose.applyPose(model);
		Vector3 tips = model.getFingerTips(finger);
		residual[0] = (double)tips.distance(point);

		return true;
	}

private:
	int finger;
	Vector3 point;
	Vector3 global_pos;
	Vector3 global_ori;
};

class CF_dir_tips_full
{
public:
	CF_dir_tips_full(int f, Vector3 d): finger(f), direction(d.normalizedCopy()){global_pos = Vector3();global_ori = Vector3(0,0,1);}
	CF_dir_tips_full(int f, Vector3 d, Vector3 p, Vector3 o): finger(f), direction(d.normalizedCopy()), global_pos(p), global_ori(o){}
	bool operator()(const double* const theta, const double* const phi, const double* const rot_x, const double* const rot_y, const double* const rot_z, double* residual) const
	{
		HandModel model = HandModel();
		HandPose pose = HandPose();
		if(finger == 0)	// thumb
			pose.setFingerPose(finger, Vector3(0, *phi, *theta), Vector3(0,0,0), Vector3(0,0,0));
		else
			pose.setFingerPose(finger, Vector3(*theta, 0, *phi), Vector3(0,0,0), Vector3(0,0,0));
		// Add global position & orientation
		if(global_pos[0] != 0 && global_pos[1] != 0 && global_pos[2] != 0)
			pose.setPosition(global_pos);
		if(global_ori[0] != 0 && global_ori[1] != 0 && global_ori[2] != 1)
			pose.setOrientation(global_ori);
		pose.setRotation(Vector3(*rot_x, *rot_y, *rot_z));
		pose.applyPose(model);
		Vector3 tips = model.getFingerTips(finger);
		Vector3 curr_dir = model.getFingerBaseJoint(finger) - tips;
		curr_dir = curr_dir.normalizedCopy();
		//direction = direction.normalizedCopy();
		residual[0] = std::acos((double)curr_dir.dot(direction));

		return true;
	}

private:
	int finger;
	Vector3 direction;
	Vector3 global_pos;
	Vector3 global_ori;

};

class CF_dis_tips
{
public:
	CF_dis_tips(int f, Vector3 t): finger(f), point(t){}
	CF_dis_tips(int f, Vector3 t, Vector3 p, Vector3 o): finger(f), point(t), global_pos(p), global_ori(o){}
	bool operator()(const double* const theta, const double* const phi, const double* const global, double* residual) const
	{
//		cout << " in dis " << finger << ", now theta = " << *theta << ", phi = " << *phi << ", global = " << *global << endl;
		HandModel model = HandModel();
		HandPose pose = HandPose();
		if(finger == 0)	// thumb
			pose.setFingerPose(finger, Vector3(0, *phi, *theta), Vector3(0,0,0), Vector3(0,0,0));
		else
			pose.setFingerPose(finger, Vector3(*theta, 0, *phi), Vector3(0,0,0), Vector3(0,0,0));
		// Add global position & orientation
		if(global_pos[0] != 0 && global_pos[1] != 0 && global_pos[2] != 0)
			pose.setPosition(global_pos);
		if(global_ori[0] != 0 && global_ori[1] != 0 && global_ori[2] != 1)
			pose.setOrientation(global_ori);
		pose.setRotation(Vector3(0, 0, *global));
		pose.applyPose(model);
		Vector3 tips = model.getFingerTips(finger);
		residual[0] = (double)tips.distance(point);
//		cout << "show me residual = " << residual[0] << endl;

		return true;
	}
	bool operator()(const double* const theta, const double* const phi, const double* const rot_x, const double* const rot_y, const double* const rot_z, double* residual) const
	{
//		cout << " in dis " << finger << ", now theta = " << *theta << ", phi = " << *phi << ", global = " << *global << endl;
		HandModel model = HandModel();
		HandPose pose = HandPose();
		if(finger == 0)	// thumb
			pose.setFingerPose(finger, Vector3(0, *phi, *theta), Vector3(0,0,0), Vector3(0,0,0));
		else
			pose.setFingerPose(finger, Vector3(*theta, 0, *phi), Vector3(0,0,0), Vector3(0,0,0));
		pose.setRotation(Vector3(*rot_x, *rot_y, *rot_z));
		pose.applyPose(model);
		Vector3 tips = model.getFingerTips(finger);
		residual[0] = (double)tips.distance(point);
//		cout << "show me residual = " << residual[0] << endl;

		return true;
	}

private:
	int finger;
	Vector3 point;
	Vector3 global_pos;
	Vector3 global_ori;
};

class CF_dir_tips
{
public:
	CF_dir_tips(int f, Vector3 d): finger(f), direction(d.normalizedCopy()){}
	CF_dir_tips(int f, Vector3 d, Vector3 p, Vector3 o): finger(f), direction(d.normalizedCopy()), global_pos(p), global_ori(o){}
	bool operator()(const double* const theta, const double* const phi, const double* const global, double* residual) const
	{
//		cout << " in dir " << finger << ", now theta = " << *theta << ", phi = " << *phi << ", global = " << *global << endl;
		HandModel model = HandModel();
		HandPose pose = HandPose();
		if(finger == 0)	// thumb
			pose.setFingerPose(finger, Vector3(0, *phi, *theta), Vector3(0,0,0), Vector3(0,0,0));
		else
			pose.setFingerPose(finger, Vector3(*theta, 0, *phi), Vector3(0,0,0), Vector3(0,0,0));
		// Add global position & orientation
		if(global_pos[0] != 0 && global_pos[1] != 0 && global_pos[2] != 0)
			pose.setPosition(global_pos);
		if(global_ori[0] != 0 && global_ori[1] != 0 && global_ori[2] != 1)
			pose.setOrientation(global_ori);
		pose.setRotation(Vector3(0, 0, *global));
		pose.applyPose(model);
		Vector3 tips = model.getFingerTips(finger);
		Vector3 curr_dir = model.getFingerBaseJoint(finger) - tips;
		curr_dir = curr_dir.normalizedCopy();
		//direction = direction.normalizedCopy();
		residual[0] = std::acos((double)curr_dir.dot(direction));
//		cout << "show me residual = " << residual[0] << endl;

		return true;
	}
	bool operator()(const double* const theta, const double* const phi, const double* const rot_x, const double* const rot_y, const double* const rot_z, double* residual) const
	{
//		cout << " in dir " << finger << ", now theta = " << *theta << ", phi = " << *phi << ", global = " << *global << endl;
		HandModel model = HandModel();
		HandPose pose = HandPose();
		if(finger == 0)	// thumb
			pose.setFingerPose(finger, Vector3(0, *phi, *theta), Vector3(0,0,0), Vector3(0,0,0));
		else
			pose.setFingerPose(finger, Vector3(*theta, 0, *phi), Vector3(0,0,0), Vector3(0,0,0));
		pose.setRotation(Vector3(*rot_x, *rot_y, *rot_z));
		pose.applyPose(model);
		Vector3 tips = model.getFingerTips(finger);
		Vector3 curr_dir = model.getFingerBaseJoint(finger) - tips;
		curr_dir = curr_dir.normalizedCopy();
		//direction = direction.normalizedCopy();
		residual[0] = std::acos((double)curr_dir.dot(direction));
//		cout << "show me residual = " << residual[0] << endl;

		return true;
	}

private:
	int finger;
	Vector3 direction;
	Vector3 global_pos;
	Vector3 global_ori;

};

class CF_x
{

public:
	CF_x(Vector3 o, Vector3 t, Vector3 p): point_o(o), point_t(t), point_p(p){}
	bool operator()(const double* const theta, const double* const phi, const double* const global, double* residual) const 
	{
		residual[0] = (point_o[0] - point_t[0]) * (std::cos(*global) * std::cos(*phi) - std::sin(*global) * std::sin(*phi)) + 
					  (-point_o[1] + point_t[1]) * (std::cos(*global) * std::sin(*phi) * std::cos(*theta) + std::sin(*global) * std::cos(*phi) * std::cos(*theta)) + 
					  (point_o[2] - point_t[2]) * (std::cos(*global) * std::sin(*phi) * std::sin(*theta) + std::sin(*global) * std::cos(*phi) * std::sin(*theta)) + 
					  point_t[0] * std::cos(*global) - point_t[1] * std::sin(*global) - point_p[0];/**/
		return true;
	}
private:
	Vector3 point_o;
	Vector3 point_t;
	Vector3 point_p;

};

class CF_y
{

public:
	CF_y(Vector3 o, Vector3 t, Vector3 p): point_o(o), point_t(t), point_p(p){}
	bool operator()(const double* const theta, const double* const phi, const double* const global, double* residual) const 
	{
		residual[0] = (point_o[0] - point_t[0]) * (std::sin(*global) * std::cos(*phi) + std::cos(*global) * std::sin(*phi)) + 
					  (-point_o[1] + point_t[1]) * (std::sin(*global) * std::sin(*phi) * std::cos(*theta) - std::cos(*global) * std::cos(*phi) * std::cos(*theta)) + 
					  (point_o[2] - point_t[2]) * (std::sin(*global) * std::sin(*phi) * std::sin(*theta) - std::cos(*global) * std::cos(*phi) * std::sin(*theta)) + 
					  point_t[0] * std::sin(*global) + point_t[1] * std::cos(*global) - point_p[1];/**/
		return true;
	}
private:
	Vector3 point_o;
	Vector3 point_t;
	Vector3 point_p;

};

class CF_z
{

public:
	CF_z(Vector3 o, Vector3 t, Vector3 p): point_o(o), point_t(t), point_p(p){}
	bool operator()(const double* const theta, const double* const phi, const double* const global, double* residual) const
	{
		residual[0] = (point_o[1] - point_t[1]) * std::sin(*theta) + (point_o[2] - point_t[2]) * std::cos(*theta) + point_t[2] - point_p[2];
		return true;
	}
private:
	Vector3 point_o;
	Vector3 point_t;
	Vector3 point_p;

};

class CF_d
{

public:
	CF_d(Vector3 o, Vector3 t, Vector3 p,  Vector3 d): point_o(o), point_t(t), point_p(p), direction(d){}
	bool operator()(const double* const theta, const double* const phi, const double* const global, double* residual) const
	{
		// get the direction now
		std::vector<double> startpoint(3, 0.0);
		startpoint[0] = (point_o[0] - point_t[0]) * (std::cos(*global) * std::cos(*phi) - std::sin(*global) * std::sin(*phi)) + 
						(-point_o[1] + point_t[1]) * (std::cos(*global) * std::sin(*phi) * std::cos(*theta) + std::sin(*global) * std::cos(*phi) * std::cos(*theta)) + 
						(point_o[2] - point_t[2]) * (std::cos(*global) * std::sin(*phi) * std::sin(*theta) + std::sin(*global) * std::cos(*phi) * std::sin(*theta)) + 
						point_t[0] * std::cos(*global) - point_t[1] * std::sin(*global);
		startpoint[1] = (point_o[0] - point_t[0]) * (std::sin(*global) * std::cos(*phi) + std::cos(*global) * std::sin(*phi)) + 
						(-point_o[1] + point_t[1]) * (std::sin(*global) * std::sin(*phi) * std::cos(*theta) - std::cos(*global) * std::cos(*phi) * std::cos(*theta)) + 
						(point_o[2] - point_t[2]) * (std::sin(*global) * std::sin(*phi) * std::sin(*theta) - std::cos(*global) * std::cos(*phi) * std::sin(*theta)) + 
						point_t[0] * std::sin(*global) + point_t[1] * std::cos(*global);
		startpoint[2] = (point_o[1] - point_t[1]) * std::sin(*theta) + (point_o[2] - point_t[2]) * std::cos(*theta) + point_t[2];
		Vector3 endpoint = point_t;
		Vector3 nowdir = endpoint;
		nowdir[0] -= startpoint[0];nowdir[1] -= startpoint[1];nowdir[2] -= startpoint[2];

		// use Eigen to calculate angle -> residual
		Vector3f ei_dir_tar(direction[0], direction[1], direction[2]);
		Vector3f ei_dir_now(nowdir[0], nowdir[1], nowdir[2]);
		ei_dir_tar.normalize();
		ei_dir_now.normalize();
		residual[0] = acos(ei_dir_tar.dot(ei_dir_now));

		return true;
	}
private:
	Vector3 point_o;
	Vector3 point_t;
	Vector3 point_p;
	Vector3 direction;

};

class CF_x_thumb
{

public:
	CF_x_thumb(Vector3 o, Vector3 t, Vector3 p): point_o(o), point_t(t), point_p(p){}
	bool operator()(const double* const theta, const double* const phi, const double* const global, double* residual) const 
	{
		residual[0] = (point_o[0] - point_t[0]) * (std::cos(*global) * std::cos(*phi) * std::cos(*theta) - std::sin(*global) * std::sin(*theta)) + 
					  (-point_o[1] + point_t[1]) * (std::cos(*global) * std::cos(*phi) * std::sin(*theta) + std::sin(*global) * std::cos(*theta)) + 
					  (-point_o[2] + point_t[2]) * (std::cos(*global) * std::sin(*phi)) + point_t[0] * std::cos(*global) - point_t[1] * std::sin(*global) - point_p[0];
		return true;
	}
private:
	Vector3 point_o;
	Vector3 point_t;
	Vector3 point_p;

};

class CF_y_thumb
{

public:
	CF_y_thumb(Vector3 o, Vector3 t, Vector3 p): point_o(o), point_t(t), point_p(p){}
	bool operator()(const double* const theta, const double* const phi, const double* const global, double* residual) const 
	{
		residual[0] = (point_o[0] - point_t[0]) * (std::sin(*global) * std::cos(*phi) * std::cos(*theta) + std::cos(*global) * std::sin(*phi)) + 
					  (-point_o[1] + point_t[1]) * (std::sin(*global) * std::cos(*phi) * std::sin(*theta) - std::cos(*global) * std::cos(*theta)) + 
					  (-point_o[2] + point_t[2]) * (std::sin(*global) * std::sin(*phi)) + point_t[0] * std::sin(*global) + point_t[1] * std::cos(*global) - point_p[1];
		return true;
	}
private:
	Vector3 point_o;
	Vector3 point_t;
	Vector3 point_p;

};

class CF_z_thumb
{

public:
	CF_z_thumb(Vector3 o, Vector3 t, Vector3 p): point_o(o), point_t(t), point_p(p){}
	bool operator()(const double* const theta, const double* const phi, const double* const global, double* residual) const
	{
//		cout << "Now theta = " << *theta << ", phi = " << *phi << ", global = " << *global << endl;
		residual[0] = (point_o[0] - point_t[0]) * std::sin(*phi) * std::cos(*theta) + 
					  (-point_o[1] + point_t[1]) * std::sin(*phi) * std::sin(*theta) + 
					  (point_o[2] - point_t[2]) * std::cos(*phi) + point_t[2] - point_p[2];
//		cout << "show me residual = " << residual[0] << endl;

		return true;
	}
private:
	Vector3 point_o;
	Vector3 point_t;
	Vector3 point_p;

};

class CF_d_thumb
{

public:
	CF_d_thumb(Vector3 o, Vector3 t, Vector3 p,  Vector3 d): point_o(o), point_t(t), point_p(p), direction(d){}
	bool operator()(const double* const theta, const double* const phi, const double* const global, double* residual) const
	{
		// get the direction now
		std::vector<double> startpoint(3, 0.0);
		startpoint[0] = (point_o[0] - point_t[0]) * (std::cos(*global) * std::cos(*phi) * std::cos(*theta) - std::sin(*global) * std::sin(*theta)) + 
						(-point_o[1] + point_t[1]) * (std::cos(*global) * std::cos(*phi) * std::sin(*theta) + std::sin(*global) * std::cos(*theta)) + 
						(-point_o[2] + point_t[2]) * (std::cos(*global) * std::sin(*phi)) + point_t[0] * std::cos(*global) - point_t[1] * std::sin(*global);
		startpoint[1] = (point_o[0] - point_t[0]) * (std::sin(*global) * std::cos(*phi) * std::cos(*theta) + std::cos(*global) * std::sin(*phi)) + 
						(-point_o[1] + point_t[1]) * (std::sin(*global) * std::cos(*phi) * std::sin(*theta) - std::cos(*global) * std::cos(*theta)) + 
						(-point_o[2] + point_t[2]) * (std::sin(*global) * std::sin(*phi)) + point_t[0] * std::sin(*global) + point_t[1] * std::cos(*global);
		startpoint[2] = (point_o[0] - point_t[0]) * std::sin(*phi) * std::cos(*theta) + 
						(-point_o[1] + point_t[1]) * std::sin(*phi) * std::sin(*theta) + 
						(point_o[2] - point_t[2]) * std::cos(*phi) + point_t[2];
		Vector3 endpoint = point_t;
		Vector3 nowdir = endpoint;
		nowdir[0] -= startpoint[0];nowdir[1] -= startpoint[1];nowdir[2] -= startpoint[2];

		// use Eigen to calculate angle -> residual
		Vector3f ei_dir_tar(direction[0], direction[1], direction[2]);
		Vector3f ei_dir_now(nowdir[0], nowdir[1], nowdir[2]);
		ei_dir_tar.normalize();
		ei_dir_now.normalize();
		residual[0] = acos(ei_dir_tar.dot(ei_dir_now));

		return true;
	}
private:
	Vector3 point_o;
	Vector3 point_t;
	Vector3 point_p;
	Vector3 direction;

};

