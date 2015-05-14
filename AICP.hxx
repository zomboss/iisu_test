#pragma once

#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/range_image/range_image.h>
#include <EasiiSDK/Iisu.h>
#include <flann/flann.hpp>
#include <math.h> 
#include <iostream>
#include <algorithm>
#include <vector>
#include <windows.h>
#include "MyTools.hxx"
#include "HandPose.hxx"
#include "HandModel.hxx"
#include "CostFunction.hxx"

using namespace std;
using namespace pcl;
using namespace SK;
using namespace SK::Easii;
using namespace flann;

class AICP
{
public:
	AICP();
	AICP(int , int , const HandPose &);
	
	void setTimes(int t) {times = t;}
	int getTimes() {return times;}
	void setIterations(int it) {iter = it;}
	int getIterations() {return iter;}
	HandPose getBestPose() {return bestpose;}

	void run_randomPara(const PointCloud<PointXYZRGB> &);
	void run_randomPara(PointCloud<PointXYZRGB> &, RangeImagePlanar &, float, vector<float> &, Index<flann::L2<float>> &);
	void run_specPara(const PointCloud<PointXYZRGB> &, int );
	void run_cyclePara(const PointCloud<PointXYZRGB> &);
	void run_randomJoint(const PointCloud<PointXYZRGB> &);

private:
	int times;
	int iter;
	HandPose bestpose;

	int getRandomJoint(int &, int &, double &, double &, double &);
	int getRandomPart(int &, double **, double **, double **);
	void getSpecJoint(int, int &, int &, double &, double &, double &);
	void updatePose(int , double );

};

class CF_Finger
{
public:
	CF_Finger(PointCloud<PointXYZRGB> &pc, int f, int j, HandPose p): 
		cloud(pc), finger(f), joint(j), handpose(p){isfull = false;}
	CF_Finger(PointCloud<PointXYZRGB> &pc, int f, int j, HandPose p, float pm, vector<float> &pv, Index<flann::L2<float>> *in): 
		cloud(pc), finger(f), joint(j), handpose(p), pix_meter(pm), pure_vec(pv), index(in){isfull = false;}
	bool operator()(const double* const theta, double* residual) const
	{
		// Set up the pose
		HandModel model = HandModel();
		HandPose pose = handpose;
		pose.setFingerParameter(finger, joint, (float)*theta);
		pose.applyPose(model);

		// Use Cost Function to compute residual
		if(!isfull)
		{
			MyCostFunction costf = MyCostFunction(cloud, model);
			costf.calculate();
			residual[0] = costf.getCost();
		}
/*		else
		{
			vector<float> tmp_vec = pure_vec;
			Index<flann::L2<float>> tmp_index = *index;
			MyCostFunction costf = MyCostFunction(cloud, model, planar, pix_meter, tmp_vec);
			costf.calculate(tmp_index);
			residual[0] = costf.getCost();
		}*/
		
		return true;
	}

private:
	PointCloud<PointXYZRGB> cloud;
	vector<float> pure_vec;
	Index<flann::L2<float>> *index;
	HandPose handpose;
	int finger;
	int joint;
	float pix_meter;
	bool isfull;

};

class CF_Global
{
public:
	CF_Global(PointCloud<PointXYZRGB> &pc, int p, HandPose po):
		cloud(pc), para(p), handpose(po), index(flann::Matrix<float>((new float[1]), 1, 1), KDTreeIndexParams(4)){isfull = false;}
	CF_Global(PointCloud<PointXYZRGB> &pc, int p, HandPose po, float pm, vector<float> &pv, Index<flann::L2<float>> &in): 
		cloud(pc), para(p), handpose(po), pix_meter(pm), pure_vec(pv), index(in){isfull = false;}
	bool operator()(const double* const theta, double* residual) const
	{
		HandModel model = HandModel();
		HandPose pose = handpose;
		pose.setGlobalParameter(para, (float)*theta);
		pose.applyPose(model);

		// Use Cost Function to compute residual
		if(!isfull)
		{
			MyCostFunction costf = MyCostFunction(cloud, model);
			costf.calculate();
			residual[0] = costf.getCost();
		}
/*		else
		{
			vector<float> tmp_vec = pure_vec;
			Index<flann::L2<float>> tmp_index = index;
			MyCostFunction costf = MyCostFunction(cloud, model, planar, pix_meter, tmp_vec);
			costf.calculate(tmp_index);
			residual[0] = costf.getCost();
		}*/

		return true;
	}

private:
	PointCloud<PointXYZRGB> cloud;
	vector<float> pure_vec;
	Index<flann::L2<float>> index;
	HandPose handpose;
	int para;
	float pix_meter;
	bool isfull;

};

class CF_Finger_Joint
{
public:
	CF_Finger_Joint(PointCloud<PointXYZRGB> &pc, int f, HandPose p):
		cloud(pc), finger(f), handpose(p){isfull = false;}
	bool operator()(const double* const mcp_fe, const double* const mcp_aa, const double* const pip_fe, const double* const dip_fe, double* residual) const
	{
		// Set up the pose
		HandModel model = HandModel();
		HandPose pose = handpose;
		pose.setFingerParameter(finger, 0, (float)*mcp_fe);
		pose.setFingerParameter(finger, 1, (float)*mcp_aa);
		pose.setFingerParameter(finger, 2, (float)*pip_fe);
		pose.setFingerParameter(finger, 3, (float)*dip_fe);
		pose.applyPose(model);

		// Use Cost Function to compute residual
		if(!isfull)
		{
			MyCostFunction costf = MyCostFunction(cloud, model);
			costf.calculate();
			residual[0] = costf.getCost();
		}

		return true;
	}

private:
	PointCloud<PointXYZRGB> cloud;
	HandPose handpose;
	int finger;
	bool isfull;
};

class CF_Global_Comb
{
public:
	CF_Global_Comb(PointCloud<PointXYZRGB> &pc, bool ch, HandPose p):
		cloud(pc), rot_pos(ch), handpose(p){isfull = false;}
	bool operator()(const double* const theta_x, const double* const theta_y, const double* const theta_z, double* residual) const
	{
		HandModel model = HandModel();
		HandPose pose = handpose;
		if(rot_pos)	pose.setRotation(Vector3(*theta_x, *theta_y, *theta_z));
		else		pose.setPosition(Vector3(*theta_x, *theta_y, *theta_z));
		pose.applyPose(model);

		// Use Cost Function to compute residual
		if(!isfull)
		{
			MyCostFunction costf = MyCostFunction(cloud, model);
			costf.calculate();
			residual[0] = costf.getCost();
		}


		return true;
	}

private:
	PointCloud<PointXYZRGB> cloud;
	HandPose handpose;
	bool rot_pos;
	bool isfull;


};