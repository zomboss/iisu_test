#pragma once

#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image_planar.h>
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
#include "CostFunctionT.hxx"

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
	AICP(int , int , const HandPose &, const HandPose &, const HandPose &);
	
	void setTimes(int t) {times = t;}
	int getTimes() {return times;}
	void setIterations(int it) {iter = it;}
	int getIterations() {return iter;}
	double getBestCost() {return cost;}
	HandPose getBestPose() {return bestpose;}

	void run_randomPara(const PointCloud<PointXYZRGB> &);
	void run_randomPara(const PointCloud<PointXYZRGB> &, const RangeImagePlanar &, float, vector<float> &, Index<flann::L2<float>> &);
	void run_globalPara(const PointCloud<PointXYZRGB> &, const RangeImagePlanar &, float, vector<float> &, Index<flann::L2<float>> &);
	void run_localPara(const PointCloud<PointXYZRGB> &, const RangeImagePlanar &, float, vector<float> &, Index<flann::L2<float>> &);
	void run_randomPara(const Eigen::Matrix<float, 3, Eigen::Dynamic> &);
	void run_specPara(const PointCloud<PointXYZRGB> &, int );
	void run_specPara(const PointCloud<PointXYZRGB> &, const RangeImagePlanar &, float, vector<float> &, Index<flann::L2<float>> &, int );
	void run_cyclePara(const PointCloud<PointXYZRGB> &);
	void run_randomJoint(const PointCloud<PointXYZRGB> &);
	void run_strategy(int ,const PointCloud<PointXYZRGB> &, const RangeImagePlanar &, float, vector<float> &, Index<flann::L2<float>> &);

private:
	int times;
	int iter;
	HandPose bestpose;
	HandPose prevpose1;
	HandPose prevpose2;
	double cost;
	bool hasprev;

	int getRandomJoint(int &, int &, double &, double &, double &);
	int getLocalJoint(int &, int &, double &, double &, double &);
	int getGlobalJoint(int &, double &);
	int getRandomPart(int &, double **, double **, double **);
	void getSpecJoint(int, int &, int &, double &, double &, double &);
	void updatePose(int , double );

};

class CF_Finger
{
public:
	CF_Finger(const PointCloud<PointXYZRGB> &pc, int f, int j, HandPose p): 
		cloud(pc), finger(f), joint(j), handpose(p){isfull = false; hasprev = false;}
	CF_Finger(const PointCloud<PointXYZRGB> &pc, int f, int j, HandPose p, HandPose pr1, HandPose pr2): 
		cloud(pc), finger(f), joint(j), handpose(p), prevpose1(pr1), prevpose2(pr2){isfull = false; hasprev = true;}
	CF_Finger(const PointCloud<PointXYZRGB> &pc, int f, int j, HandPose p, const RangeImagePlanar *pl, float pm, vector<float> &pv, Index<flann::L2<float>> *in): 
		cloud(pc), finger(f), joint(j), handpose(p), planar(pl), pix_meter(pm), index(in){pure_vec = pv; isfull = true; hasprev = false;}
	CF_Finger(const PointCloud<PointXYZRGB> &pc, int f, int j, HandPose p, HandPose pr1, HandPose pr2, const RangeImagePlanar *pl, float pm, vector<float> &pv, Index<flann::L2<float>> *in): 
		cloud(pc), finger(f), joint(j), handpose(p), prevpose1(pr1), prevpose2(pr2), planar(pl), pix_meter(pm), index(in){pure_vec = pv; isfull = true; hasprev = true;}
	template <typename T>	
	bool operator()(const double* const theta, T* residual) const
	{
//		cout << "theta = " << *theta << endl;
		// Set up the pose
		HandModel model = HandModel();
		HandPose pose = handpose;
		pose.setFingerParameter(finger, joint, (float)*theta);
		pose.applyPose(model);

		// Use Cost Function to compute residual
		if(hasprev)
		{
			if(!isfull)
			{
				MyCostFunction costf = MyCostFunction(cloud, model);
				costf.calculate(prevpose1, prevpose2);
				residual[0] = T(costf.getCost());
			}
			else
			{
				MyCostFunction costf = MyCostFunction(cloud, model, *planar, pix_meter, (vector<float>)pure_vec);
				costf.calculate(*index, prevpose1, prevpose2);
				residual[0] = T(costf.getCost());
	/*			cout << "cost in D-term = " << costf.getDTerm() << ", F-term = " << costf.getFTerm() << ", L-term = " << costf.getLTerm() << ", M-term = " << costf.getMTerm() << endl;
				cout << "total cost = " << costf.getCost() << endl;*/
			}
		}
		else
		{
			if(!isfull)
			{
				MyCostFunction costf = MyCostFunction(cloud, model);
				costf.calculate();
				residual[0] = T(costf.getCost());
			}
			else
			{
				MyCostFunction costf = MyCostFunction(cloud, model, *planar, pix_meter, (vector<float>)pure_vec);
				costf.calculate(*index);
				residual[0] = T(costf.getCost());
			}
		}
		
		return true;
	}

private:
	PointCloud<PointXYZRGB> cloud;
	const RangeImagePlanar *planar;
	vector<float> pure_vec;
	Index<flann::L2<float>> *index;
	HandPose handpose;
	HandPose prevpose1;
	HandPose prevpose2;
	int finger;
	int joint;
	float pix_meter;
	bool isfull;
	bool hasprev;
};

class CF_Global
{
public:
	CF_Global(const PointCloud<PointXYZRGB> &pc, int p, HandPose po):
		cloud(pc), para(p), handpose(po){isfull = false; hasprev = false;}
	CF_Global(const PointCloud<PointXYZRGB> &pc, int p, HandPose po, HandPose pr1, HandPose pr2): 
		cloud(pc), para(p), handpose(po), prevpose1(pr1), prevpose2(pr2){isfull = false; hasprev = true;}
	CF_Global(const PointCloud<PointXYZRGB> &pc, int p, HandPose po, const RangeImagePlanar *pl, float pm, vector<float> &pv, Index<flann::L2<float>> *in): 
		cloud(pc), para(p), handpose(po), planar(pl), pix_meter(pm), index(in){pure_vec = pv; isfull = true; hasprev = false;}
	CF_Global(const PointCloud<PointXYZRGB> &pc, int p, HandPose po, HandPose pr1, HandPose pr2, const RangeImagePlanar *pl, float pm, vector<float> &pv, Index<flann::L2<float>> *in): 
		cloud(pc), para(p), handpose(po), prevpose1(pr1), prevpose2(pr2), planar(pl), pix_meter(pm), index(in){pure_vec = pv; isfull = true; hasprev = true;}
	template <typename T>
	bool operator()(const double* const theta, T* residual) const
	{
		HandModel model = HandModel();
		HandPose pose = handpose;
		T tmp_theta = *theta;
		pose.setGlobalParameter(para, (float)*theta);
		pose.applyPose(model);

		// Use Cost Function to compute residual
		if(hasprev)
		{
			if(!isfull)
			{
				MyCostFunction costf = MyCostFunction(cloud, model);
				costf.calculate(prevpose1, prevpose2);
				residual[0] = T(costf.getCost());
			}
			else
			{
				MyCostFunction costf = MyCostFunction(cloud, model, *planar, pix_meter, (vector<float>)pure_vec);
				costf.calculate(*index, prevpose1, prevpose2);
				residual[0] = T(costf.getCost());
			}
		}
		else
		{
			if(!isfull)
			{
				MyCostFunction costf = MyCostFunction(cloud, model);
				costf.calculate();
				residual[0] = T(costf.getCost());
			}
			else
			{
				MyCostFunction costf = MyCostFunction(cloud, model, *planar, pix_meter, (vector<float>)pure_vec);
				costf.calculate(*index);
				residual[0] = T(costf.getCost());
			}
		}

		return true;
	}

private:
	PointCloud<PointXYZRGB> cloud;
	const RangeImagePlanar *planar;
	vector<float> pure_vec;
	Index<flann::L2<float>> *index;
	HandPose handpose;
	HandPose prevpose1;
	HandPose prevpose2;
	int para;
	float pix_meter;
	bool isfull;
	bool hasprev;
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

class CF_T
{
public:
	CF_T(const Eigen::Matrix<float, 3, Eigen::Dynamic> &cm, Eigen::Matrix<float, 1, 26> p, Eigen::Matrix<float, 3, 1> po, int i): 
		cloud_mat(cm), para_mat(p), pose_ori(po), index(i)
	{
		
	}
	template <typename T>
	bool operator()(const T* const theta, T* residual) const
	{
		// what!?
		Eigen::Quaternion<T> test(T(1.0), T(1.0), T(1.0), T(1.0));
		
		HandModel handmodel = HandModel();
		Eigen::Matrix<float, 1, SPHERE_NUM> radius_mat = handmodel.getRadiusMat(0.0f);
		
		// Cast to T type
		Eigen::Matrix<T, 3, Eigen::Dynamic> cloud_mat_t = cloud_mat.cast<T>();
		Eigen::Matrix<T, 1, SPHERE_NUM> radius_mat_t = radius_mat.cast<T>();
		Eigen::Matrix<T, 1, 26> para_mat_t = para_mat.cast<T>();
		Eigen::Matrix<T, 3, 1> pose_ori_t = pose_ori.cast<T>();

		// Set up the pose
		para_mat_t(0, index) = *theta;
		Eigen::Matrix<T, 3, SPHERE_NUM> model_mat_t = handmodel.transformT(para_mat_t, pose_ori_t);

		// Use Cost Function to compute residual
		CostFunctionT<T> costtf = CostFunctionT<T>(cloud_mat_t, model_mat_t, radius_mat_t);
		costtf.calculate();
		residual[0] = costtf.getCost();/**/
		
		return true;
	}

private:
	Eigen::Matrix<float, 3, Eigen::Dynamic> cloud_mat;
//	Eigen::Matrix<float, 1, SPHERE_NUM> radius_mat;
//	HandModel handmodel;
	Eigen::Matrix<float, 1, 26> para_mat;
	Eigen::Matrix<float, 3, 1> pose_ori;

	int index;
};