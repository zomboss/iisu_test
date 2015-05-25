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
#include "DataDriven.hxx"
#include "CostFunction.hxx"

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace SK;
using namespace SK::Easii;
using namespace flann;

class ICPPCA
{
public:
	ICPPCA();
	ICPPCA(int , int , const HandPose &, const MatrixXf &, const VectorXf &);

	void setTimes(int t) {times = t;}
	int getTimes() {return times;}
	void setIterations(int it) {iter = it;}
	int getIterations() {return iter;}
	void setTransMatrix(MatrixXf mat) {trans_matrix = mat;}
	MatrixXf getTransMatrix() {return trans_matrix;}
	HandPose getBestPose() {return bestpose;}

	void run(const PointCloud<PointXYZRGB> &);

private:
	int times;
	int iter;
	HandPose bestpose;
	MatrixXf trans_matrix;
	VectorXf mean_vector;

	void updateGlobal(double *);
	void updatePose(double *);

};

class CF_PCA_Joint
{
public:
	CF_PCA_Joint(const PointCloud<PointXYZRGB> &pc, HandPose po, const MatrixXf mat, const VectorXf vec):
	  cloud(pc), handpose(po), rev_matrix(mat), mean_vector(vec){}
	bool operator()(const double* const para_1, const double* const para_2, double* residual) const
	{
		// Assume 2 parameter!!!!
		HandModel model = HandModel();
		HandPose pose = handpose;
		
		// recover to 26-d parameters
		SK::Vector3 pos_vec = pose.getPosition();
		SK::Vector3 rot_vec = pose.getRotation();
		Eigen::VectorXf curr_pca_para;
		curr_pca_para.resize(2 + 6);
		curr_pca_para[0] = (float)*para_1;	curr_pca_para[1] = (float)*para_2;
		curr_pca_para[2] = pos_vec[0];	curr_pca_para[3] = pos_vec[1];	curr_pca_para[4] = pos_vec[2];
		curr_pca_para[5] = rot_vec[0];	curr_pca_para[6] = rot_vec[1];	curr_pca_para[7] = rot_vec[2];
		Eigen::VectorXf curr_nor_para = Eigen::VectorXf::Zero(26);
		curr_nor_para = rev_matrix * curr_pca_para + mean_vector;
		SK::Array<float> curr_nor_array = MyTools::EigentoSKVector(curr_nor_para);
		pose.setAllParameters(curr_nor_array);

		pose.applyPose(model);

		MyCostFunction costf = MyCostFunction(cloud, model);
		costf.calculate();
		residual[0] = costf.getCost();

		return true;
	}

private:
	PointCloud<PointXYZRGB> cloud;
	HandPose handpose;
	MatrixXf rev_matrix;
	VectorXf mean_vector;

};

class CF_PCA_Global
{
public:
	CF_PCA_Global(const PointCloud<PointXYZRGB> &pc, HandPose po):
	  cloud(pc), handpose(po){}
	bool operator()(const double* const pos_x, const double* const pos_y, const double* const pos_z, 
					const double* const rot_x, const double* const rot_y, const double* const rot_z, double* residual) const
	{
		HandModel model = HandModel();
		HandPose pose = handpose;
		pose.setPosition(SK::Vector3((float)*pos_x, (float)*pos_y, (float)*pos_z));
		pose.setRotation(SK::Vector3((float)*rot_x, (float)*rot_y, (float)*rot_z));
		pose.applyPose(model);

		MyCostFunction costf = MyCostFunction(cloud, model);
		costf.calculate();
		residual[0] = costf.getCost();

		return true;
	}

private:
	PointCloud<PointXYZRGB> cloud;
	HandPose handpose;

};