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
#include <ctime>
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
	ICPPCA(int , int , int , const HandPose &, const MatrixXf &, const VectorXf &, const VectorXf &, const VectorXf &);

	void setTimes(int t) {times = t;}
	int getTimes() {return times;}
	void setIterations(int it) {iter = it;}
	int getIterations() {return iter;}
	void setTransMatrix(MatrixXf mat) {trans_matrix = mat;}
	MatrixXf getTransMatrix() {return trans_matrix;}
	HandPose getBestPose() {return bestpose;}

	void run(const PointCloud<PointXYZRGB> &);

	HandPose pureTrans(int );

	void updateGlobal(double *);
	void updatePose(double *);

private:
	int times;
	int iter;
	int length;
	HandPose bestpose;
	MatrixXf trans_matrix;
	VectorXf mean_vector;
	VectorXf min_vector;
	VectorXf max_vector;

};

class CF_PCA_Joint
{
public:
	CF_PCA_Joint(const PointCloud<PointXYZRGB> &pc, HandPose po, const MatrixXf mat, const VectorXf vec):
	  cloud(pc), handpose(po), rev_matrix(mat), mean_vector(vec){}
	bool operator()(const double* const para_1, double* residual) const
	{
		// Assume 1 parameter!!!!
		HandModel model = HandModel();
		HandPose pose = handpose;
		
		// recover to 26-d parameters
//		clock_t start = clock();
		SK::Vector3 pos_vec = pose.getPosition();
		SK::Vector3 rot_vec = pose.getRotation();
		Eigen::VectorXf curr_pca_para;
		curr_pca_para.resize(1 + 6);
		curr_pca_para[0] = (float)*para_1;
		curr_pca_para[1] = pos_vec[0];	curr_pca_para[2] = pos_vec[1];	curr_pca_para[3] = pos_vec[2];
		curr_pca_para[4] = rot_vec[0];	curr_pca_para[5] = rot_vec[1];	curr_pca_para[6] = rot_vec[2];
		Eigen::VectorXf curr_nor_para = Eigen::VectorXf::Zero(26);
		curr_nor_para = rev_matrix * curr_pca_para + mean_vector;
//		clock_t mid_1 = clock();
		SK::Array<float> curr_nor_array = MyTools::EigentoSKVector(curr_nor_para);
		pose.setAllParameters(curr_nor_array);

		pose.applyPose(model);
//		clock_t mid_2 = clock();

		MyCostFunction costf = MyCostFunction(cloud, model);
		costf.calculate();
		residual[0] = costf.getCost();
//		clock_t end = clock();
//		cout << "section 1 = " << double(mid_1 - start) << " ms, section 2 = " << double(mid_2 - mid_1) << " ms, section 3 = " << double(end - mid_2)  << " ms" << endl;/**/


		return true;
	}
	bool operator()(const double* const para_1, const double* const para_2, double* residual) const
	{
		// Assume 2 parameter!!!!
		HandModel model = HandModel();
		HandPose pose = handpose;
		
		// recover to 26-d parameters
//		clock_t start = clock();
		SK::Vector3 pos_vec = pose.getPosition();
		SK::Vector3 rot_vec = pose.getRotation();
		Eigen::VectorXf curr_pca_para;
		curr_pca_para.resize(2 + 6);
		curr_pca_para[0] = (float)*para_1;	curr_pca_para[1] = (float)*para_2;
		curr_pca_para[2] = pos_vec[0];	curr_pca_para[3] = pos_vec[1];	curr_pca_para[4] = pos_vec[2];
		curr_pca_para[5] = rot_vec[0];	curr_pca_para[6] = rot_vec[1];	curr_pca_para[7] = rot_vec[2];
		Eigen::VectorXf curr_nor_para = Eigen::VectorXf::Zero(26);
		curr_nor_para = rev_matrix * curr_pca_para + mean_vector;
//		clock_t mid_1 = clock();
		SK::Array<float> curr_nor_array = MyTools::EigentoSKVector(curr_nor_para);
		pose.setAllParameters(curr_nor_array);

		pose.applyPose(model);
//		clock_t mid_2 = clock();

		MyCostFunction costf = MyCostFunction(cloud, model);
		costf.calculate();
		residual[0] = costf.getCost();
//		clock_t end = clock();
//		cout << "section 1 = " << double(mid_1 - start) << " ms, section 2 = " << double(mid_2 - mid_1) << " ms, section 3 = " << double(end - mid_2)  << " ms" << endl;/**/


		return true;
	}
	bool operator()(const double* const para_1, const double* const para_2, const double* const para_3, double* residual) const
	{
		// Assume 3 parameter!!!!
		HandModel model = HandModel();
		HandPose pose = handpose;
		
		// recover to 26-d parameters
		SK::Vector3 pos_vec = pose.getPosition();
		SK::Vector3 rot_vec = pose.getRotation();
		Eigen::VectorXf curr_pca_para;
		curr_pca_para.resize(3 + 6);
		curr_pca_para[0] = (float)*para_1;	curr_pca_para[1] = (float)*para_2;	curr_pca_para[2] = (float)*para_3;
		curr_pca_para[3] = pos_vec[0];	curr_pca_para[4] = pos_vec[1];	curr_pca_para[5] = pos_vec[2];
		curr_pca_para[6] = rot_vec[0];	curr_pca_para[7] = rot_vec[1];	curr_pca_para[8] = rot_vec[2];
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
	bool operator()(const double* const para_1, const double* const para_2, const double* const para_3,
					const double* const para_4, double* residual) const
	{
		// Assume 4 parameter!!!!
		HandModel model = HandModel();
		HandPose pose = handpose;
		
		// recover to 26-d parameters
		SK::Vector3 pos_vec = pose.getPosition();
		SK::Vector3 rot_vec = pose.getRotation();
		Eigen::VectorXf curr_pca_para;
		curr_pca_para.resize(4 + 6);
		curr_pca_para[0] = (float)*para_1;	curr_pca_para[1] = (float)*para_2;
		curr_pca_para[2] = (float)*para_3;	curr_pca_para[3] = (float)*para_4;
		curr_pca_para[4] = pos_vec[0];	curr_pca_para[5] = pos_vec[1];	curr_pca_para[6] = pos_vec[2];
		curr_pca_para[7] = rot_vec[0];	curr_pca_para[8] = rot_vec[1];	curr_pca_para[9] = rot_vec[2];
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
	bool operator()(const double* const para_1, const double* const para_2, const double* const para_3,
					const double* const para_4, const double* const para_5, double* residual) const
	{
		// Assume 5 parameter!!!!
		HandModel model = HandModel();
		HandPose pose = handpose;
		
		// recover to 26-d parameters
		SK::Vector3 pos_vec = pose.getPosition();
		SK::Vector3 rot_vec = pose.getRotation();
		Eigen::VectorXf curr_pca_para;
		curr_pca_para.resize(5 + 6);
		curr_pca_para[0] = (float)*para_1;	curr_pca_para[1] = (float)*para_2;	curr_pca_para[2] = (float)*para_3;
		curr_pca_para[3] = (float)*para_4;	curr_pca_para[4] = (float)*para_5;
		curr_pca_para[5] = pos_vec[0];	curr_pca_para[6] = pos_vec[1];	curr_pca_para[7] = pos_vec[2];
		curr_pca_para[8] = rot_vec[0];	curr_pca_para[9] = rot_vec[1];	curr_pca_para[10] = rot_vec[2];
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
	bool operator()(const double* const para_1, const double* const para_2, const double* const para_3, 
					const double* const para_4, const double* const para_5, const double* const para_6, double* residual) const
	{
		// Assume 6 parameter!!!!
		HandModel model = HandModel();
		HandPose pose = handpose;
		
		// recover to 26-d parameters
//		clock_t start = clock();
		SK::Vector3 pos_vec = pose.getPosition();
		SK::Vector3 rot_vec = pose.getRotation();
		Eigen::VectorXf curr_pca_para;
		curr_pca_para.resize(6 + 6);
		curr_pca_para[0] = (float)*para_1;	curr_pca_para[1] = (float)*para_2;	curr_pca_para[2] = (float)*para_3;
		curr_pca_para[3] = (float)*para_4;	curr_pca_para[4] = (float)*para_5;	curr_pca_para[5] = (float)*para_6;
		curr_pca_para[6] = pos_vec[0];	curr_pca_para[7] = pos_vec[1];	curr_pca_para[8] = pos_vec[2];
		curr_pca_para[9] = rot_vec[0];	curr_pca_para[10] = rot_vec[1];	curr_pca_para[11] = rot_vec[2];
		Eigen::VectorXf curr_nor_para = Eigen::VectorXf::Zero(26);
		curr_nor_para = rev_matrix * curr_pca_para + mean_vector;
//		clock_t mid_1 = clock();
		SK::Array<float> curr_nor_array = MyTools::EigentoSKVector(curr_nor_para);
		pose.setAllParameters(curr_nor_array);

		pose.applyPose(model);
//		clock_t mid_2 = clock();
		
		MyCostFunction costf = MyCostFunction(cloud, model);
		costf.calculate();
		residual[0] = costf.getCost();
//		clock_t end = clock();
//		cout << "section 1 = " << double(mid_1 - start) << " ms, section 2 = " << double(mid_2 - mid_1) << " ms, section 3 = " << double(end - mid_2)  << " ms" << endl;/**/

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