#pragma once

#include <pcl/range_image/range_image_planar.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <EasiiSDK/Iisu.h>
#include <math.h> 
#include <iostream>
#include <algorithm>
#include <vector>
#include <random>
#include <windows.h>
#include "MyTools.hxx"
#include "Sphere.hxx"
#include "HandModel.hxx"


using namespace std;
using namespace pcl;
using namespace SK;
using namespace SK::Easii;
using namespace flann;

template <typename T>
class CostFunctionT
{
public:
	CostFunctionT(){}
	CostFunctionT(const PointCloud<PointXYZRGB> &, const HandModel &)
	{
		// please don't use in real time


	}
	CostFunctionT(const Eigen::Matrix<T, 3, Eigen::Dynamic> &cloud_mat, const Eigen::Matrix<T, 3, SPHERE_NUM> &model_mat, Eigen::Matrix<T, 1, SPHERE_NUM> &r_mat)
	{
		d_term_value = T(0.0);
		f_term_value = T(0.0);
		l_term_value = T(0.0);
		pointcloud_mat = cloud_mat;
		handmodel_mat = model_mat;
		radius_mat = r_mat;
	}
	T getDTerm(){return d_term_value;}
	T getFTerm(){return f_term_value;}
	T getLTerm(){return l_term_value;}
	T getCost(){return d_term_value + f_term_value + l_term_value;}
	void resetTerm(){d_term_value = T(0.0); f_term_value = T(0.0); l_term_value = T(0.0);}
	void setDTerm()
	{
		SK::Array<int> min_list;
		for(int i = 0; i < pointcloud_mat.cols(); i++)
		{
			int min_index = -1;
			Eigen::Matrix<T, 3, 1> curr_point = pointcloud_mat.col(i);
			(handmodel_mat.colwise() - curr_point).colwise().squaredNorm().minCoeff(&min_index);
//			T tmp_value = abs((handmodel_mat.col(min_index) - curr_point).norm() - radius_mat(0, min_index));
			T tmp_value = (handmodel_mat.col(min_index) - curr_point).norm() - radius_mat(0, min_index);
			d_term_value += tmp_value * tmp_value;
			min_list.pushBack(min_index);

		}
		// lamda weight
		d_term_value *= T(SPHERE_NUM) / T(pointcloud_mat.cols());
/*		cout << "Show min list in T: " << endl;
		for(size_t i = 0; i < min_list.size(); i++)
			cout << min_list[i] << "\t";
		cout << endl;*/
	}
	void setFTerm()
	{



	}
	void setLTerm()
	{
		for(int index = 16; index < 48; index++)
		{
			Eigen::Matrix<T, 3, 1> curr_center = handmodel_mat.col(index);
			Eigen::Matrix<T, 3, Eigen::Dynamic> nei_center_mat;
			Eigen::Matrix<T, 1, Eigen::Dynamic> nei_radius_mat;
			// Get center and radius matrix
			getFingerNeighborMat(index, nei_center_mat, nei_radius_mat);
			// Get matrix with current center and division matrix
			Eigen::Matrix<T, 1, Eigen::Dynamic> curr_radius_mat, mat_2;
			curr_radius_mat.resize(1, nei_radius_mat.cols());
			mat_2.resize(1, nei_radius_mat.cols());
			curr_radius_mat.fill(radius_mat(0, index));
			mat_2.fill(T(0.5));
			// Get the valid distance
			Eigen::Matrix<T, 1, Eigen::Dynamic> colli = (nei_radius_mat + curr_radius_mat) - (nei_center_mat.colwise() - curr_center).colwise().norm();
			l_term_value += ((colli + colli.cwiseAbs()).cwiseProduct(mat_2)).sum();
/*			cout << "In index " << index << "'s collision:" << endl << colli << endl;
			cout << "cost? = " << ((colli + colli.cwiseAbs()).cwiseProduct(mat_2)).sum() << endl << endl;*/
		}
	}
	void calculate()
	{
		setDTerm();
		setFTerm();
		setLTerm();
	}


private:
	T d_term_value;
	T f_term_value;
	T l_term_value;
	Eigen::Matrix<T, 3, Eigen::Dynamic> pointcloud_mat;
	Eigen::Matrix<T, 3, SPHERE_NUM> handmodel_mat;
	Eigen::Matrix<T, 1, SPHERE_NUM> radius_mat;

	void getFingerNeighborMat(int index, Eigen::Matrix<T, 3, Eigen::Dynamic> &nei_center_mat, Eigen::Matrix<T, 1, Eigen::Dynamic> &nei_radius_mat)
	{
		Eigen::Matrix<T, 3, Eigen::Dynamic> tmp_center1, tmp_center2;
		Eigen::Matrix<T, 1, Eigen::Dynamic> tmp_radius1, tmp_radius2;
		if(index >= 40 && index < 48)			// thumb, consider index
		{
			tmp_center1 = handmodel_mat.middleCols(16, 6);
			nei_center_mat.resize(3, 6);
			nei_center_mat << tmp_center1;
			tmp_radius1 = radius_mat.middleCols(16, 6);
			nei_radius_mat.resize(1, 6);
			nei_radius_mat << tmp_radius1;
		}
		else if(index >= 16 && index < 22)		// index, consider thumb and middle
		{
			tmp_center1 = handmodel_mat.middleCols(40, 8);
			tmp_center2 = handmodel_mat.middleCols(22, 6);
			nei_center_mat.resize(3, 14);
			nei_center_mat << tmp_center1, tmp_center2;
			tmp_radius1 = radius_mat.middleCols(40, 8);
			tmp_radius2 = radius_mat.middleCols(22, 6);
			nei_radius_mat.resize(1, 14);
			nei_radius_mat << tmp_radius1, tmp_radius2;
		}
		else if(index >= 22 && index < 28)		// middle, consider index and ring
		{
			tmp_center1 = handmodel_mat.middleCols(16, 6);
			tmp_center2 = handmodel_mat.middleCols(28, 6);
			nei_center_mat.resize(3, 12);
			nei_center_mat << tmp_center1, tmp_center2;
			tmp_radius1 = radius_mat.middleCols(16, 6);
			tmp_radius2 = radius_mat.middleCols(28, 6);
			nei_radius_mat.resize(1, 12);
			nei_radius_mat << tmp_radius1, tmp_radius2;
		}
		else if(index >= 28 && index < 34)		// ring, consider middle and little
		{
			tmp_center1 = handmodel_mat.middleCols(22, 6);
			tmp_center2 = handmodel_mat.middleCols(34, 6);
			nei_center_mat.resize(3, 12);
			nei_center_mat << tmp_center1, tmp_center2;
			tmp_radius1 = radius_mat.middleCols(22, 6);
			tmp_radius2 = radius_mat.middleCols(34, 6);
			nei_radius_mat.resize(1, 12);
			nei_radius_mat << tmp_radius1, tmp_radius2;
		}
		else if(index >= 34 && index < 40)		// little, consider ring
		{
			tmp_center1 = handmodel_mat.middleCols(28, 6);
			nei_center_mat.resize(3, 6);
			nei_center_mat << tmp_center1;
			tmp_radius1 = radius_mat.middleCols(28, 6);
			nei_radius_mat.resize(1, 6);
			nei_radius_mat << tmp_radius1;
		}
	}

};
