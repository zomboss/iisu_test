#pragma once

#include <EasiiSDK/Iisu.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include "HandPose.hxx"
#include "HandModel.hxx"

using namespace std;
using namespace Eigen;
using namespace SK;
using namespace SK::Easii;

class DataDriven
{
public:
	DataDriven();
	DataDriven(int );
	DataDriven(char* );
	void startPCA();
	void startPCA(int );
	SK::Array<SK::Array<float>> getDataSet() {return dataset;}
	MatrixXf getCovMatrix() {return cov_matrix;}
	MatrixXf getTransMatrix(){return trans_matrix;}
	MatrixXf getrevMatrix(){return trans_matrix.transpose();}
	VectorXf getMeanVector() {return mean_vector;}
	VectorXf getMaxVector() {return max_vector;}
	VectorXf getMinVector() {return min_vector;}
	int getDataSetSize() {return n;}

private:
	int m;	// numbers of rows (original dimensions)
	int n;	// numbers of columns (numbers of data)
	int l;	// (pc space dimensions)
	SK::Array<SK::Array<float>> dataset;
	MatrixXf cov_matrix;
	MatrixXf trans_matrix;	// (l + 6) * 26 matrix
	VectorXf mean_vector;	// 26-d vector with 6 zero-till
	VectorXf max_vector;	// l-d vector with 6 zero-till
	VectorXf min_vector;	// l-d vector with 6 zero-till

};