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
	DataDriven(char* );
	void startPCA(int );
	SK::Array<SK::Array<float>> getDataSet() {return dataset;}
	MatrixXf getCovMatrix() {return cov_matrix;}
	MatrixXf getTransMatrix();
	VectorXf getMeanVector() {return mean_vector;}
	int getDataSetSize() {return n;}

private:
	int m;	// numbers of rows (original dimensions)
	int n;	// numbers of columns (numbers of data)
	int l;	// (pc space dimensions)
	SK::Array<SK::Array<float>> dataset;
	MatrixXf cov_matrix;
	MatrixXf trans_matrix;
	VectorXf mean_vector;


};