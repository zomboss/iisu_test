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
	void startPCA(bool);
	void startPCA(int , bool);
	SK::Array<SK::Array<float>> getDataSet() {return dataset;}
	SK::Array<float> getSpecData(int index){return dataset[index];}
	MatrixXf getDataMatrix() {return data_matrix;}
	MatrixXf getCovMatrix() {return cov_matrix;}
	MatrixXf getTransMatrix(){return trans_matrix;}
	MatrixXf getrevMatrix(){return trans_matrix.transpose();}
	VectorXf getMeanVector() {return mean_vector;}
	VectorXf getMaxVector() {return max_vector;}
	VectorXf getMinVector() {return min_vector;}
	int getDataSetSize() {return n;}
	int getPCSpaceSize() {return l;}

private:
	int m;	// numbers of rows (original dimensions: 26)
	int n;	// numbers of columns (numbers of data)
	int l;	// (pc space dimensions: 2~6)
	SK::Array<SK::Array<float>> dataset;
	MatrixXf data_matrix;	// 20 * (number) matrix
	MatrixXf cov_matrix;
	MatrixXf trans_matrix;	// (l + 6) * 26 matrix
	VectorXf mean_vector;	// 26-d vector with 6 zero-till
	VectorXf max_vector;	// l-d vector with 6 zero-till
	VectorXf min_vector;	// l-d vector with 6 zero-till

	void buildMinMax();
};