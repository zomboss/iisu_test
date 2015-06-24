#pragma once

#include <pcl/common/transforms.h>
#include <EasiiSDK/Iisu.h>
#include <math.h> 
#include <iostream>
#include <algorithm>
#include <vector>
#include <random>
#include <windows.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "MyTools.hxx"
#include "Sphere.hxx"
#include "HandModel.hxx"
#include "HandPose.hxx"


using namespace std;
using namespace pcl;
using namespace SK;
using namespace SK::Easii;
using namespace flann;
using namespace Eigen;

class MyCostFunction
{
public:
	MyCostFunction();
	MyCostFunction(const PointCloud<PointXYZRGB> &, const HandModel &);
	MyCostFunction(const PointCloud<PointXYZRGB> &, const HandModel &, const RangeImagePlanar &, float, vector<float> &);
	
	double getDTerm(){return d_term_value;}
	double getFTerm(){return f_term_value;}
	double getLTerm(){return l_term_value;}
	double getMTerm(){return m_term_value;}
	double getCost(){return (d_term_value + f_term_value + l_term_value + m_term_value);}
	void resetTerm(){d_term_value = 0.0; f_term_value = 0.0; l_term_value = 0.0; m_term_value = 0.0;}
	
	void setDTerm();
	void setDTerm_KD();
	void setDTerm_f();
	void setFTerm(flann::Index<flann::L2<float> > &);
	void setLTerm();
	void setMTerm(HandPose , HandPose );


	void calculate();
	void calculate(flann::Index<flann::L2<float> > &);
	void calculate(HandPose , HandPose );
	void calculate(flann::Index<flann::L2<float> > &, HandPose , HandPose );

private:
	double d_term_value;
	double f_term_value;
	double l_term_value;
	double m_term_value;
	PointCloud<PointXYZRGB> cloud;
	RangeImagePlanar planar;
	float pix_meter;
	HandModel model;
	KdTreeFLANN<pcl::PointXYZRGB> kdtree;

	vector<float> pure_data; //testing

	float getNearestNeighborNeighbor(flann::Index<flann::L2<float> > &, int , int );

};