#pragma once

#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image_planar.h>
#include <EasiiSDK/Iisu.h>
#include <flann/flann.hpp>
#include <math.h> 
#include <iostream>
#include <algorithm>
#include <vector>
#include <random>
#include <windows.h>
#include "MyTools.hxx"
#include "HandPose.hxx"
#include "HandModel.hxx"

using namespace std;
using namespace pcl;
using namespace SK;
using namespace SK::Easii;
using namespace flann;

class PSO
{
public:
	PSO();
	PSO(int, int, int, int);

	void setGenerationNum(int _g) {generation_num = _g;}
	void setParticlesNum(int _p) {particles_num = _p;}
	void setM(int _m) {m = _m;}
	void setK(int _k) {k = _k;}
	int getGenerationNum() {return generation_num;}
	int getParticlesNum() {return particles_num;}
	int getM() {return m;}
	int getK() {return k;}
	float getPixmeter() {return pix_meter;}
	double getBestPoint() {return gk_point;}
	SK::Array<HandPose> getAllParticles() {return particles;}
	HandPose getBestPose() {return bestPose;}

	Index<flann::L2<float>> buildDatasetIndex(const RangeImagePlanar &, vector<float> &);
	
	void generateParticles(const HandPose &);
	void generateParticles(HandPose &, HandPose &);
	void goGeneration(const PointCloud<PointXYZRGB> &, const HandModel &, bool );
	void goGeneration_mp(const PointCloud<PointXYZRGB> &, const HandModel &, bool, bool );
	void goGeneration_full(const PointCloud<PointXYZRGB> &, const RangeImagePlanar &, const HandModel &, bool, bool );

private:
	int generation_num;
	int particles_num;
	int m;
	int k;
	Vector3 ori_orientation;
	float pix_meter;
//	HandModel model;	 useless?
	
	// Swarm information
	HandPose bestPose;				//Gk
	double gk_point;
	
	// Particles information
	SK::Array<HandPose> particles;		// xk
	SK::Array<double> curr_points;
	SK::Array<SK::Array<float>> velocity;	// yk
	SK::Array<double> pk_point;
	SK::Array<HandPose> hispose;	//Pk
	
	void PSOupdate();
};