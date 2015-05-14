#pragma once

#include <pcl/common/transforms.h>
#include <EasiiSDK/Iisu.h>
#include <math.h> 
#include <iostream>
#include <algorithm>
#include <windows.h>
#include "HandModel.hxx"

#define INDEX_MCP_AA_LOWERBOUND (-M_PI / 9.0)
#define INDEX_MCP_AA_UPPERBOUND (M_PI / 9.0)
#define INDEX_MCP_FE_LOWERBOUND (-M_PI / 18.0)
#define INDEX_MCP_FE_UPPERBOUND (M_PI / 2.0)
#define INDEX_PIP_FE_LOWERBOUND 0
#define INDEX_PIP_FE_UPPERBOUND (M_PI / 1.8)
#define INDEX_DIP_FE_LOWERBOUND 0
#define INDEX_DIP_FE_UPPERBOUND (M_PI / 2.0)
#define MIDDLE_MCP_AA_LOWERBOUND (-M_PI / 9.0)
#define MIDDLE_MCP_AA_UPPERBOUND (M_PI / 9.0)
#define MIDDLE_MCP_FE_LOWERBOUND (-M_PI / 18.0)
#define MIDDLE_MCP_FE_UPPERBOUND (M_PI / 2.0)
#define MIDDLE_PIP_FE_LOWERBOUND 0
#define MIDDLE_PIP_FE_UPPERBOUND (M_PI / 1.8)
#define MIDDLE_DIP_FE_LOWERBOUND 0
#define MIDDLE_DIP_FE_UPPERBOUND (M_PI / 2.0)
#define RING_MCP_AA_LOWERBOUND (-M_PI / 9.0)
#define RING_MCP_AA_UPPERBOUND (M_PI / 9.0)
#define RING_MCP_FE_LOWERBOUND (-M_PI / 18.0)
#define RING_MCP_FE_UPPERBOUND (M_PI / 2.0)
#define RING_PIP_FE_LOWERBOUND 0
#define RING_PIP_FE_UPPERBOUND (M_PI / 1.8)
#define RING_DIP_FE_LOWERBOUND 0
#define RING_DIP_FE_UPPERBOUND (M_PI / 2.0)
#define LITTLE_MCP_AA_LOWERBOUND (-M_PI / 9.0)
#define LITTLE_MCP_AA_UPPERBOUND (M_PI / 9.0)
#define LITTLE_MCP_FE_LOWERBOUND (-M_PI / 18.0)
#define LITTLE_MCP_FE_UPPERBOUND (M_PI / 2.0)
#define LITTLE_PIP_FE_LOWERBOUND 0
#define LITTLE_PIP_FE_UPPERBOUND (M_PI / 1.8)
#define LITTLE_DIP_FE_LOWERBOUND 0
#define LITTLE_DIP_FE_UPPERBOUND (M_PI / 2.0)
#define THUMB_MCP_AA_LOWERBOUND (-M_PI / 15.0)
#define THUMB_MCP_AA_UPPERBOUND (M_PI / 9.0)
#define THUMB_MCP_FE_LOWERBOUND (-M_PI / 18.0)
#define THUMB_MCP_FE_UPPERBOUND (M_PI / 2.0)
#define THUMB_PIP_FE_LOWERBOUND 0
#define THUMB_PIP_FE_UPPERBOUND (M_PI / 2.0)
#define THUMB_DIP_FE_LOWERBOUND 0
#define THUMB_DIP_FE_UPPERBOUND (M_PI / 2.0)

using namespace std;
using namespace SK;
using namespace SK::Easii;

class HandPose
{
public:
	HandPose();
	HandPose(Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3 );
	HandPose(Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3 );
	HandPose(Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3 );
//	Array<HandPose> getRandomPose(int );
	void applyPose(HandModel& );
	void applyPose_f(HandModel& );
	Array<Vector3> getThumbPose();
	Array<Vector3> getIndexPose();
	Array<Vector3> getMiddlePose();
	Array<Vector3> getRingPose();
	Array<Vector3> getLittlePose();
	Vector3 getPosition() {return global_pos;}
	Vector3 getRotation() {return global_rot;}
	Vector3 getOrientation() {return global_ori;}
	Array<float> getAllParameters();
	void setAllParameters(Array<float> );
	void setFingerPose(int , Vector3 , Vector3 , Vector3 );
	void setFingerParameter(int , int , float );
	void setGlobalParameter(int , float );
	void setPosition(Vector3 newpos) {global_pos = newpos;}
	void setRotation(Vector3 newrot) {global_rot = newrot;}
	void setOrientation(Vector3 newori) {global_ori = newori;}


	static float validFingers(int, float);
	static float validFingers(int, int, float);
	static double getBound(int, bool);

private:
	Vector3 j_thumb_base;	// 2 degrees
	Vector3 j_thumb_mid;	// 1 degree
	Vector3 j_thumb_top;	// 1 degree
	Vector3 j_index_base;	// 2 degree
	Vector3 j_index_mid;	// 1 degree
	Vector3 j_index_top;	// 1 degree
	Vector3 j_middle_base;	// 2 degree
	Vector3 j_middle_mid;	// 1 degree
	Vector3 j_middle_top;	// 1 degree
	Vector3 j_ring_base;	// 2 degree
	Vector3 j_ring_mid;		// 1 degree
	Vector3 j_ring_top;		// 1 degree
	Vector3 j_little_base;	// 2 degree
	Vector3 j_little_mid;	// 1 degree
	Vector3 j_little_top;	// 1 degree
	Vector3 global_pos;		// 3 degree		palm center position in world space
	Vector3 global_ori;		// 3 degree		palm orientation in world space
	Vector3 global_rot;		// 3 degree		rotation according to different axis
};