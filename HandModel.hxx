#pragma once

#include <pcl/ModelCoefficients.h>
#include <EasiiSDK/Iisu.h>
#include <math.h> 
#include <iostream>
#include <algorithm>
#include <windows.h>
//#include "MyTools.hxx"
//#include "HandPose.hxx"
#include "Sphere.hxx"

#define SPHERE_NUM 48

using namespace std;
using namespace SK;
using namespace SK::Easii;
using namespace pcl;

class HandModel
{
public:
	HandModel();

//	void changeHandPose(HandPose );
	Matrix4 movetoGlobal(Vector3, Vector3 );
	Matrix4 moveHand(Vector3, Vector3 );
	Matrix4 moveHand(Vector3 );
	void rotateHand(Vector3 );
	void bentThumb(SK::Array<Vector3> );
	void bentFinger(int , SK::Array<Vector3> );	//int fin-> 2:index, 3:middle, 4:ring, 5:little
	void WorldtoObject();
	void ObjecttoWorld(Vector3 pos, Vector3 ori, Vector3 rot);
	SK::Array<Sphere> getFullHand(){return models;}
	SK::Array<int> getAllFingersIndex();
	SK::Array<Sphere> getNeighbors(int );
	Vector3 getFingerTips(int );
	Vector3 getFingerBaseJoint(int );
	Vector3 getFingerMiddleJoint(int );
	Vector3 getFingerTopJoint(int );
	Vector3 getGlobaltrans() {return globalori;}
	Vector3 getGlobalpos() {return globalpos;}
	Vector3 getGlobalup() {return globalup;}
	SK::Array<Vector3> getFingerRotation(int );
	int getSphereSize(){return models.size();}
	int getRelated(int index){return relatedmap[index];}

	SK::Array<ModelCoefficients> getSkeleton();

private:
	SK::Array<Sphere> models;
	int relatedmap[SPHERE_NUM];

	// 26 Degree of freedom
	Vector3 globalori;		// palm orientation in world space
	Vector3 globalpos;		// palm center position in world space
	Vector3 globalup;		// hand up in world space (useless now)
	SK::Array<Vector3> thumb;
	SK::Array<Vector3> index;
	SK::Array<Vector3> mid;
	SK::Array<Vector3> ring;
	SK::Array<Vector3> little;

};