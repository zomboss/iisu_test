#pragma once

#include <EasiiSDK/Iisu.h>
#include <math.h> 
#include <iostream>
#include <algorithm>
#include <windows.h>

using namespace std;
using namespace SK;
using namespace SK::Easii;


class Sphere
{
public:
	Sphere();
	Sphere(Vector3 &c, float r, Sphere *s);
	Sphere(Vector3 &c, float r, Sphere *s, string n);
	Sphere(Vector3 &c, float r, Sphere *s, string n, Vector3 &co);

	void setCenter(Vector3 &c);
	void setRadius(float r);
	Vector3 getCenter() {return center;}
	float getRadius() {return radius;}
	Sphere *getRelatedSphere() {return related;}
	string getName() {return name;}
	Vector3 getCoherence() {return coherence;}
	Vector3 getColor() {return color;}


private:
	string name;
	Vector3 center;
	float radius;
	Vector3 coherence;
	Sphere *related;
	Vector3 color;

};
