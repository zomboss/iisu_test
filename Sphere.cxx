#include <pcl/common/transforms.h>
#include "Sphere.hxx"

using namespace pcl;

Sphere::Sphere()
{
	//center = Vector3(-25,-28,0);	old version
	center = Vector3(-25,-32,0);
	radius = 10;
	name = "root";
	related = NULL;
	color = Vector3(255,255,255);
}

Sphere::Sphere(Vector3 &c, float r, Sphere *s)
{
	center = c;
	radius = r;
	related = s;
	if(related != NULL)
		coherence = Vector3((center[0] - related->getCenter()[0]), (center[1] - related->getCenter()[1]), (center[2] - related->getCenter()[2]));
	else
		coherence = Vector3(0,0,0);
	color = Vector3(255,255,255);
}

Sphere::Sphere(Vector3 &c, float r, Sphere *s, string n)
{
	center = c;
	radius = r;
	related = s;
	name = n;
	if(related != NULL)
		coherence = Vector3((center[0] - related->getCenter()[0]), (center[1] - related->getCenter()[1]), (center[2] - related->getCenter()[2]));
	else
		coherence = Vector3(0,0,0);
	color = Vector3(255,255,255);
}

Sphere::Sphere(Vector3 &c, float r, Sphere *s, string n, Vector3 &co)
{
	center = c;
	radius = r;
	related = s;
	name = n;
	if(related != NULL)
		coherence = Vector3((center[0] - related->getCenter()[0]), (center[1] - related->getCenter()[1]), (center[2] - related->getCenter()[2]));
	else
		coherence = Vector3(0,0,0);
	color = co;
}

void Sphere::setCenter(Vector3 &c)
{
	center = c;
}

void Sphere::setRadius(float r)
{
	radius = r;
}