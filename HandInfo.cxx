#include <pcl/kdtree/kdtree_flann.h>
#include <string>
#include <vector>
#include <numeric>
#include "HandInfo.hxx"
#include "MyTools.hxx"

int* getColor(int index)
{
	int color[3] = {0, 0, 0};
	switch(index)
	{
		case 0:
			color[0] = 255;
			color[1] = 0;
			color[2] = 0;
			break;
		case 1:
			color[0] = 0;
			color[1] = 255;
			color[2] = 0;
			break;
		case 2:
			color[0] = 0;
			color[1] = 0;
			color[2] = 255;
			break;
		case 3:
			color[0] = 255;
			color[1] = 255;
			color[2] = 0;
			break;
		case 4:
			color[0] = 255;
			color[1] = 0;
			color[2] = 255;
			break;
		default:
			color[0] = 255;
			color[1] = 255;
			color[2] = 255;
			break;
	}
	return color;
}

bool getDistance(PointXYZRGB &tips, PointXYZRGB &point, int radius)
{
	return radius > sqrt(pow((tips.x - point.x), 2) + pow((tips.y - point.y), 2) + pow((tips.z - point.z), 2));
}

bool getDistance(Vector3 tips, PointXYZRGB &point, int radius)
{
	return radius > sqrt(pow((tips[0] - point.x), 2) + pow((tips[1] - point.y), 2) + pow((tips[2] - point.z), 2));
}

template <typename T>
vector<int> ordered(vector<T> const& values) {
    vector<int> indices(values.size());
    iota(begin(indices), end(indices), static_cast<int>(0));

    sort(
        begin(indices), end(indices),
        [&](size_t a, size_t b) { return values[a] < values[b]; }
    );
    return indices;
}

HandInfo::HandInfo()
{


}

HandInfo::HandInfo(int num, Array<Vector3> hpoints, Array<Vector3> fpoints, Vector3 &palm_str, Vector3 &palm_nor)
{
	finger_num = num;
	handpoints = hpoints;
	fingerspoints = fpoints;
	palm_ori.pushBack(palm_str * 1000);
	palm_ori.pushBack(palm_str * 1000 + palm_nor * 100);
	palm_vec = palm_nor.normalizedCopy() * 100;
	zero_point = Vector3();
}

Array<Vector3> HandInfo::getAllFingerTips()
{
	Array<Vector3> tips;
	for(size_t i = 0; i < fingers.size(); i++)
		tips.pushBack(fingers[i].gettipPoint());
	return tips;
}

Array<Vector3> HandInfo::getAllFingerDirs()
{
	Array<Vector3> oris;
	for(size_t i = 0; i < fingers.size(); i++)
		oris.pushBack(fingers[i].getDirection().normalizedCopy() * 100);
	return oris;
}

Array<int> HandInfo::getOrderedFingers()
{
	// We still have all five fingers, but we can only accept first sufficient numbers of them
	int finger_cross_compare[5][5];

	// Compute cross product
	for(int i = 0; i < finger_num; i++)
	{
		finger_cross_compare[i][i] = 0;
		for(int j = (i + 1); j < finger_num; j++)
		{
			Vector3 fin_vec1 = fingers[i].gettipPoint() - palm_ori[0];
			Vector3 fin_vec2 = fingers[j].gettipPoint() - palm_ori[0];
			Vector3 fin_cross = fin_vec1.cross(fin_vec2);
			if(palm_vec.dot(fin_cross) < 0)
			{
				finger_cross_compare[i][j] = -1;
				finger_cross_compare[j][i] = 1;
			}
			else
			{
				finger_cross_compare[i][j] = 1;
				finger_cross_compare[j][i] = -1;
			}
		}
	}

	// Order the fingers by cross
	vector<int> sum;
	sum.resize(finger_num, 0);
	for(int i = 0; i < finger_num; i++)
	{
		for(int j = 0; j < finger_num; j++)
			sum[i] += finger_cross_compare[i][j];
	}
	vector<int> indices = ordered(sum);
	return Array<int>(indices);
}

int HandInfo::getFingerNum()
{
	int count = 0;
	for(size_t i = 0; i < fingerspoints.size(); i++)
		if(isFingers(i)) count++;
	return count;
}

void HandInfo::setHandPointCloud(PointCloud<PointXYZRGB> &cloud, Calibration &cal)
{
	// Update viewer: finger points
	for(int i = 0; i < (int)fingerspoints.size(); i++)
	{
		Vector3 camerapoint = cal.worldToCamera(fingerspoints[i]);
		cloud.points[i].x = camerapoint[0] * 1000;
		cloud.points[i].y = camerapoint[1] * 1000;
		cloud.points[i].z = camerapoint[2] * 1000;
		int *color = getColor(i);
		cloud.points[i].r = color[0];
		cloud.points[i].g = color[1];
		cloud.points[i].b = color[2];
	}
	
	// Update viewer: hand points
	// Initial fingers
	initFinger(cloud, cal);

	// Iterative term
	iterFinger(cloud, cal);
}

void HandInfo::initFinger(PointCloud<PointXYZRGB> &cloud, Calibration &cal)
{
	// Array for finger initialization
	Array<Vector3> fingersPointCloud[5];
	Array<int> fingersPointIndex[5];

	// Update viewer
	for(int i = 0; i < (int)handpoints.size(); i++)
	{
		Vector3 camerapoint = cal.worldToCamera(handpoints[i]);
		cloud.points[fingerspoints.size() + i].x = camerapoint[0] * 1000;
		cloud.points[fingerspoints.size() + i].y = camerapoint[1] * 1000;
		cloud.points[fingerspoints.size() + i].z = camerapoint[2] * 1000;
		cloud.points[fingerspoints.size() + i].r = 255;
		cloud.points[fingerspoints.size() + i].g = 255;
		cloud.points[fingerspoints.size() + i].b = 255;
				
		// find finger points for initialization
		for(size_t j = 0; j < fingerspoints.size(); j++)
		{
			if(getDistance(cloud.points[j], cloud.points[fingerspoints.size() + i], RADIUS))
			{
				int *color = getColor(j);
				cloud.points[fingerspoints.size() + i].r = color[0];
				cloud.points[fingerspoints.size() + i].g = color[1];
				cloud.points[fingerspoints.size() + i].b = color[2];
						
				// add to fingersPointCloud
				Vector3 tmpPoint;
				tmpPoint[0] = camerapoint[0] * 1000;
				tmpPoint[1] = camerapoint[1] * 1000;
				tmpPoint[2] = camerapoint[2] * 1000;
				fingersPointCloud[j].pushBack(tmpPoint);
				fingersPointIndex[j].pushBack(i);
			}

		}
	}

	// Initial fingers
	for(size_t i = 0; i < fingerspoints.size(); i++)
	{
		Vector3 cameraFpoint = cal.worldToCamera(fingerspoints[i]);
		cameraFpoint[0] *= 1000;
		cameraFpoint[1] *= 1000;
		cameraFpoint[2] *= 1000;
		Finger finger = Finger(i, cameraFpoint, fingersPointCloud[i], fingersPointIndex[i]);
		fingers.pushBack(finger);
	}
}

void HandInfo::iterFinger(PointCloud<PointXYZRGB> &cloud, Calibration &cal)
{
	for(int t = 0; t < ITER; t++)
	{
		Array<Vector3> fingersPointCloud_Iter[5];
		Array<int> fingersPointIndex_Iter[5];
		for(size_t i = 0; i < handpoints.size(); i++)
		{
			Vector3 camerapoint = cal.worldToCamera(handpoints[i]);
			for(size_t j = 0; j < fingerspoints.size(); j++)
			{
				// get the farest point
				Vector3 maxp = fingers[j].getmaxPoint();
				if(getDistance(maxp, cloud.points[fingerspoints.size() + i], RADIUS))
				{
					int *color = getColor(j);
					cloud.points[fingerspoints.size() + i].r = color[0];
					cloud.points[fingerspoints.size() + i].g = color[1];
					cloud.points[fingerspoints.size() + i].b = color[2];
					// add to fingersPointCloud
					Vector3 tmpPoint;
					tmpPoint[0] = camerapoint[0] * 1000;
					tmpPoint[1] = camerapoint[1] * 1000;
					tmpPoint[2] = camerapoint[2] * 1000;
					fingersPointCloud_Iter[j].pushBack(tmpPoint);
					fingersPointIndex_Iter[j].pushBack(i);
				}
			}
		}
		for(size_t i = 0; i < fingerspoints.size(); i++)
			fingers[i].addPointCloud(fingersPointCloud_Iter[i], fingersPointIndex_Iter[i]);
	}
}

void HandInfo::setHandPointCloud(PointCloud<PointXYZRGB>::Ptr &cloudptr,Calibration &cal)
{
	// Update viewer: finger points
	for(int i = 0; i < (int)fingerspoints.size(); i++)
	{
//		Vector3 camerapoint = cal.worldToCamera(fingerspoints[i]);
		Vector3 camerapoint = fingerspoints[i];
		cloudptr->points[i].x = camerapoint[0] * 1000;
		cloudptr->points[i].y = camerapoint[1] * 1000;
		cloudptr->points[i].z = camerapoint[2] * 1000;
		int *color = getColor(i);
		cloudptr->points[i].r = color[0];
		cloudptr->points[i].g = color[1];
		cloudptr->points[i].b = color[2];
	}

	// Update viewer: hand points
	for(int i = 0; i < (int)handpoints.size(); i++)
	{
//		Vector3 camerapoint = cal.worldToCamera(handpoints[i]);
		Vector3 camerapoint = handpoints[i];
		cloudptr->points[fingerspoints.size() + i].x = camerapoint[0] * 1000;
		cloudptr->points[fingerspoints.size() + i].y = camerapoint[1] * 1000;
		cloudptr->points[fingerspoints.size() + i].z = camerapoint[2] * 1000;
		cloudptr->points[fingerspoints.size() + i].r = 255;
		cloudptr->points[fingerspoints.size() + i].g = 255;
		cloudptr->points[fingerspoints.size() + i].b = 255;
	}

	// Start finger Detection
	fingerDetection(cloudptr, cal);

}

void HandInfo::setHandPointCloud(PointCloud<PointXYZRGB>::Ptr &cloudptr, bool isrot)
{
	// Compute matrix
	Matrix4 rot;
	if(handpoints.size() == 0)
		rot = Matrix4::IDENTITY;
	else
		rot = MyTools::transVector((palm_ori[0] / 1000), palm_vec, Vector3(0, 0, 0), Vector3(0, 0, 1));
/*	cout << "ori = " << palm_ori[0] << ", vector = " << palm_vec << endl;
	cout << "matrix = " << rot << endl;*/

	// palm orientation
	if(isrot)
	{
		palm_ori[0] = rot.multiplyByPoint(palm_ori[0] / 1000) * 1000;
		palm_ori[1] = rot.multiplyByPoint(palm_ori[1] / 1000) * 1000;
		palm_vec = rot.multiplyByVector(palm_vec);
	}
	
	// Update viewer: finger points
	for(int i = 0; i < (int)fingerspoints.size(); i++)
	{
		if(isrot)	fingerspoints[i] = rot.multiplyByPoint(fingerspoints[i]);
		cloudptr->points[i].x = fingerspoints[i][0] * 1000;
		cloudptr->points[i].y = fingerspoints[i][1] * 1000;
		cloudptr->points[i].z = fingerspoints[i][2] * 1000;
		int *color = getColor(i);
		cloudptr->points[i].r = color[0];
		cloudptr->points[i].g = color[1];
		cloudptr->points[i].b = color[2];
	}

	// Update viewer: hand points
	for(int i = 0; i < (int)handpoints.size(); i++)
	{
		if(isrot)	handpoints[i] = rot.multiplyByPoint(handpoints[i]);
		cloudptr->points[fingerspoints.size() + i].x = handpoints[i][0] * 1000;
		cloudptr->points[fingerspoints.size() + i].y = handpoints[i][1] * 1000;
		cloudptr->points[fingerspoints.size() + i].z = handpoints[i][2] * 1000;
		cloudptr->points[fingerspoints.size() + i].r = 255;
		cloudptr->points[fingerspoints.size() + i].g = 255;
		cloudptr->points[fingerspoints.size() + i].b = 255;
	}

	// zero point
	if(isrot)	zero_point = rot.multiplyByPoint(zero_point) * 1000;

	// Start finger Detection
	fingerDetection(cloudptr);

}

void HandInfo::fingerDetection(PointCloud<PointXYZRGB>::Ptr &cloudptr,Calibration &cal)
{
	KdTreeFLANN<PointXYZRGB> kdtree;
	kdtree.setInputCloud(cloudptr);
	
	// Finger Initialization
	for(size_t f = 0; f < fingerspoints.size(); f++)
	{
		Array<Vector3> fingersPointCloud;
		Array<int> fingersPointIndexArray;
		vector<int> fingersPointIndex;
		vector<float> fingersPointDistance;
		
//		Vector3 cameraFpoint = cal.worldToCamera(fingerspoints[f]);
		Vector3 cameraFpoint = fingerspoints[f];
		cameraFpoint = cameraFpoint * 1000;
		PointXYZRGB curr_tip = MyTools::vectortoPointXYZRGB(cameraFpoint);
		if(kdtree.radiusSearch(curr_tip, RADIUS, fingersPointIndex, fingersPointDistance) > 0)
		{
			fingersPointIndexArray = Array<int>(fingersPointIndex);
			for(size_t i = 0; i < fingersPointIndex.size(); i++)
			{
				// tag with color
				int *color = getColor(f);
				cloudptr->points[fingersPointIndex[i]].r = color[0];
				cloudptr->points[fingersPointIndex[i]].g = color[1];
				cloudptr->points[fingersPointIndex[i]].b = color[2];

				// Put into fingersPointcloud
				fingersPointCloud.pushBack(MyTools::PointtoVector3(cloudptr->points[fingersPointIndex[i]]));
			}
		}
		Finger finger = Finger(f, cameraFpoint, fingersPointCloud, fingersPointIndexArray);
		fingers.pushBack(finger);
	}

	// Finger propagation
	for(size_t f = 0; f < fingerspoints.size(); f++)
	{
		for(int t = 0; t < ITER; t++)
		{
			Array<Vector3> fingersPointCloud;
			Array<int> fingersPointIndexArray;
			vector<int> fingersPointIndex;
			vector<float> fingersPointDistance;
			
			// get the farest point
			Vector3 maxp = fingers[f].getmaxPoint();
			PointXYZRGB curr_tip = MyTools::vectortoPointXYZRGB(maxp);
			if(kdtree.radiusSearch(curr_tip, RADIUS, fingersPointIndex, fingersPointDistance) > 0)
			{
				fingersPointIndexArray = Array<int>(fingersPointIndex);
				for(size_t i = 0; i < fingersPointIndex.size(); i++)
				{
					// tag with color
					int *color = getColor(f);
					cloudptr->points[fingersPointIndex[i]].r = color[0];
					cloudptr->points[fingersPointIndex[i]].g = color[1];
					cloudptr->points[fingersPointIndex[i]].b = color[2];

					// Put into fingersPointcloud
					fingersPointCloud.pushBack(MyTools::PointtoVector3(cloudptr->points[fingersPointIndex[i]]));
				}
			}
			fingers[f].addPointCloud(fingersPointCloud, fingersPointIndexArray);
		}
	}
}

void HandInfo::fingerDetection(PointCloud<PointXYZRGB>::Ptr &cloudptr)
{
	KdTreeFLANN<PointXYZRGB> kdtree;
	kdtree.setInputCloud(cloudptr);
	
	// Finger Initialization
	for(size_t f = 0; f < fingerspoints.size(); f++)
	{
		Array<Vector3> fingersPointCloud;
		Array<int> fingersPointIndexArray;
		vector<int> fingersPointIndex;
		vector<float> fingersPointDistance;
		
		Vector3 cameraFpoint = fingerspoints[f];
		cameraFpoint = cameraFpoint * 1000;
		PointXYZRGB curr_tip = MyTools::vectortoPointXYZRGB(cameraFpoint);
		if(kdtree.radiusSearch(curr_tip, RADIUS, fingersPointIndex, fingersPointDistance) > 0)
		{
			fingersPointIndexArray = Array<int>(fingersPointIndex);
			for(size_t i = 0; i < fingersPointIndex.size(); i++)
			{
				// tag with color
				int *color = getColor(f);
				cloudptr->points[fingersPointIndex[i]].r = color[0];
				cloudptr->points[fingersPointIndex[i]].g = color[1];
				cloudptr->points[fingersPointIndex[i]].b = color[2];

				// Put into fingersPointcloud
				fingersPointCloud.pushBack(MyTools::PointtoVector3(cloudptr->points[fingersPointIndex[i]]));
			}
		}
		Finger finger = Finger(f, cameraFpoint, fingersPointCloud, fingersPointIndexArray);
		fingers.pushBack(finger);
	}

	// Finger propagation
	for(size_t f = 0; f < fingerspoints.size(); f++)
	{
		// if no fingers here, skip
		if(fingers[f].gettipPoint()[0] == zero_point[0] && 
		   fingers[f].gettipPoint()[1] == zero_point[1] && 
		   fingers[f].gettipPoint()[2] == zero_point[2])
			continue;
		
		for(int t = 0; t < ITER; t++)
		{
			Array<Vector3> fingersPointCloud;
			Array<int> fingersPointIndexArray;
			vector<int> fingersPointIndex;
			vector<float> fingersPointDistance;
			
			// get the farest point
			Vector3 maxp = fingers[f].getmaxPoint();
			PointXYZRGB curr_tip = MyTools::vectortoPointXYZRGB(maxp);
			if(kdtree.radiusSearch(curr_tip, RADIUS, fingersPointIndex, fingersPointDistance) > 0)
			{
				fingersPointIndexArray = Array<int>(fingersPointIndex);
				for(size_t i = 0; i < fingersPointIndex.size(); i++)
				{
					// tag with color
					int *color = getColor(f);
					cloudptr->points[fingersPointIndex[i]].r = color[0];
					cloudptr->points[fingersPointIndex[i]].g = color[1];
					cloudptr->points[fingersPointIndex[i]].b = color[2];

					// Put into fingersPointcloud
					fingersPointCloud.pushBack(MyTools::PointtoVector3(cloudptr->points[fingersPointIndex[i]]));
				}
			}
			fingers[f].addPointCloud(fingersPointCloud, fingersPointIndexArray);
		}
	}
}

void HandInfo::switchtoOrigin()
{
	if(handpoints.size() == 0)	return;
	
	// Compute matrix
	Matrix4 rot = MyTools::transVector((palm_ori[0] / 1000), palm_vec, Vector3(0, 0, 0), Vector3(0, 0, 1));
	cout << "ori = " << palm_ori[0] << ", vector = " << palm_vec << endl;
	cout << "matrix = " << rot << endl;

	// palm orientation
	palm_ori[0] = rot.multiplyByPoint(palm_ori[0] / 1000) * 1000;
	palm_ori[1] = rot.multiplyByPoint(palm_ori[1] / 1000) * 1000;
	palm_vec = rot.multiplyByVector(palm_vec);
	
	// hand points
	for(size_t i = 0; i < handpoints.size(); i++)
		handpoints[i] = rot.multiplyByPoint(handpoints[i]);	

	// finger points
	for(size_t i = 0; i < fingerspoints.size(); i++)
		fingerspoints[i] = rot.multiplyByPoint(fingerspoints[i]);

	// zero point
	zero_point = rot.multiplyByPoint(zero_point) * 1000;
}

bool HandInfo::isEmpty()
{
	if(handpoints.size() != 0)
		return false;
	return true;
}

bool HandInfo::isFingers()
{
	for(size_t i = 0; i < fingerspoints.size(); i++)
	{
		Vector3 tmp_vec = fingers[i].gettipPoint();
		if(tmp_vec[0] != zero_point[0] && tmp_vec[1] != zero_point[1] && tmp_vec[2] != zero_point[2])
			return true;
	}
	return false;
}

bool HandInfo::isFingers(int index)
{
	Vector3 tmp_vec = fingers[index].gettipPoint();
	if(tmp_vec[0] != zero_point[0] && tmp_vec[1] != zero_point[1] && tmp_vec[2] != zero_point[2])
		return true;
	return false;
}

void HandInfo::showInfo()
{
	if(isFingers())
	{
		for(size_t i = 0; i < fingerspoints.size(); i++)
		{
			Vector3 pcaori = fingers[i].getDirection().normalizedCopy() * 100;
			cout << "finger " << i << " tips : " << fingers[i].gettipPoint() << endl;
			cout << "direct to : " << (fingers[i].gettipPoint() + pcaori) << endl;
		}
	}
	if(!isEmpty())
		cout << "Plam ori: " << palm_ori[0] << " -> " << palm_ori[1] << endl << endl;
		
}

void HandInfo::showInfo(boost::shared_ptr<visualization::PCLVisualizer> &viewer)
{
//	cout << "zero point = " << zero_point << endl;
	if(isFingers())
	{
		cout << "finger num = " << fingers.size() << endl;
		for(size_t i = 0; i < fingers.size(); i++)
		{
			Vector3 pcaori = fingers[i].getDirection().normalizedCopy() * 100;
			cout << "finger " << i << " tips : " << fingers[i].gettipPoint() << endl;
			cout << "direct to : " << (fingers[i].gettipPoint() + pcaori) << endl;

			// Add sphere and arrow to represent tips and finger direction
/*			string sname = "Cfinger_tip ";
			string dname = "Cfinger_dir ";
			sname = sname + to_string(static_cast<long long>(i));
			dname = dname + to_string(static_cast<long long>(i));
			int *color = getColor(i);
			if(isFingers(i))
			{
				viewer->addSphere(fingers[i].gettipPoint(), 5.0, color[0], color[1], color[2], sname);
				viewer->addArrow(fingers[i].gettipPoint(), (fingers[i].gettipPoint() + pcaori), 200, 0, 0, false, dname);
			}*/
		}
	}
	if(!isEmpty())
	{
		cout << "Plam ori: " << palm_ori[0] << " -> " << palm_ori[1] << endl << endl;

		// Add arrow to represent plam orientation
//		viewer->addArrow(palm_ori[0], palm_ori[1], 200, 200, 200, false, "Cplam_ori");
	}
		
}
