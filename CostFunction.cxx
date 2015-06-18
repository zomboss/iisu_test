#include <ctime>
#include "CostFunction.hxx"

const int HEIGHT = 240;
const int WIDTH = 320;

MyCostFunction::MyCostFunction(const PointCloud<PointXYZRGB> &pc, const HandModel &hm)
{
	cloud = pc;
	model = hm;
	d_term_value = 0.0;
	f_term_value = 0.0;
	l_term_value = 0.0;
}

MyCostFunction::MyCostFunction(const PointCloud<PointXYZRGB> &pc, const HandModel &hm, const RangeImagePlanar &pl, float pm, vector<float> &data)
{
	cloud = pc;
	model = hm;
	planar = pl;
	pix_meter = pm;
	pure_data = data;
	d_term_value = 0.0;
	f_term_value = 0.0;
	l_term_value = 0.0;
}

void MyCostFunction::setDTerm()
{
	Array<Sphere> sgroup = model.getFullHand();
	for(size_t p = 0; p < cloud.size(); p++)
	{
		double min_dis = 1000000;
		for(size_t c = 0; c < sgroup.size(); c++)
		{
			Vector3 tmppoint = Vector3(cloud.points[p].x, cloud.points[p].y, cloud.points[p].z);
			double dis = tmppoint.distance(sgroup[c].getCenter());
			dis = (dis - sgroup[c].getRadius() >= 0) ? (dis - sgroup[c].getRadius()) : (sgroup[c].getRadius() - dis);
			if(min_dis > dis)
				min_dis = dis;
		}
		d_term_value += min_dis * min_dis;
	}
	// lamda weight
	d_term_value *= (double)sgroup.size() / (double)cloud.size();
}

void MyCostFunction::setDTerm_f()
{
	Array<Sphere> sgroup = model.getFullHand();
	SK::Array<size_t> min_list;
	for(size_t p = 0; p < cloud.size(); p++)
	{
		size_t min_index = 0;
		double min_dis = 1000000;
		for(size_t c = 0; c < sgroup.size(); c++)
		{
			Vector3 tmppoint = Vector3(cloud.points[p].x, cloud.points[p].y, cloud.points[p].z);
			double dis = tmppoint.distance(sgroup[c].getCenter());
			dis = (dis - sgroup[c].getRadius() >= 0) ? (dis - sgroup[c].getRadius()) : (sgroup[c].getRadius() - dis);
			if(min_dis > dis)
			{
				min_dis = dis;
				min_index = c;
			}
		}
		d_term_value += min_dis * min_dis;
		min_list.pushBack(min_index);
	}
	// lamda weight
	d_term_value *= (double)sgroup.size() / (double)cloud.size();
	cout << "Show min list in T: " << endl;
	for(size_t i = 0; i < min_list.size(); i++)
		cout << min_list[i] << "\t";
	cout << endl;
}

void MyCostFunction::setDTerm_KD()
{
	// Use KD-tree to find nearest point
	PointCloud<pcl::PointXYZRGB>::Ptr cloudptr(&cloud);
	kdtree.setInputCloud(cloudptr);
	
	Array<Sphere> sgroup = model.getFullHand();
	for(size_t c = 0; c < sgroup.size(); c++)
	{
		PointXYZRGB searchpoint;
		searchpoint.x = (sgroup[c].getCenter())[0];
		searchpoint.y = (sgroup[c].getCenter())[1];
		searchpoint.z = (sgroup[c].getCenter())[2];
		vector<int> index(1);
		vector<float> distance(1);
		if(kdtree.nearestKSearch (searchpoint, 1, index, distance) <= 0)
			cout << "some error found in kd-tree" << endl;
		double min_dis = abs(sqrt(distance[0]) - sgroup[c].getRadius());
		d_term_value += min_dis * min_dis;
	}

	// lamda weight
	d_term_value *= (double)SPHERE_NUM / (double)SAMPLE_NUM;
}

void MyCostFunction::setFTerm(flann::Index<flann::L2<float> > &index)
{
	Array<Sphere> sgroup = model.getFullHand();
	for(size_t c = 0; c < sgroup.size(); c++)
	{
		Eigen::Vector3f point = MyTools::SKtoEigenVector(sgroup[c].getCenter());
		int image_x, image_y;
		float range;
		planar.getImagePoint(point, image_x, image_y, range);
		if(planar.isValid(image_x, image_y))
		{
			float ref = planar.getPoint(image_x, image_y).range;
			if(ref > range)
				f_term_value += (double)(ref - range);
		}
		else if(!planar.isValid(image_x, image_y))
			f_term_value += getNearestNeighborNeighbor(index, image_x, image_y) * pix_meter;/**/

	}

	// lamda weight
	f_term_value *= 1.0;
}

void MyCostFunction::setLTerm()
{
	Array<Sphere> sgroup = model.getFullHand();
	Array<int> fingerlist = model.getAllFingersIndex();
	for(size_t i = 0; i < fingerlist.size(); i++)
	{
		double tmp_dis = 0;
		Sphere tmpsphere = sgroup[fingerlist[i]];
		Array<Sphere> neoghorlist = model.getNeighbors(fingerlist[i]);
		for(size_t j = 0; j < neoghorlist.size(); j++)
		{
			Sphere neisphere = neoghorlist[j];
			double tmpvalue = tmpsphere.getRadius() + neisphere.getRadius() - tmpsphere.getCenter().distance(neisphere.getCenter());
			tmp_dis = (tmpvalue > 0) ? (tmp_dis + tmpvalue) : tmp_dis;
		}
		l_term_value += tmp_dis * tmp_dis;
	}
}

void MyCostFunction::calculate()
{
	setDTerm();
//	setDTerm_KD();
	setLTerm();
}

void MyCostFunction::calculate(flann::Index<flann::L2<float> > &index)
{
	setDTerm();
	setFTerm(index);
	setLTerm();
}

float MyCostFunction::getNearestNeighborNeighbor(flann::Index<flann::L2<float> > &index, int query_x, int query_y)
{
	float *query_data = new float[2];
	query_data[0] = (float)query_x; query_data[1] = (float)query_y;

	// Initial flann
    flann::Matrix<float> query(query_data, 1, 2);
	flann::Matrix<int> indices(new int[query.rows], query.rows, 1);
    flann::Matrix<float> dists(new float[query.rows], query.rows, 1);
	
	// Do a knn search, using 128 checks
	index.knnSearch(query, indices, dists, 1, flann::SearchParams(128));
	int tar = indices[0][0];
//	cout << "In NN, target = (" << pure_data[tar * 2] << ", " << pure_data[tar * 2 + 1] << "), index = " << tar << ", dist = " << dists[0][0] << endl;

	return dists[0][0];

}