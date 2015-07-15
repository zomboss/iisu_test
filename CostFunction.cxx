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
	m_term_value = 0.0;
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
	m_term_value = 0.0;
}

void MyCostFunction::setDTerm()
{
	SK::Array<Sphere> sgroup = model.getFullHand();
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
	d_term_value = d_term_value * (double)sgroup.size() / (double)cloud.size();
}

void MyCostFunction::setDTerm_f()
{
	SK::Array<Sphere> sgroup = model.getFullHand();
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
	
	SK::Array<Sphere> sgroup = model.getFullHand();
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
	SK::Array<Sphere> sgroup = model.getFullHand();
	for(size_t c = 0; c < sgroup.size(); c++)
	{
		Eigen::Vector3f point = MyTools::SKtoEigenVector(sgroup[c].getCenter());
		int image_x, image_y;
		float range;
		planar.getImagePoint(point, image_x, image_y, range);		// range of center
		if(planar.isValid(image_x, image_y))
		{
			float ref = planar.getPoint(image_x, image_y).range;	// range of image
			if(ref > range)
				f_term_value += (double)((ref - range) * (ref - range));
		}
		else if(!planar.isValid(image_x, image_y))
		{
			int tar_x, tar_y;
			float tmpbe = getNearestNeighborNeighbor(index, image_x, image_y, tar_x, tar_y);
			float tmpref = planar.getPoint(tar_x, tar_y).range;
/*			if(c == 33)
			{
				cout << "In NN, project on: (" << image_x << ", " << image_y << "), nearest: (" << tar_x << ", " << tar_y << ")";
				cout << ", tmpref = " << tmpref << ", range = " << range << endl;
			}*/
			if(tmpref > range)
				f_term_value += (tmpbe * tmpbe * pix_meter + (tmpref - range) * (tmpref - range));
			else
				f_term_value += tmpbe * tmpbe * pix_meter;
		}
	}

	// lamda weight
	f_term_value *= 3.0;
}

void MyCostFunction::setLTerm()
{
	SK::Array<Sphere> sgroup = model.getFullHand();
	SK::Array<int> fingerlist = model.getAllFingersIndex();
	for(size_t i = 0; i < fingerlist.size(); i++)
	{
		double tmp_dis = 0;
		Sphere tmpsphere = sgroup[fingerlist[i]];
		SK::Array<Sphere> neoghorlist = model.getNeighbors(fingerlist[i]);
		for(size_t j = 0; j < neoghorlist.size(); j++)
		{
			Sphere neisphere = neoghorlist[j];
			double tmpvalue = tmpsphere.getRadius() + neisphere.getRadius() - tmpsphere.getCenter().distance(neisphere.getCenter());
			tmp_dis = (tmpvalue > 0) ? (tmp_dis + tmpvalue) : tmp_dis;
		}
		l_term_value += tmp_dis * tmp_dis;
	}

	// lamda weight
	l_term_value *= 5.0;
}

void MyCostFunction::setMTerm(HandPose pre_pose1, HandPose pre_pose2)
{
	HandModel prevmodel_1 = HandModel();
	pre_pose1.applyPose(prevmodel_1);
	HandModel prevmodel_2 = HandModel();
	pre_pose2.applyPose(prevmodel_2);
	Eigen::Matrix<float, 3, SPHERE_NUM> curr_center = model.getAllCenterMat(1.0f);
	curr_center = (curr_center.colwise() - MyTools::SKtoEigenVector(model.getGlobalpos()));
	Eigen::Matrix<float, 3, SPHERE_NUM> prev_center_1 = prevmodel_1.getAllCenterMat(1.0f);
	prev_center_1 = (prev_center_1.colwise() - MyTools::SKtoEigenVector(pre_pose1.getPosition()));
	Eigen::Matrix<float, 3, SPHERE_NUM> prev_center_2 = prevmodel_2.getAllCenterMat(1.0f);
	prev_center_2 = (prev_center_2.colwise() - MyTools::SKtoEigenVector(pre_pose2.getPosition()));
//	m_term_value = double((curr_center - prev_center_1 * 2 + prev_center_2).squaredNorm() * 0.5) + double((curr_center - prev_center_1).squaredNorm() * 0.5);
	m_term_value = double((curr_center - prev_center_1 * 2 + prev_center_2).squaredNorm() * 1.5);
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

void MyCostFunction::calculate(HandPose pre_pose1, HandPose pre_pose2)
{
	setDTerm();
	setMTerm(pre_pose1, pre_pose2);
	setLTerm();
}

void MyCostFunction::calculate(flann::Index<flann::L2<float> > &index, HandPose pre_pose1, HandPose pre_pose2)
{
	setDTerm();
	setFTerm(index);
	setMTerm(pre_pose1, pre_pose2);
	setLTerm();
}

float MyCostFunction::getNearestNeighborNeighbor(flann::Index<flann::L2<float> > &index, int query_x, int query_y, int &tar_x, int &tar_y)
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
	tar_x = (int)pure_data[tar * 2];
	tar_y = (int)pure_data[tar * 2 + 1];
//	cout << "In NN, target = (" <<tar_x << ", " << tar_y << "), index = " << tar << ", dist = " << dists[0][0] << endl;

	return dists[0][0];

}