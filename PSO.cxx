#include <omp.h>
#include <ctime>
#include "PSO.hxx"
#include "AICP.hxx"
#include "CostFunction.hxx"

const double c1 = 2.8;
const double c2 = 1.3;
const int HEIGHT = 240;
const int WIDTH = 320;

PSO::PSO()
{
	PSO(30, 20, 10, 1);
}

PSO::PSO(int _g, int _p, int _m, int _k)
{
	generation_num = _g;
	particles_num = _p;
	m = _m;
	k = _k;
	pix_meter = 0.0;
}

void PSO::generateParticles(const HandPose &inipose)
{
	hispose.resize(particles_num);
	bestPose = inipose;
	ori_orientation = bestPose.getOrientation();
	Array<float> curr_para = bestPose.getAllParameters();
	for(int i = 0; i < particles_num; i++)
	{
		HandPose tmppose = bestPose;
		Array<float> new_para = MyTools::perturbParameter(curr_para);
		tmppose.setAllParameters(new_para);
		tmppose.setOrientation(ori_orientation);
		particles.pushBack(tmppose);

		//initial particle's parameters
		Array<float> v;
		v.assign(new_para.size(), 0.0);
		velocity.pushBack(v);
		curr_points.pushBack(0.0);
		pk_point.pushBack(100000.0);
	}
	gk_point = 1000000.0;
}

void PSO::generateParticles(HandPose &inipose, HandPose &prepose)
{
	hispose.resize(particles_num);
	bestPose = prepose;
	Array<float> curr_para = inipose.getAllParameters();
	Array<float> pre_para = prepose.getAllParameters();
	for(int i = 0; i < particles_num; i++)
	{
		HandPose tmppose;
		Array<float> new_para;
		// Assign ratio: 3:1
		if(i % 4 == 0)
		{
			new_para = MyTools::perturbParameter(curr_para);
			tmppose = inipose;
		}
		else
		{
			new_para = MyTools::perturbParameter(pre_para);
			tmppose = prepose;
		}
		tmppose.setAllParameters(new_para);
		particles.pushBack(tmppose);

		//initial particle's parameters
		Array<float> v;
		v.assign(new_para.size(), 0.0);
		velocity.pushBack(v);
		curr_points.pushBack(0.0);
		pk_point.pushBack(100000.0);
	}
	gk_point = 100000.0;
}

void PSO::goGeneration(const PointCloud<PointXYZRGB> &cloud, const HandModel &model, bool iskmean = false)
{
	double prepoint = 0.0;
	for(int g = 0; g < generation_num; g++)
	{
//		cout << "In generation " << g << ", show best point: " << gk_point << endl;
		prepoint = gk_point;
		for(int p = 0; p < particles_num; p++)
		{
			// compute point-sphere correspondences (CostFunction)
			HandModel testmodel = model;
			particles[p].applyPose(testmodel);
			MyCostFunction costf = MyCostFunction(cloud, testmodel);
			costf.calculate();
			curr_points[p] = costf.getCost();
			// Update pk and gk
			if(curr_points[p] < pk_point[p])
			{
				pk_point[p] = curr_points[p];	// point
				hispose[p] = particles[p];		// position
			}
			if(curr_points[p] < gk_point)
			{
				gk_point = curr_points[p];		// point
				bestPose = particles[p];		// position
			}

			// Gradient descent on a random parameter, if m < 0, PSO only
			if(m > 0)
			{
				AICP aicp = AICP(m, 2, particles[p]);
				aicp.run_randomPara(cloud);
				particles[p] = aicp.getBestPose();
//				cout << "ICP Iteration in " << p << " done..." << endl;
			}

		}
		// k-means clustering of all particles (Not yet!!!)

		// Particle swarm update within each cluster (PSOupdate)
		PSOupdate();

		// Stop if point change a little
		if(abs(prepoint - gk_point) < 0.00001)	break;
	}
}

void PSO::goGeneration_mp(const PointCloud<PointXYZRGB> &cloud, const HandModel &model, bool iskmean = false, bool show = false)
{
	double prepoint = 0.0;
	for(int g = 0; g < generation_num; g++)
	{
		if(show)	cout << "In generation " << g << ", show best point: " << gk_point << endl;
		// Stop if point change a little
		if(abs(prepoint - gk_point) < 0.00001)	break;
		prepoint = gk_point;

		#pragma omp parallel for
		for(int p = 0; p < particles_num; p++)
		{
			// compute point-sphere correspondences (CostFunction)
			HandModel testmodel = model;
			particles[p].applyPose(testmodel);
			MyCostFunction costf = MyCostFunction(cloud, testmodel);
			costf.calculate();
			curr_points[p] = costf.getCost();
//			cout << "point in particle " << p << ", D = " << costf.getDTerm() << ", L = " << costf.getLTerm() << endl;

			// Gradient descent on a random parameter, if m < 0, PSO only
			if(m > 0)
			{
				AICP aicp = AICP(m, 10, particles[p]);
				aicp.run_randomPara(cloud);
				particles[p] = aicp.getBestPose();
//				cout << "ICP Iteration in " << p << " done..." << endl;
			}

		}
		// Update pk and Gk
		for(int p = 0; p < particles_num; p++)
		{
			if(curr_points[p] < pk_point[p])
			{
				pk_point[p] = curr_points[p];	// point
				hispose[p] = particles[p];		// position
			}
			if(curr_points[p] < gk_point)
			{
				gk_point = curr_points[p];		// point
				bestPose = particles[p];		// position
			}
		}

		// k-means clustering of all particles (Not yet!!!)

		// Particle swarm update within each cluster (PSOupdate)
		PSOupdate();

	}
}

void PSO::goGeneration_full(const PointCloud<PointXYZRGB> &cloud, const RangeImagePlanar &planar, const HandModel &model, bool iskmean = false, bool show = false)
{
	// build dataset from planar
	vector<float> pure_vec;
	Index<flann::L2<float>> index = buildDatasetIndex(planar, pure_vec);
	
	double prepoint = 0.0;
	for(int g = 0; g < generation_num; g++)
	{
		if(show)	cout << "In generation " << g << ", show best point: " << gk_point << endl;
		
		clock_t begin_t = clock();
		// Stop if point change a little
		if(abs(prepoint - gk_point) < 0.001)	break;
		prepoint = gk_point;

		int iCPU = omp_get_num_procs();
		omp_set_dynamic(0);     // Explicitly disable dynamic teams
		#pragma omp parallel for num_threads(iCPU)
		for(int p = 0; p < particles_num; p++)
		{
			// compute point-sphere correspondences (CostFunction)
			HandModel testmodel = model;
			particles[p].applyPose(testmodel);
			MyCostFunction costf = MyCostFunction(cloud, testmodel, planar, pix_meter, pure_vec);
			costf.calculate(index);
			curr_points[p] = costf.getCost();
//			cout << "show D-term = " << costf.getDTerm() << ", F-term = " << costf.getFTerm() << endl;

			// Gradient descent on a random parameter, if m < 0, PSO only
			if(m > 0)
			{
				AICP aicp = AICP(m, 10, particles[p]);
				aicp.run_randomPara(cloud);
				particles[p] = aicp.getBestPose();
//				cout << "ICP Iteration in " << p << " done..." << endl;
			}
			

		}

		clock_t main_t = clock();
		// Update pk and Gk
		for(int p = 0; p < particles_num; p++)
		{
			if(curr_points[p] < pk_point[p])
			{
				pk_point[p] = curr_points[p];	// point
				hispose[p] = particles[p];		// position
			}
			if(curr_points[p] < gk_point)
			{
				gk_point = curr_points[p];		// point
				bestPose = particles[p];		// position
			}
		}

		// k-means clustering of all particles (Not yet!!!)

		// Particle swarm update within each cluster (PSOupdate)
		PSOupdate();
		clock_t end_t = clock();
		if(show)	cout << "in generation " << g << ", main time = " << double(main_t - begin_t) << " ms, update time = " << double(end_t - main_t) << endl;

	}
}

Index<flann::L2<float>> PSO::buildDatasetIndex(const RangeImagePlanar &planar, vector<float> &data)
{
	// Load valid index (x,y)
	bool prev = false;
	int counter = 0;
	for(int j = 0; j < HEIGHT; j++)
	{
		for(int i = 0; i < WIDTH; i++)
		{
			if(planar.isValid(i, j))
			{
				// get the distance between one pixel
				if(prev)
				{
					PointWithRange prev_point = planar.getPoint(i - 1, j);
					PointWithRange curr_point = planar.getPoint(i, j);
					pix_meter += squaredEuclideanDistance(prev_point, curr_point);
					counter++;
				}
				data.push_back((float)i);
				data.push_back((float)j);
				prev = true;
			}
			else prev = false;
		}
		prev = false;
	}
	pix_meter /= (float)counter;

	int pure_size = (int)data.size() / 2;
	float *pure_data = data.data();

	// Initial flann
	flann::Matrix<float> dataset(pure_data, pure_size, 2);
	
	// Construct an randomized kd-tree index using 4 kd-trees
	flann::Index<flann::L2<float> > index(dataset, flann::KDTreeIndexParams(4));
    index.buildIndex();

	return index;
}

void PSO::PSOupdate()
{
	for(int p = 0; p < particles_num; p++)
	{
		float r1 = ((float) rand() / (RAND_MAX));
		float r2 = ((float) rand() / (RAND_MAX));
		float psi = c1 + c2;
		float w = 2.0 / abs(2.0 - psi - sqrt(psi * psi - 4.0 * psi));
		// v(k+1) = w(vk + c1 * r1 * (Pk - xk) + c2 * r2 * (Gk - xk))
		Array<float> tmpv1 = MyTools::subArray(hispose[p].getAllParameters(), particles[p].getAllParameters());
		tmpv1 = MyTools::scaArray(tmpv1, (c1 * r1));
		Array<float> tmpv2 = MyTools::subArray(bestPose.getAllParameters(), particles[p].getAllParameters());
		tmpv2 = MyTools::scaArray(tmpv2, (c2 * r2));
		tmpv1 = MyTools::addArray(tmpv1, tmpv2);
		tmpv1 = MyTools::addArray(tmpv1, velocity[p]);
		velocity[p] = MyTools::scaArray(tmpv1, w);
		// x(k+1) = xk + v(k+1)
		Array<float> tmpx = MyTools::addArray(particles[p].getAllParameters(), velocity[p]);
		particles[p].setAllParameters(tmpx);
	}
}
