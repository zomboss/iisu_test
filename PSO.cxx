#include <omp.h>
#include <ctime>
#include "PSO.hxx"
#include "AICP.hxx"
#include "ICPPCA.hxx"
#include "CostFunction.hxx"

const double c1 = 2.8;
const double c2 = 1.3;
const double c3 = 1.0;
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
	gk_point.assign(generation_num, std::numeric_limits<double>::infinity());
	prev_pose1 = HandPose();
	prev_pose2 = HandPose();
	hasprev = false;
	ismov = false;
}

void PSO::setPrevPose(HandPose prev1, HandPose prev2)
{
	prev_pose1 = prev1;
	prev_pose2 = prev2;
	hasprev = true;
}

void PSO::checkMoving(SK::Vector3 curr, SK::Vector3 prev)
{
	if(curr.distance(prev) > 2.0)
		ismov = true;
	else
		ismov = false;
}

void PSO::checkMoving(Eigen::Matrix<float, 3, 1> curr, Eigen::Matrix<float, 3, 1> prev)
{
	if((curr - prev).norm() > 2.0)
		ismov = true;
	else
		ismov = false;
}

void PSO::generateParticles(const HandPose &inipose)
{
	hispose.resize(particles_num);
	bestPose = inipose;
	ori_orientation = bestPose.getOrientation();
	SK::Array<float> curr_para = bestPose.getAllParameters();
	for(int i = 0; i < particles_num; i++)
	{
		HandPose tmppose = bestPose;
		SK::Array<float> new_para = MyTools::perturbParameter(curr_para);
		tmppose.setAllParameters(new_para);
		tmppose.setOrientation(ori_orientation);
		if(i == 0)	particles.pushBack(bestPose);
		else		particles.pushBack(tmppose);

		//initial particle's parameters
		SK::Array<float> v;
		v.assign(new_para.size(), 0.0);
		// random velocities
		velocity.pushBack(v);
//		velocity.pushBack(MyTools::perturbParameter(v));
		curr_points.pushBack(0.0);
		pk_point.pushBack(100000.0);
	}
}

void PSO::generateParticles(HandPose &inipose, HandPose &prepose)
{
	hispose.resize(particles_num);
	bestPose = prepose;
	SK::Array<float> curr_para = inipose.getAllParameters();
	SK::Array<float> pre_para = prepose.getAllParameters();
	for(int i = 0; i < particles_num; i++)
	{
		HandPose tmppose;
		SK::Array<float> new_para;
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
		if(i == 0)		particles.pushBack(inipose);
		else if(i == 1)	particles.pushBack(prepose);
		else			particles.pushBack(tmppose);

		//initial particle's parameters
		SK::Array<float> v;
		v.assign(new_para.size(), 0.0);
		// random velocities
		velocity.pushBack(v);
//		velocity.pushBack(MyTools::perturbParameter(v));
		curr_points.pushBack(0.0);
		pk_point.pushBack(100000.0);
	}
}

void PSO::generateLParticles(const HandPose &inipose, Eigen::Matrix<float, 3, 1> info_ori)
{
	particles.resize(particles_num);
	hispose.resize(particles_num);
	curr_points.resize(particles_num);
	pk_point.resize(particles_num);
	bestPose = inipose;
	ori_orientation = bestPose.getOrientation();
	SK::Array<float> curr_para = bestPose.getAllParameters();
	for(int i = 0; i < particles_num; i++)
	{
		bestPose.setPosition(MyTools::EigentoSKVector(info_ori));
		HandPose tmppose = bestPose;
		SK::Array<float> new_para = MyTools::perturbParameterL(curr_para, info_ori);
		tmppose.setAllParameters(new_para);
		tmppose.setOrientation(ori_orientation);
		if(i == 0)	particles[i] = bestPose;
		else		particles[i] = tmppose;

		//initial particle's parameters
		SK::Array<float> v;
		v.assign(new_para.size(), 0.0);
		// random velocities
//		velocity.pushBack(v);
		velocity.pushBack(MyTools::perturbParameterL(v, Eigen::Matrix<float, 3, 1>::Zero()));
		curr_points[i] = 0.0;
		pk_point[i] = std::numeric_limits<double>::infinity();
	}
}

void PSO::goGeneration(const PointCloud<PointXYZRGB> &cloud, const HandModel &model, bool iskmean = false)
{
	// useless
	double prepoint = 0.0;
	for(int g = 0; g < generation_num; g++)
	{
//		cout << "In generation " << g << ", show best point: " << gk_point[g] << endl;
		prepoint = gk_point[g];
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
			if(curr_points[p] < gk_point[g])
			{
				gk_point[g] = curr_points[p];		// point
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
		if(abs(prepoint - gk_point[g]) < 0.00001)	break;
	}
}

void PSO::goGeneration_mp(const PointCloud<PointXYZRGB> &cloud, const HandModel &model, bool iskmean = false, bool show = false)
{
	double prepoint = 0.0;
	for(int g = 0; g < generation_num; g++)
	{
		if(show)	cout << "In generation " << g << ", show best point: " << gk_point[g] << endl;
		// Stop if point change a little
		if(abs(prepoint - gk_point[g]) < 0.00001)	break;
		prepoint = gk_point[g];

		#pragma omp parallel for
		for(int p = 0; p < particles_num; p++)
		{
			if(m <= 0)	// compute point-sphere correspondences (CostFunction)
			{
				HandModel testmodel = model;
				particles[p].applyPose(testmodel);
				MyCostFunction costf = MyCostFunction(cloud, testmodel);
				costf.calculate();
				curr_points[p] = costf.getCost();
//				cout << "point in particle " << p << ", D = " << costf.getDTerm() << ", L = " << costf.getLTerm() << endl;
			}
			else	// Gradient descent on a random parameter, if m < 0, PSO only
			{
				AICP aicp = AICP(m, 5, particles[p]);
				aicp.run_randomPara(cloud);
				particles[p] = aicp.getBestPose();
				curr_points[p] = aicp.getBestCost();
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
			if(curr_points[p] < gk_point[g])
			{
				gk_point[g] = curr_points[p];		// point
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
		clock_t begin_t = clock();
		// Stop if point change a little
//		if(abs(prepoint - gk_point[g]) < 0.001)	break;
		prepoint = gk_point[g];

		int iCPU = omp_get_num_procs();
		omp_set_dynamic(0);     // Explicitly disable dynamic teams
		#pragma omp parallel for num_threads(iCPU)
		for(int p = 0; p < particles_num; p++)
		{
			if(m <= 0)	// compute point-sphere correspondences (CostFunction)
			{
				HandModel testmodel = model;
				particles[p].applyPose(testmodel);
				MyCostFunction costf = MyCostFunction(cloud, testmodel, planar, pix_meter, pure_vec);
				costf.calculate(index);
				curr_points[p] = costf.getCost();
//				cout << "show D-term = " << costf.getDTerm() << ", F-term = " << costf.getFTerm() << endl;
			}
			else	// Gradient descent on a random parameter, if m < 0, PSO only
			{
				AICP aicp = AICP(m, 5, particles[p]);
				aicp.run_randomPara(cloud);
				particles[p] = aicp.getBestPose();
				curr_points[p] = aicp.getBestCost();
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
			if(curr_points[p] < gk_point[g])
			{
				gk_point[g] = curr_points[p];		// point
				bestPose = particles[p];		// position
			}
		}

		// k-means clustering of all particles (Not yet!!!)

		// Particle swarm update within each cluster (PSOupdate)
		PSOupdate();
		clock_t end_t = clock();
		if(show)	cout << "In generation " << g << ", show best point: " << gk_point[g] << endl;
		if(show)	cout << "in generation " << g << ", main time = " << double(main_t - begin_t) << " ms, update time = " << double(end_t - main_t) << endl;

	}
}

SK::Array<HandPose> PSO::goGeneration_test(const PointCloud<PointXYZRGB> &cloud, const RangeImagePlanar &planar, const HandModel &model, bool iskmean = false, bool show = false)
{
	// build dataset from planar
	vector<float> pure_vec;
	Index<flann::L2<float>> index = buildDatasetIndex(planar, pure_vec);
	SK::Array<HandPose> bestpose_list;

	double prepoint = 0.0;
	for(int g = 0; g < generation_num; g++)
	{
		clock_t begin_t = clock();
		// Stop if point change a little
//		if(abs(prepoint - gk_point[g]) < 0.001)	break;
		prepoint = gk_point[g];

		int iCPU = omp_get_num_procs();
		omp_set_dynamic(0);     // Explicitly disable dynamic teams
		#pragma omp parallel for num_threads(iCPU)
		for(int p = 0; p < particles_num; p++)
		{
			if(m <= 0)	// compute point-sphere correspondences (CostFunction)
			{
				HandModel testmodel = model;
				particles[p].applyPose(testmodel);
				MyCostFunction costf = MyCostFunction(cloud, testmodel, planar, pix_meter, pure_vec);
				if(!hasprev)	costf.calculate(index);
				if(hasprev)		costf.calculate(index, prev_pose1, prev_pose2);
				curr_points[p] = costf.getCost();
//				cout << " in particle " << p << ", show D-term = " << costf.getDTerm() << ", F-term = " << costf.getFTerm() << ", M-term = " << costf.getMTerm() << endl;
			}
			else	// Gradient descent on a random parameter, if m < 0, PSO only
			{
				HandModel testmodel = model;
				particles[p].applyPose(testmodel);
				MyCostFunction costf = MyCostFunction(cloud, testmodel, planar, pix_meter, pure_vec);
				if(!hasprev)	costf.calculate(index);
				if(hasprev)		costf.calculate(index, prev_pose1, prev_pose2);
				curr_points[p] = costf.getCost();
//				cout << " in particle " << p << ", show D-term = " << costf.getDTerm() << ", F-term = " << costf.getFTerm() << ", M-term = " << costf.getMTerm() << endl;

				// Get gradient descent
				AICP aicp = hasprev ? AICP(m, 1, particles[p], prev_pose1, prev_pose2) : AICP(m, 1, particles[p]);
				if(ismov && g == 0)	aicp.run_globalPara(cloud, planar, pix_meter, pure_vec, index);
				else				aicp.run_randomPara(cloud, planar, pix_meter, pure_vec, index);
/*				aicp.run_localPara(cloud, planar, pix_meter, pure_vec, index);*/
				SK::Array<float> stepvel = MyTools::subArray(aicp.getBestPose().getAllParameters(), particles[p].getAllParameters());
				velocity[p] = MyTools::addArray(velocity[p], stepvel);
			}
		}

		clock_t main_t = clock();
		// Update pk and Gk
		int best_prat_num = -1;
		for(int p = 0; p < particles_num; p++)
		{
			if(curr_points[p] < pk_point[p])
			{
				pk_point[p] = curr_points[p];	// point
				hispose[p] = particles[p];		// position
			}
			if(curr_points[p] < gk_point[g])
			{
				gk_point[g] = curr_points[p];		// point
				bestPose = particles[p];		// position
				best_prat_num = p;
			}
		}

		// k-means clustering of all particles (Not yet!!!)

		// Particle swarm update within each cluster (PSOupdate)
		PSOupdate();
		clock_t end_t = clock();
		
		if(show)	cout << "In generation " << g << ", show best point: " << gk_point[g] << endl;
//		if(show)	cout << "in generation " << g << ", main time = " << double(main_t - begin_t) << " ms, update time = " << double(end_t - main_t) << endl;
		if(show)	cout << "best particle in generation " << g << " :" << best_prat_num << endl;

		// add to list
		bestpose_list.pushBack(bestPose);
	}

	return bestpose_list;
}

SK::Array<HandPose> PSO::goGeneration_triv(const PointCloud<PointXYZRGB> &cloud, const RangeImagePlanar &planar, const HandModel &model, bool iskmean = false, bool show = false)
{
	// build dataset from planar
	vector<float> pure_vec;
	Index<flann::L2<float>> index = buildDatasetIndex(planar, pure_vec);
	SK::Array<HandPose> bestpose_list;

	double prepoint = 0.0;
	for(int g = 0; g < generation_num; g++)
	{
		clock_t begin_t = clock();
		// Stop if point change a little
//		if(abs(prepoint - gk_point[g]) < 0.001)	break;
		prepoint = gk_point[g];
		SK::Array<SK::Array<float>> stepvellist;
		stepvellist.resize(particles_num);

		int iCPU = omp_get_num_procs();
		omp_set_dynamic(0);     // Explicitly disable dynamic teams
		#pragma omp parallel for num_threads(iCPU)
		for(int p = 0; p < particles_num; p++)
		{
			if(m <= 0)	// compute point-sphere correspondences (CostFunction)
			{
				HandModel testmodel = model;
				particles[p].applyPose(testmodel);
				MyCostFunction costf = MyCostFunction(cloud, testmodel, planar, pix_meter, pure_vec);
				if(!hasprev)	costf.calculate(index);
				if(hasprev)		costf.calculate(index, prev_pose1, prev_pose2);
				curr_points[p] = costf.getCost();
//				cout << " in particle " << p << ", show D-term = " << costf.getDTerm() << ", F-term = " << costf.getFTerm() << ", M-term = " << costf.getMTerm() << endl;
			}
			else	// Gradient descent on a random parameter, if m < 0, PSO only
			{
				HandModel testmodel = model;
				particles[p].applyPose(testmodel);
				MyCostFunction costf = MyCostFunction(cloud, testmodel, planar, pix_meter, pure_vec);
				if(!hasprev)	costf.calculate(index);
				if(hasprev)		costf.calculate(index, prev_pose1, prev_pose2);
				curr_points[p] = costf.getCost();
//				cout << " in particle " << p << ", show D-term = " << costf.getDTerm() << ", F-term = " << costf.getFTerm() << ", M-term = " << costf.getMTerm() << endl;

				// Get gradient descent
				AICP aicp = hasprev ? AICP(m, 1, particles[p], prev_pose1, prev_pose2) : AICP(m, 1, particles[p]);
/*				if(ismov && g == 0)	aicp.run_globalPara(cloud, planar, pix_meter, pure_vec, index);
				else				aicp.run_randomPara(cloud, planar, pix_meter, pure_vec, index);*/
				aicp.run_localPara(cloud, planar, pix_meter, pure_vec, index);
				stepvellist[p] = aicp.getBestPose().getAllParameters();
			}
		}

		clock_t main_t = clock();
		// Update pk and Gk
		int best_prat_num = -1;
		for(int p = 0; p < particles_num; p++)
		{
			if(curr_points[p] < pk_point[p])
			{
				pk_point[p] = curr_points[p];	// point
				hispose[p] = particles[p];		// position
			}
			if(curr_points[p] < gk_point[g])
			{
				gk_point[g] = curr_points[p];		// point
				bestPose = particles[p];		// position
				best_prat_num = p;
			}
		}

		// k-means clustering of all particles (Not yet!!!)

		// Particle swarm update within each cluster (PSOupdate)
		PSOupdate_tri(stepvellist);
		clock_t end_t = clock();
		
		if(show)	cout << "In generation " << g << ", show best point: " << gk_point[g] << endl;
//		if(show)	cout << "in generation " << g << ", main time = " << double(main_t - begin_t) << " ms, update time = " << double(end_t - main_t) << endl;
		if(show)	cout << "best particle in generation " << g << " :" << best_prat_num << endl;

		// add to list
		bestpose_list.pushBack(bestPose);
	}

	return bestpose_list;
}

SK::Array<HandPose> PSO::goGeneration_loop(const PointCloud<PointXYZRGB> &cloud, const RangeImagePlanar &planar, const HandModel &model, bool iskmean = false, bool show = false)
{
	// build dataset from planar
	vector<float> pure_vec;
	Index<flann::L2<float>> index = buildDatasetIndex(planar, pure_vec);
	SK::Array<HandPose> bestpose_list;

	double prepoint = 0.0;
	for(int g = 0; g < generation_num; g++)
	{
		clock_t begin_t = clock();
		// Stop if point change a little
//		if(abs(prepoint - gk_point[g]) < 0.001)	break;
		prepoint = gk_point[g];
		SK::Array<SK::Array<float>> stepvellist;
		stepvellist.resize(particles_num);

		int iCPU = omp_get_num_procs();
		omp_set_dynamic(0);     // Explicitly disable dynamic teams
		#pragma omp parallel for num_threads(iCPU)
		for(int p = 0; p < particles_num; p++)
		{
			if(m <= 0)	// compute point-sphere correspondences (CostFunction)
			{
				HandModel testmodel = model;
				particles[p].applyPose(testmodel);
				MyCostFunction costf = MyCostFunction(cloud, testmodel, planar, pix_meter, pure_vec);
				if(!hasprev)	costf.calculate(index);
				if(hasprev)		costf.calculate(index, prev_pose1, prev_pose2);
				curr_points[p] = costf.getCost();
//				cout << " in particle " << p << ", show D-term = " << costf.getDTerm() << ", F-term = " << costf.getFTerm() << ", M-term = " << costf.getMTerm() << endl;
			}
			else	// Gradient descent on a random parameter, if m < 0, PSO only
			{
				HandModel testmodel = model;
				particles[p].applyPose(testmodel);
				MyCostFunction costf = MyCostFunction(cloud, testmodel, planar, pix_meter, pure_vec);
				if(!hasprev)	costf.calculate(index);
				if(hasprev)		costf.calculate(index, prev_pose1, prev_pose2);
				curr_points[p] = costf.getCost();
//				cout << " in particle " << p << ", show D-term = " << costf.getDTerm() << ", F-term = " << costf.getFTerm() << ", M-term = " << costf.getMTerm() << endl;

				// Get gradient descent
				AICP aicp = hasprev ? AICP(m, 1, particles[p], prev_pose1, prev_pose2) : AICP(m, 1, particles[p]);
				aicp.run_strategy(g, cloud, planar, pix_meter, pure_vec, index);
				stepvellist[p] = aicp.getBestPose().getAllParameters();
			}
		}

		clock_t main_t = clock();
		// Update pk and Gk
		int best_prat_num = -1;
		for(int p = 0; p < particles_num; p++)
		{
			if(curr_points[p] < pk_point[p])
			{
				pk_point[p] = curr_points[p];	// point
				hispose[p] = particles[p];		// position
			}
			if(curr_points[p] < gk_point[g])
			{
				gk_point[g] = curr_points[p];		// point
				bestPose = particles[p];		// position
				best_prat_num = p;
			}
		}

		// k-means clustering of all particles (Not yet!!!)

		// Particle swarm update within each cluster (PSOupdate)
		PSOupdate_tri(stepvellist);
		clock_t end_t = clock();
		
		if(show)	cout << "In generation " << g << ", show best point: " << gk_point[g] << endl;
//		if(show)	cout << "in generation " << g << ", main time = " << double(main_t - begin_t) << " ms, update time = " << double(end_t - main_t) << endl;
		if(show)	cout << "best particle in generation " << g << " :" << best_prat_num << endl;

		// add to list
		bestpose_list.pushBack(bestPose);
	}

	return bestpose_list;
}

SK::Array<HandPose> PSO::goGeneration_func(const PointCloud<PointXYZRGB> &cloud, const RangeImagePlanar &planar, const HandModel &model, Eigen::Matrix<float, 3, 1> center, bool iskmean = false, bool show = false)
{
	// build dataset from planar
	vector<float> pure_vec;
	Index<flann::L2<float>> index = buildDatasetIndex(planar, pure_vec);
	SK::Array<HandPose> bestpose_list;

	double prepoint = 0.0;
	for(int g = 0; g < generation_num; g++)
	{
		clock_t begin_t = clock();
		// Stop if point change a little
//		if(abs(prepoint - gk_point[g]) < 0.001)	break;
		prepoint = gk_point[g];
		SK::Array<SK::Array<float>> stepvellist;
		stepvellist.resize(particles_num);

		// get modified particles
		if((g % 6) == 0 && g != 0)
			generateLParticles(bestPose, center);
		int _g = (g == 0) ? -2 : ((g % 6) - 1);
		SK::Array<HandPose> modparts = generateTParticles(_g);

		int iCPU = omp_get_num_procs();
		omp_set_dynamic(0);     // Explicitly disable dynamic teams
		#pragma omp parallel for num_threads(iCPU)
		for(int p = 0; p < particles_num; p++)
		{
			if(m <= 0)	// compute point-sphere correspondences (CostFunction)
			{
				HandModel testmodel = model;
				modparts[p].applyPose(testmodel);
				MyCostFunction costf = MyCostFunction(cloud, testmodel, planar, pix_meter, pure_vec);
				if(!hasprev)	costf.calculate(index);
				if(hasprev)		costf.calculate(index, prev_pose1, prev_pose2);
				curr_points[p] = costf.getCost();
//				cout << " in particle " << p << ", show D-term = " << costf.getDTerm() << ", F-term = " << costf.getFTerm() << ", M-term = " << costf.getMTerm() << endl;
			}
			else	// Gradient descent on a random parameter, if m < 0, PSO only
			{
				HandModel testmodel = model;
				modparts[p].applyPose(testmodel);
				MyCostFunction costf = MyCostFunction(cloud, testmodel, planar, pix_meter, pure_vec);
				if(!hasprev)	costf.calculate(index);
				if(hasprev)		costf.calculate(index, prev_pose1, prev_pose2);
				curr_points[p] = costf.getCost();
//				cout << " in particle " << p << ", show D-term = " << costf.getDTerm() << ", F-term = " << costf.getFTerm() << ", M-term = " << costf.getMTerm() << endl;

				// Get gradient descent
				AICP aicp = hasprev ? AICP(m, 1, modparts[p], prev_pose1, prev_pose2) : AICP(m, 1, modparts[p]);
				aicp.run_strategy(g, cloud, planar, pix_meter, pure_vec, index);
				stepvellist[p] = aicp.getBestPose().getAllParameters();
			}
		}

		clock_t main_t = clock();
		// Update pk and Gk
		int best_prat_num = -1;
		for(int p = 0; p < particles_num; p++)
		{
			if(curr_points[p] < gk_point[g])
			{
				gk_point[g] = curr_points[p];		// point
				bestPose = modparts[p];		// position
				best_prat_num = p;
			}
		}

		// k-means clustering of all particles (Not yet!!!)

		// Particle swarm update within each cluster (PSOupdate)
		PSOupdate_noH(modparts, stepvellist);
		clock_t end_t = clock();
		
		if(show)	cout << "In generation " << g << ", show best point: " << gk_point[g] << endl;
		if(show)	cout << "best particle in generation " << g << " :" << best_prat_num << endl;

		// add to list
		bestpose_list.pushBack(bestPose);
	}

	return bestpose_list;
}

void PSO::goGeneration_data(const PointCloud<PointXYZRGB> &cloud, const HandModel &model, DataDriven &data_driven, bool iskmean = false, bool show = false)
{
	double prepoint = 0.0;
	for(int g = 0; g < generation_num; g++)
	{
		if(show)	cout << "In generation " << g << ", show best point: " << gk_point[g] << endl;
		// Stop if point change a little
		if(abs(prepoint - gk_point[g]) < 0.00001)	break;
		prepoint = gk_point[g];

		#pragma omp parallel for
		for(int p = 0; p < particles_num; p++)
		{
			if(m <= 0)	// compute point-sphere correspondences (CostFunction)
			{
				HandModel testmodel = model;
				particles[p].applyPose(testmodel);
				MyCostFunction costf = MyCostFunction(cloud, testmodel);
				costf.calculate();
				curr_points[p] = costf.getCost();
//				cout << "point in particle " << p << ", D = " << costf.getDTerm() << ", L = " << costf.getLTerm() << endl;
			}
			else	// Gradient descent on a random parameter, if m < 0, PSO only
			{
				ICPPCA icppca = ICPPCA(4, 5, data_driven.getPCSpaceSize(), particles[p], data_driven.getTransMatrix(), 
									   data_driven.getMeanVector(), data_driven.getMaxVector(), data_driven.getMinVector());
				icppca.run(cloud);
				particles[p] = icppca.getBestPose();
				curr_points[p] = icppca.getBestCost();
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
			if(curr_points[p] < gk_point[g])
			{
				gk_point[g] = curr_points[p];		// point
				bestPose = particles[p];		// position
			}
		}

		// k-means clustering of all particles (Not yet!!!)

		// Particle swarm update within each cluster (PSOupdate)
		PSOupdate();

	}
}

void PSO::goGeneration_datafull(const PointCloud<PointXYZRGB> &cloud, const RangeImagePlanar &planar, const HandModel &model, DataDriven &data_driven, bool iskmean = false, bool show = false)
{
	// build dataset from planar
	vector<float> pure_vec;
	Index<flann::L2<float>> index = buildDatasetIndex(planar, pure_vec);
	
	double prepoint = 0.0;
	for(int g = 0; g < generation_num; g++)
	{
		if(show)	cout << "In generation " << g << ", show best point: " << gk_point[g] << endl;
		
		clock_t begin_t = clock();
		// Stop if point change a little
//		if(abs(prepoint - gk_point[g]) < 0.001)	break;
		prepoint = gk_point[g];

		int iCPU = omp_get_num_procs();
		omp_set_dynamic(0);     // Explicitly disable dynamic teams
		#pragma omp parallel for num_threads(iCPU)
		for(int p = 0; p < particles_num; p++)
		{
			if(m <= 0)	// compute point-sphere correspondences (CostFunction)
			{
				HandModel testmodel = model;
				particles[p].applyPose(testmodel);
				MyCostFunction costf = MyCostFunction(cloud, testmodel, planar, pix_meter, pure_vec);
				costf.calculate(index);
				curr_points[p] = costf.getCost();
//				cout << "show D-term = " << costf.getDTerm() << ", F-term = " << costf.getFTerm() << endl;
			}
			else	// Gradient descent on a random parameter, if m < 0, PSO only
			{
				ICPPCA icppca = ICPPCA(4, 5, data_driven.getPCSpaceSize(), particles[p], data_driven.getTransMatrix(), 
									   data_driven.getMeanVector(), data_driven.getMaxVector(), data_driven.getMinVector());
				icppca.run(cloud);
				particles[p] = icppca.getBestPose();
				curr_points[p] = icppca.getBestCost();
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
			if(curr_points[p] < gk_point[g])
			{
				gk_point[g] = curr_points[p];		// point
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
	pix_meter /= (float)counter;	// weird result
	pix_meter = 0.5;

	int pure_size = (int)data.size() / 2;
	float *pure_data = data.data();

	// Initial flann
	flann::Matrix<float> dataset(pure_data, pure_size, 2);
	
	// Construct an randomized kd-tree index using 4 kd-trees
	flann::Index<flann::L2<float> > index(dataset, flann::KDTreeIndexParams(4));
    index.buildIndex();

	return index;
}

SK::Array<HandPose> PSO::generateTParticles(int fin)
{
	SK::Array<HandPose> tmpparticles;
	tmpparticles.assign(particles_num, bestPose);
	if(fin == -2)
		return particles;
	if(fin == -1)
	{
		for(int p = 0; p < particles_num; p++)
			tmpparticles[p].setRotation(particles[p].getRotation());
		return tmpparticles;
	}
	for(int p = 0; p < particles_num; p++)
	{
		SK::Array<SK::Vector3> arr = particles[p].getFingerPose(fin);
		tmpparticles[p].setFingerPose(fin, arr[0], arr[1], arr[2]);
	}
	return tmpparticles;
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
		SK::Array<float> tmpv1 = MyTools::subArray(hispose[p].getAllParameters(), particles[p].getAllParameters());
		tmpv1 = MyTools::scaArray(tmpv1, (c1 * r1));
		SK::Array<float> tmpv2 = MyTools::subArray(bestPose.getAllParameters(), particles[p].getAllParameters());
		tmpv2 = MyTools::scaArray(tmpv2, (c2 * r2));
		tmpv1 = MyTools::addArray(tmpv1, tmpv2);
		tmpv1 = MyTools::addArray(tmpv1, velocity[p]);
		velocity[p] = MyTools::scaArray(tmpv1, w);
		// x(k+1) = xk + v(k+1)
		SK::Array<float> tmpx = MyTools::addArray(particles[p].getAllParameters(), velocity[p]);
		particles[p].setAllParameters(tmpx);
	}
}

void PSO::PSOupdate_tri(SK::Array<SK::Array<float>> &stepvellist)
{
	for(int p = 0; p < particles_num; p++)
	{
		float r1 = ((float) rand() / (RAND_MAX));
		float r2 = ((float) rand() / (RAND_MAX));
		float r3 = ((float) rand() / (RAND_MAX));
		float psi = c1 + c2 + c3;
		float w = 2.0 / abs(2.0 - psi - sqrt(psi * psi - 4.0 * psi));
		// v(k+1) = w(vk + c1 * r1 * (Pk - xk) + c2 * r2 * (Gk - xk) + c3 * r3 * (Delk - xk))
		SK::Array<float> tmpv1 = MyTools::subArray(hispose[p].getAllParameters(), particles[p].getAllParameters());
		tmpv1 = MyTools::scaArray(tmpv1, (c1 * r1));
		SK::Array<float> tmpv2 = MyTools::subArray(bestPose.getAllParameters(), particles[p].getAllParameters());
		tmpv2 = MyTools::scaArray(tmpv2, (c2 * r2));
		SK::Array<float> tmpv3 = MyTools::subArray(stepvellist[p], particles[p].getAllParameters());
		tmpv3 = MyTools::scaArray(tmpv3, (c3 * r3));
		tmpv1 = MyTools::addArray(tmpv1, tmpv2);
		tmpv1 = MyTools::addArray(tmpv1, tmpv3);
		tmpv1 = MyTools::addArray(tmpv1, velocity[p]);
		velocity[p] = MyTools::scaArray(tmpv1, w);
		// x(k+1) = xk + v(k+1)
		SK::Array<float> tmpx = MyTools::addArray(particles[p].getAllParameters(), velocity[p]);
		particles[p].setAllParameters(tmpx);
	}
}

void PSO::PSOupdate_noH(SK::Array<HandPose> &poselist, SK::Array<SK::Array<float>> &stepvellist)
{
	for(int p = 0; p < particles_num; p++)
	{
		float r2 = ((float) rand() / (RAND_MAX));
		float r3 = ((float) rand() / (RAND_MAX));
		float psi = c2 + c1;
		float w = 2.0 / abs(2.0 - psi - sqrt(psi * psi - 4.0 * psi));

		// v = w(c2 * r2 * (Gk - xk) + c3 * r3 * (Delk - xk))
		SK::Array<float> tmpv2 = MyTools::subArray(bestPose.getAllParameters(), poselist[p].getAllParameters());
		tmpv2 = MyTools::scaArray(tmpv2, (c2 * r2));
		SK::Array<float> tmpv3 = MyTools::subArray(stepvellist[p], poselist[p].getAllParameters());
		tmpv3 = MyTools::scaArray(tmpv3, (c1 * r3));
		
		tmpv2 = MyTools::addArray(tmpv2, tmpv3);
		//tmpv2 = MyTools::addArray(tmpv2, velocity[p]);
		velocity[p] = MyTools::scaArray(tmpv2, w);
		
		// x(k+1) = xk + v
		SK::Array<float> tmpx = MyTools::addArray(poselist[p].getAllParameters(), velocity[p]);
		poselist[p].setAllParameters(tmpx);
	}
}
