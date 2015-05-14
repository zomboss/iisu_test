#include <stdlib.h>
#include <time.h> 
#include "AICP.hxx"
#include "ceres/ceres.h"
#include "Eigen"

using namespace ceres;

AICP::AICP()
{
	AICP(10, 20, HandPose());
};

AICP::AICP(int t, int it, const HandPose &pose)
{
	times = t;
	iter = it;
	bestpose = pose;
}

int AICP::getRandomJoint(int &finger, int &joint, double &theta, double &upper, double &lower)
{
	int randomjoint = rand() % 26;
	getSpecJoint(randomjoint, finger, joint, theta, upper, lower);
	return randomjoint;
}

int AICP::getRandomPart(int &finger, double **theta, double **upper, double **lower)
{
	int randompart = rand() % 7;
	double *tmptheta, *tmpupper, *tmplower;
	switch(randompart)
	{
	case 0:		// thumb
	case 1:		// index
	case 2:		// middle
	case 3:		// ring
	case 4:		// little
		finger = randompart;
		tmptheta = new double[4];
		tmpupper = new double[4];
		tmplower = new double[4];
		for(int i = 0; i < 4; i++)
		{
			int index = randompart + i;
			tmptheta[i] = bestpose.getAllParameters()[index];
			tmpupper[i] = HandPose::getBound(index, true);
			tmplower[i] = HandPose::getBound(index, false);
		}
		break;
	case 5:		// rotation
		finger = -1;
		tmptheta = new double[3];
		for(int i = 0; i < 3; i++)
		{
			int index = 23 + i;
			tmptheta[i] = bestpose.getAllParameters()[index];
		}
		break;
	case 6:		// position
		finger = -2;
		tmptheta = new double[3];
		for(int i = 0; i < 3; i++)
		{
			int index = 20 + i;
			tmptheta[i] = bestpose.getAllParameters()[index];
		}
		break;

	}
	*theta = tmptheta;
	*upper = tmpupper;
	*lower = tmplower;

	return randompart;
}

void AICP::getSpecJoint(int index, int &finger, int &joint, double &theta, double &upper, double &lower)
{
	upper = HandPose::getBound(index, true);
	lower = HandPose::getBound(index, false);
	if(index >= 20)
	{
		finger = -1;
		joint = index - 20;
	}
	else
	{
		finger = index / 4;
		joint = index % 4;
	}
	theta = bestpose.getAllParameters()[index];

}

void AICP::updatePose(int index, double theta)
{
	Array<float> tmpparas = bestpose.getAllParameters();
	tmpparas[index] = theta;
	bestpose.setAllParameters(tmpparas);
}

void AICP::run_randomPara(const PointCloud<PointXYZRGB> &cloud)
{
	srand(time(NULL));
	for(int t = 0; t < times; t++)
	{
		PointCloud<PointXYZRGB> tmpcloud = cloud;
		int finger = -2, joint = -2;
		double theta = 0.0, upper = 0.0, lower = 0.0;
		int seed = getRandomJoint(finger, joint, theta, upper, lower);
		cout << "In time " << t << ", seed = " << seed << endl;
/*		if(finger == -1)
			cout << "Get finger = " << finger << ", joint = " << joint << ", theta = " << theta << endl;
		else
			cout << "Get finger = " << finger << ", joint = " << joint << ", theta = " << theta << ", upper = " << upper << ", lower = " << lower << endl;*/
		Problem problem;
		ceres::CostFunction* cf_curr;
		if(finger == -1)
		{
			cf_curr = new NumericDiffCostFunction<CF_Global, CENTRAL, 1, 1> (new CF_Global(tmpcloud, joint, bestpose));
			problem.AddResidualBlock(cf_curr, NULL, &theta);
		}
		else
		{
			cf_curr = new NumericDiffCostFunction<CF_Finger, CENTRAL, 1, 1> (new CF_Finger(tmpcloud, finger, joint, bestpose));
			problem.AddResidualBlock(cf_curr, NULL, &theta);
			problem.SetParameterUpperBound(&theta, 0, upper);
			problem.SetParameterLowerBound(&theta, 0, lower);
		}
		cout << "Number of residual blocks: " << problem.NumResidualBlocks() << std::endl;

		// Set the solver
		Solver::Options options;
		options.max_num_iterations = iter;
		options.linear_solver_type = ceres::DENSE_QR;
		options.minimizer_progress_to_stdout = false;

		// Run the solver
		Solver::Summary summary;
		Solve(options, &problem, &summary);

		// Recover the theta
		cout << "final cost = " << summary.final_cost << ", theta = " << theta << endl;
		updatePose(seed, theta);
	}
}

void AICP::run_randomPara(PointCloud<PointXYZRGB> &cloud, RangeImagePlanar &planar, float pix_meter, vector<float> &pure_vec, Index<flann::L2<float>> &index)
{
	srand(time(NULL));
	for(int t = 0; t < times; t++)
	{
		cout << "start" << endl;
		int finger = -2, joint = -2;
		double theta = 0.0, upper = 0.0, lower = 0.0;
		int seed = getRandomJoint(finger, joint, theta, upper, lower);
		cout << "In time " << t << ", seed = " << seed << endl;/**/

		Problem problem;
		ceres::CostFunction* cf_curr;
		if(finger == -1)
		{
			cf_curr = new NumericDiffCostFunction<CF_Global, CENTRAL, 1, 1> (new CF_Global(cloud, joint, bestpose, /*planar,*/ pix_meter, pure_vec, index));
			problem.AddResidualBlock(cf_curr, NULL, &theta);
		}
		else
		{
			cf_curr = new NumericDiffCostFunction<CF_Finger, CENTRAL, 1, 1> (new CF_Finger(cloud, finger, joint, bestpose, /*planar,*/ pix_meter, pure_vec, &index));
			problem.AddResidualBlock(cf_curr, NULL, &theta);
			problem.SetParameterUpperBound(&theta, 0, upper);
			problem.SetParameterLowerBound(&theta, 0, lower);
		}
		cout << "Number of residual blocks: " << problem.NumResidualBlocks() << std::endl;

		// Set the solver
		Solver::Options options;
		options.max_num_iterations = iter;
		options.linear_solver_type = ceres::DENSE_QR;
		options.minimizer_progress_to_stdout = false;

		// Run the solver
		Solver::Summary summary;
		Solve(options, &problem, &summary);

		// Recover the theta
		cout << "final cost = " << summary.final_cost << ", theta = " << theta << endl;
		updatePose(seed, theta);
		cout << "next" << endl;
	}
}

void AICP::run_specPara(const PointCloud<PointXYZRGB> &cloud, int spec_para)
{
	PointCloud<PointXYZRGB> tmpcloud = cloud;
	int finger = -2, joint = -2;
	double theta = 0.0, upper = 0.0, lower = 0.0;
	getSpecJoint(spec_para, finger, joint, theta, upper, lower);

	Problem problem;
	ceres::CostFunction* cf_curr;
	if(finger == -1)
	{
		cf_curr = new NumericDiffCostFunction<CF_Global, CENTRAL, 1, 1> (new CF_Global(tmpcloud, joint, bestpose));
		problem.AddResidualBlock(cf_curr, NULL, &theta);
	}
	else
	{
		cf_curr = new NumericDiffCostFunction<CF_Finger, CENTRAL, 1, 1> (new CF_Finger(tmpcloud, finger, joint, bestpose));
		problem.AddResidualBlock(cf_curr, NULL, &theta);
		problem.SetParameterUpperBound(&theta, 0, upper);
		problem.SetParameterLowerBound(&theta, 0, lower);
	}
//		cout << "Number of residual blocks: " << problem.NumResidualBlocks() << std::endl;

	// Set the solver
	Solver::Options options;
	options.max_num_iterations = iter;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;

	// Run the solver
	Solver::Summary summary;
	Solve(options, &problem, &summary);

	// Recover the theta
//		cout << "final cost = " << summary.final_cost << ", theta = " << theta << endl;
	updatePose(spec_para, theta);
}

void AICP::run_cyclePara(const PointCloud<PointXYZRGB> &cloud)
{
	for(int t = 0; t < (26 * times); t++)
	{
		PointCloud<PointXYZRGB> tmpcloud = cloud;
		int finger = -2, joint = -2;
		double theta = 0.0, upper = 0.0, lower = 0.0;
		int curr_para = (t + 20) % 26;
		if (curr_para == 24)	continue;
		getSpecJoint(curr_para, finger, joint, theta, upper, lower);

		Problem problem;
		ceres::CostFunction* cf_curr;
		if(finger == -1)
		{
			cf_curr = new NumericDiffCostFunction<CF_Global, CENTRAL, 1, 1> (new CF_Global(tmpcloud, joint, bestpose));
			problem.AddResidualBlock(cf_curr, NULL, &theta);
		}
		else
		{
			cf_curr = new NumericDiffCostFunction<CF_Finger, CENTRAL, 1, 1> (new CF_Finger(tmpcloud, finger, joint, bestpose));
			problem.AddResidualBlock(cf_curr, NULL, &theta);
			problem.SetParameterUpperBound(&theta, 0, upper);
			problem.SetParameterLowerBound(&theta, 0, lower);
		}
//		cout << "Number of residual blocks: " << problem.NumResidualBlocks() << std::endl;

		// Set the solver
		Solver::Options options;
		options.max_num_iterations = iter;
		options.linear_solver_type = ceres::DENSE_QR;
		options.minimizer_progress_to_stdout = false;

		// Run the solver
		Solver::Summary summary;
		Solve(options, &problem, &summary);

		// Recover the theta
//		cout << "final cost = " << summary.final_cost << ", theta = " << theta << endl;
		updatePose(curr_para, theta);
	}
}

void AICP::run_randomJoint(const PointCloud<PointXYZRGB> &cloud)
{
	srand(time(NULL));
	for(int t = 0; t < times; t++)
	{
		PointCloud<PointXYZRGB> tmpcloud = cloud;
		int finger = -3;
		double *theta, *upper, *lower;
		int seed = getRandomPart(finger, &theta, &upper, &lower);
		cout << "seed = " << seed << ", finger = " << finger;
		if(finger >= 0)
		{
			cout << ", theta = (" << theta[0] << ", " << theta[1] << ", " << theta[2] << ", " << theta[3] << ")" << endl;
			cout << "upper = (" << upper[0] << ", " << upper[1] << ", " << upper[2] << ", " << upper[3] << ")" << endl;
			cout << "lower = (" << lower[0] << ", " << lower[1] << ", " << lower[2] << ", " << lower[3] << ")" << endl;
		}
		else
			cout << ", theta = (" << theta[0] << ", " << theta[1] << ", " << theta[2] << ")" << endl;
		
		Problem problem;
		ceres::CostFunction* cf_curr;
		if(finger < 0)
		{
			bool ch = (finger == -1)? true: false;	
			cf_curr = new NumericDiffCostFunction<CF_Global_Comb, CENTRAL, 1, 1, 1, 1> (new CF_Global_Comb(tmpcloud, ch, bestpose));
			problem.AddResidualBlock(cf_curr, NULL, &theta[0], &theta[1], &theta[2]);
		}
		else
		{
			cf_curr = new NumericDiffCostFunction<CF_Finger_Joint, CENTRAL, 1, 1, 1, 1, 1> (new CF_Finger_Joint(tmpcloud, finger, bestpose));
			problem.AddResidualBlock(cf_curr, NULL, &theta[0], &theta[1], &theta[2], &theta[3]);
			for(int i = 0; i < 4; i++)
			{
				problem.SetParameterUpperBound(&theta[i], 0, upper[i]);
				problem.SetParameterLowerBound(&theta[i], 0, lower[i]);
			}
		}
		cout << "Number of residual blocks: " << problem.NumResidualBlocks() << std::endl;

		// Set the solver
		Solver::Options options;
		options.max_num_iterations = iter;
		options.linear_solver_type = ceres::DENSE_QR;
		options.minimizer_progress_to_stdout = true;

		// Run the solver
		Solver::Summary summary;
		Solve(options, &problem, &summary);

		// Recover the theta
		cout << "final cost = " << summary.final_cost;
		if(finger >= 0)
			cout << ", theta = (" << theta[0] << ", " << theta[1] << ", " << theta[2] << ", " << theta[3] << ")" << endl;
		else
			cout << ", theta = (" << theta[0] << ", " << theta[1] << ", " << theta[2] << ")" << endl;
/*		updatePose(seed, theta);*/
	}
}
