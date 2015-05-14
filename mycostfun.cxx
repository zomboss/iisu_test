#include <vector>
#include <math.h>
#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen"

using ceres::AutoDiffCostFunction;
using ceres::NumericDiffCostFunction;
using ceres::CENTRAL;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using namespace Eigen;

class CF_x {

public:
	CF_x(std::vector<double> o, std::vector<double> t, std::vector<double> p): point_o(o), point_t(t), point_p(p){}
	bool operator()(const double* const theta, const double* const phi, const double* const global, double* residual) const 
	{
//		residual[0] = x1[0] + T(10.0) * x2[0];
//		residual[0] = point_o[0] - point_p[0];
//		residual[0] = (point_o[0] - point_t[0]) * cos(*phi) + (-point_o[2] + point_t[2]) * sin(*phi) + point_t[0] - point_p[0];
//		residual[0] = (point_o[0] - point_t[0]) * cos(*phi) + (-point_o[1] + point_t[1]) * sin(*phi) * sin(*theta) + (-point_o[2] + point_t[2]) * sin(*phi) * cos(*theta) + point_t[0] - point_p[0];
//		residual[0] = (point_o[0] - point_t[0]) * cos(*phi) + (-point_o[1] + point_t[1]) * sin(*phi) * cos(*theta) + (point_o[2] - point_t[2]) * sin(*phi) * sin(*theta) + point_t[0] - point_p[0];
		residual[0] = (point_o[0] - point_t[0]) * (cos(*global) * cos(*phi) - sin(*global) * sin(*phi)) + 
					  (-point_o[1] + point_t[1]) * (cos(*global) * sin(*phi) * cos(*theta) + sin(*global) * cos(*phi) * cos(*theta)) + 
					  (point_o[2] - point_t[2]) * (cos(*global) * sin(*phi) * sin(*theta) + sin(*global) * cos(*phi) * sin(*theta)) + 
					  point_t[0] * cos(*global) - point_t[1] * sin(*global) - point_p[0];/**/
		return true;
	}
private:
	std::vector<double> point_o;
	std::vector<double> point_t;
	std::vector<double> point_p;

};

class CF_y {

public:
	CF_y(std::vector<double> o, std::vector<double> t, std::vector<double> p): point_o(o), point_t(t), point_p(p){}
	bool operator()(const double* const theta, const double* const phi, const double* const global, double* residual) const 
	{
//		residual[0] = x1[0] + T(10.0) * x2[0];
//		residual[0] = (point_o[1] + point_t[1]) * cos(*theta) - (point_o[2] + point_t[2]) * sin(*theta) - point_t[1] - point_p[1];
//		residual[0] = point_o[1] - point_p[1];
//		residual[0] = (point_o[1] - point_t[1]) * cos(*theta) + (-point_o[2] + point_t[2]) * sin(*theta) + point_t[1] - point_p[1];
//		residual[0] = (point_o[0] - point_t[0]) * sin(*phi) + (point_o[1] - point_t[1]) * cos(*phi) * cos(*theta) + (-point_o[2] + point_t[2]) * cos(*phi) * sin(*theta) + point_t[1] - point_p[1];
		residual[0] = (point_o[0] - point_t[0]) * (sin(*global) * cos(*phi) + cos(*global) * sin(*phi)) + 
					  (-point_o[1] + point_t[1]) * (sin(*global) * sin(*phi) * cos(*theta) - cos(*global) * cos(*phi) * cos(*theta)) + 
					  (point_o[2] - point_t[2]) * (sin(*global) * sin(*phi) * sin(*theta) - cos(*global) * cos(*phi) * sin(*theta)) + 
					  point_t[0] * sin(*global) + point_t[1] * cos(*global) - point_p[1];/**/
		return true;
	}
private:
	std::vector<double> point_o;
	std::vector<double> point_t;
	std::vector<double> point_p;

};

class CF_z {

public:
	CF_z(std::vector<double> o, std::vector<double> t, std::vector<double> p): point_o(o), point_t(t), point_p(p){}
	bool operator()(const double* const theta, const double* const phi, const double* const global, double* residual) const
	{
//		residual[0] = x1[0] + T(10.0) * x2[0];
//		residual[0] = (point_o[1] + point_t[1]) * sin(*theta) + (point_o[2] + point_t[2]) * cos(*theta) - point_t[2] - point_p[2];
//		residual[0] = (point_o[0] - point_t[0]) * sin(*phi) + (point_o[2] - point_t[2]) * cos(*phi) + point_t[2] - point_p[2];
//		residual[0] = (point_o[0] - point_t[0]) * sin(*phi) + (point_o[1] - point_t[1]) * cos(*phi) * sin(*theta) + (point_o[2] - point_t[2]) * cos(*phi) * cos(*theta) + point_t[2] - point_p[2];
		residual[0] = (point_o[1] - point_t[1]) * sin(*theta) + (point_o[2] - point_t[2]) * cos(*theta) + point_t[2] - point_p[2];
		return true;
	}
private:
	std::vector<double> point_o;
	std::vector<double> point_t;
	std::vector<double> point_p;

};

class CF_d {

public:
	CF_d(std::vector<double> o, std::vector<double> t, std::vector<double> p,  std::vector<double> d): point_o(o), point_t(t), point_p(p), direction(d){}
	bool operator()(const double* const theta, const double* const phi, const double* const global, double* residual) const
	{
		// get the direction now
		std::vector<double> startpoint(3, 0.0);
/*		startpoint[0] = (point_o[0] - point_t[0]) * cos(*phi) + (-point_o[1] + point_t[1]) * sin(*phi) * cos(*theta) + (point_o[2] - point_t[2]) * sin(*phi) * sin(*theta) + point_t[0];
		startpoint[1] = (point_o[0] - point_t[0]) * sin(*phi) + (point_o[1] - point_t[1]) * cos(*phi) * cos(*theta) + (-point_o[2] + point_t[2]) * cos(*phi) * sin(*theta) + point_t[1];
		startpoint[2] = (point_o[1] - point_t[1]) * sin(*theta) + (point_o[2] - point_t[2]) * cos(*theta) + point_t[2];*/
		startpoint[0] = (point_o[0] - point_t[0]) * (cos(*global) * cos(*phi) - sin(*global) * sin(*phi)) + 
						(-point_o[1] + point_t[1]) * (cos(*global) * sin(*phi) * cos(*theta) + sin(*global) * cos(*phi) * cos(*theta)) + 
						(point_o[2] - point_t[2]) * (cos(*global) * sin(*phi) * sin(*theta) + sin(*global) * cos(*phi) * sin(*theta)) + 
						point_t[0] * cos(*global) - point_t[1] * sin(*global);
		startpoint[1] = (point_o[0] - point_t[0]) * (sin(*global) * cos(*phi) + cos(*global) * sin(*phi)) + 
						(-point_o[1] + point_t[1]) * (sin(*global) * sin(*phi) * cos(*theta) - cos(*global) * cos(*phi) * cos(*theta)) + 
						(point_o[2] - point_t[2]) * (sin(*global) * sin(*phi) * sin(*theta) - cos(*global) * cos(*phi) * sin(*theta)) + 
						point_t[0] * sin(*global) + point_t[1] * cos(*global);
		startpoint[2] = (point_o[1] - point_t[1]) * sin(*theta) + (point_o[2] - point_t[2]) * cos(*theta) + point_t[2];
		std::vector<double> endpoint = point_t;
		std::vector<double> nowdir = endpoint;
		nowdir[0] -= startpoint[0];nowdir[1] -= startpoint[1];nowdir[2] -= startpoint[2];

		// use Eigen to calculate angle -> residual
		Vector3f ei_dir_tar(direction[0], direction[1], direction[2]);
		Vector3f ei_dir_now(nowdir[0], nowdir[1], nowdir[2]);
		ei_dir_tar.normalize();
		ei_dir_now.normalize();
		residual[0] = acos(ei_dir_tar.dot(ei_dir_now));

		return true;
	}
private:
	std::vector<double> point_o;
	std::vector<double> point_t;
	std::vector<double> point_p;
	std::vector<double> direction;

};

class CF_x_thumb {

public:
	CF_x_thumb(std::vector<double> o, std::vector<double> t, std::vector<double> p): point_o(o), point_t(t), point_p(p){}
	bool operator()(const double* const theta, const double* const phi, const double* const global, double* residual) const 
	{
/*		residual[0] = (point_o[0] - point_t[0]) * (cos(*global) * cos(*phi) - sin(*global) * sin(*phi)) + 
					  (-point_o[1] + point_t[1]) * (cos(*global) * sin(*phi) * cos(*theta) + sin(*global) * cos(*phi) * cos(*theta)) + 
					  (point_o[2] - point_t[2]) * (cos(*global) * sin(*phi) * sin(*theta) + sin(*global) * cos(*phi) * sin(*theta)) + 
					  point_t[0] * cos(*global) - point_t[1] * sin(*global) - point_p[0];*/
		residual[0] = (point_o[0] - point_t[0]) * (cos(*global) * cos(*phi) * cos(*theta) - sin(*global) * sin(*theta)) + 
					  (-point_o[1] + point_t[1]) * (cos(*global) * cos(*phi) * sin(*theta) + sin(*global) * cos(*theta)) + 
					  (-point_o[2] + point_t[2]) * (cos(*global) * sin(*phi)) + point_t[0] * cos(*global) - point_t[1] * sin(*global) - point_p[0];
		return true;
	}
private:
	std::vector<double> point_o;
	std::vector<double> point_t;
	std::vector<double> point_p;

};

class CF_y_thumb {

public:
	CF_y_thumb(std::vector<double> o, std::vector<double> t, std::vector<double> p): point_o(o), point_t(t), point_p(p){}
	bool operator()(const double* const theta, const double* const phi, const double* const global, double* residual) const 
	{
/*		residual[0] = (point_o[0] - point_t[0]) * (sin(*global) * cos(*phi) + cos(*global) * sin(*phi)) + 
					  (-point_o[1] + point_t[1]) * (sin(*global) * sin(*phi) * cos(*theta) - cos(*global) * cos(*phi) * cos(*theta)) + 
					  (point_o[2] - point_t[2]) * (sin(*global) * sin(*phi) * sin(*theta) - cos(*global) * cos(*phi) * sin(*theta)) + 
					  point_t[0] * sin(*global) + point_t[1] * cos(*global) - point_p[1];*/
		residual[0] = (point_o[0] - point_t[0]) * (sin(*global) * cos(*phi) * cos(*theta) + cos(*global) * sin(*phi)) + 
					  (-point_o[1] + point_t[1]) * (sin(*global) * cos(*phi) * sin(*theta) - cos(*global) * cos(*theta)) + 
					  (-point_o[2] + point_t[2]) * (sin(*global) * sin(*phi)) + point_t[0] * sin(*global) + point_t[1] * cos(*global) - point_p[1];
		return true;
	}
private:
	std::vector<double> point_o;
	std::vector<double> point_t;
	std::vector<double> point_p;

};

class CF_z_thumb {

public:
	CF_z_thumb(std::vector<double> o, std::vector<double> t, std::vector<double> p): point_o(o), point_t(t), point_p(p){}
	bool operator()(const double* const theta, const double* const phi, const double* const global, double* residual) const
	{
/*		residual[0] = (point_o[1] - point_t[1]) * sin(*theta) + (point_o[2] - point_t[2]) * cos(*theta) + point_t[2] - point_p[2];*/
		residual[0] = (point_o[0] - point_t[0]) * sin(*phi) * cos(*theta) + 
					  (-point_o[1] + point_t[1]) * sin(*phi) * sin(*theta) + 
					  (point_o[2] - point_t[2]) * cos(*phi) + point_t[2] - point_p[2];
		return true;
	}
private:
	std::vector<double> point_o;
	std::vector<double> point_t;
	std::vector<double> point_p;

};

class CF_d_thumb {

public:
	CF_d_thumb(std::vector<double> o, std::vector<double> t, std::vector<double> p,  std::vector<double> d): point_o(o), point_t(t), point_p(p), direction(d){}
	bool operator()(const double* const theta, const double* const phi, const double* const global, double* residual) const
	{
		// get the direction now
		std::vector<double> startpoint(3, 0.0);
		startpoint[0] = (point_o[0] - point_t[0]) * (cos(*global) * cos(*phi) * cos(*theta) - sin(*global) * sin(*theta)) + 
						(-point_o[1] + point_t[1]) * (cos(*global) * cos(*phi) * sin(*theta) + sin(*global) * cos(*theta)) + 
						(-point_o[2] + point_t[2]) * (cos(*global) * sin(*phi)) + point_t[0] * cos(*global) - point_t[1] * sin(*global);
		startpoint[1] = (point_o[0] - point_t[0]) * (sin(*global) * cos(*phi) * cos(*theta) + cos(*global) * sin(*phi)) + 
						(-point_o[1] + point_t[1]) * (sin(*global) * cos(*phi) * sin(*theta) - cos(*global) * cos(*theta)) + 
						(-point_o[2] + point_t[2]) * (sin(*global) * sin(*phi)) + point_t[0] * sin(*global) + point_t[1] * cos(*global);
		startpoint[2] = (point_o[0] - point_t[0]) * sin(*phi) * cos(*theta) + 
						(-point_o[1] + point_t[1]) * sin(*phi) * sin(*theta) + 
						(point_o[2] - point_t[2]) * cos(*phi) + point_t[2];
		std::vector<double> endpoint = point_t;
		std::vector<double> nowdir = endpoint;
		nowdir[0] -= startpoint[0];nowdir[1] -= startpoint[1];nowdir[2] -= startpoint[2];

		// use Eigen to calculate angle -> residual
		Vector3f ei_dir_tar(direction[0], direction[1], direction[2]);
		Vector3f ei_dir_now(nowdir[0], nowdir[1], nowdir[2]);
		ei_dir_tar.normalize();
		ei_dir_now.normalize();
		residual[0] = acos(ei_dir_tar.dot(ei_dir_now));

		return true;
	}
private:
	std::vector<double> point_o;
	std::vector<double> point_t;
	std::vector<double> point_p;
	std::vector<double> direction;

};



std::vector<double> getTarget(int index)
{
	// index = 0: index finger, index = 1: middle, index = 2: ring, index = 3: little, index = 4: thumb
	double tar[] = {65.0365, 88.5227, 21.3156, 96.0187, 58.8531, -27.6308, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	std::vector<double> point(tar + (index * 3), tar + (index * 3) + 3);
	return point;
}

std::vector<double> getDirection(int index)
{
	// index = 0: index finger, index = 1: middle, index = 2: ring, index = 3: little, index = 4: thumb
	double tar[] = {-61.7497, -74.637, -16.7258, -79.7898, -54.1132, 18.5948, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	std::vector<double> point(tar + (index * 3), tar + (index * 3) + 3);
	return point;
}

std::vector<double> getTips(int index)
{
	// index = 0: index finger, index = 1: middle, index = 2: ring, index = 3: little, index = 4: thumb
	double tar[] = {-34.0, 107.0, 0.0, -12.0, 119.0, 0.0, 12.0, 117.0, 0.0, 34.0, 112.0, 0.0, -80.0, 47.0, 0.0};
	std::vector<double> point(tar + (index * 3), tar + (index * 3) + 3);
	return point;
}

std::vector<double> getJoint(int index)
{
	// index = 0: index finger, index = 1: middle, index = 2: ring, index = 3: little, index = 4: thumb
	double tar[] = {34.0, -37.0, 0.0, -12.0, 49.0, 0.0, 12.0, 47.0, 0.0, 34.0, 42.0, 0.0, -20.0, -23.0, 0.0};
	std::vector<double> point(tar + (index * 3), tar + (index * 3) + 3);
	return point;
}

int main(int argc, char** argv) {

	google::InitGoogleLogging(argv[0]);

	// Testing eigen here!!
	Vector3f vec(0.0,0.0,3.0);
	vec.normalize();
	std::cout << "eigen vector: (" << vec[0] << ", " << vec[1] << ", " << vec[2] << ")" << std::endl;

	// Initial guess
	double theta_array[5] = {0.0,0.0,0.0,0.0,0.0}, phi_array[5] = {0.0,0.0,0.0,0.0}, cost_array[5] = {0.0,0.0,0.0,0.0,0.0}, global = 0.0;
	int finger = 3;
	Problem problem;

	for(int i = 0; i < 2; i++)
	{
		std::cout << "In target " << i << " ..." << std::endl;
		// Initial guess
		//double theta = 0.0, phi = 0.0;
		// Because there is no Vector3, we use vccetor<double> instead
		std::vector<double> pointO = getTips(i);
		std::vector<double> pointT = getJoint(i);
		std::vector<double> pointP = getTarget(i);
		std::vector<double> direction = getDirection(i);
		CostFunction* cost_function_x;
		CostFunction* cost_function_y;
		CostFunction* cost_function_z;
		CostFunction* cost_function_d;
		if(i != 4)
		{
			cost_function_x = new NumericDiffCostFunction<CF_x, CENTRAL, 1, 1, 1, 1> (new CF_x(pointO, pointT, pointP));
			problem.AddResidualBlock(cost_function_x, NULL, &theta_array[i], &phi_array[i], &global);
			cost_function_y = new NumericDiffCostFunction<CF_y, CENTRAL, 1, 1, 1, 1> (new CF_y(pointO, pointT, pointP));
			problem.AddResidualBlock(cost_function_y, NULL, &theta_array[i], &phi_array[i], &global);
			cost_function_z = new NumericDiffCostFunction<CF_z, CENTRAL, 1, 1, 1, 1> (new CF_z(pointO, pointT, pointP));
			problem.AddResidualBlock(cost_function_z, NULL, &theta_array[i], &phi_array[i], &global);
			cost_function_d = new NumericDiffCostFunction<CF_d, CENTRAL, 1, 1, 1, 1> (new CF_d(pointO, pointT, pointP, direction));
			problem.AddResidualBlock(cost_function_d, NULL, &theta_array[i], &phi_array[i], &global);
			problem.SetParameterUpperBound(&theta_array[i], 0, M_PI / 2.0);
			problem.SetParameterLowerBound(&theta_array[i], 0, -M_PI / 18.0);
			problem.SetParameterUpperBound(&phi_array[i], 0, M_PI / 9.0);
			problem.SetParameterLowerBound(&phi_array[i], 0, -M_PI / 9.0);
		}
		else
		{
			cost_function_x = new NumericDiffCostFunction<CF_x_thumb, CENTRAL, 1, 1, 1, 1> (new CF_x_thumb(pointO, pointT, pointP));
			problem.AddResidualBlock(cost_function_x, NULL, &theta_array[i], &phi_array[i], &global);
			cost_function_y = new NumericDiffCostFunction<CF_y_thumb, CENTRAL, 1, 1, 1, 1> (new CF_y_thumb(pointO, pointT, pointP));
			problem.AddResidualBlock(cost_function_y, NULL, &theta_array[i], &phi_array[i], &global);
			cost_function_z = new NumericDiffCostFunction<CF_z_thumb, CENTRAL, 1, 1, 1, 1> (new CF_z_thumb(pointO, pointT, pointP));
			problem.AddResidualBlock(cost_function_z, NULL, &theta_array[i], &phi_array[i], &global);
			cost_function_d = new NumericDiffCostFunction<CF_d_thumb, CENTRAL, 1, 1, 1, 1> (new CF_d_thumb(pointO, pointT, pointP, direction));
			problem.AddResidualBlock(cost_function_d, NULL, &theta_array[i], &phi_array[i], &global);
			problem.SetParameterUpperBound(&theta_array[i], 0, M_PI / 9.0);
			problem.SetParameterLowerBound(&theta_array[i], 0, -M_PI / 15.0);
			problem.SetParameterUpperBound(&phi_array[i], 0, M_PI / 2.0);
			problem.SetParameterLowerBound(&phi_array[i], 0, -M_PI / 18.0);
		}
		

	}
	std::cout << "Number of residual blocks: " << problem.NumResidualBlocks() << std::endl;
	problem.SetParameterUpperBound(&global, 0, M_PI);
	problem.SetParameterLowerBound(&global, 0, -M_PI);

	Solver::Options options;
	options.max_num_iterations = 200;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;
	std::cout << "Initial theta = (" << theta_array[0] << ", " << theta_array[1] << ", " << theta_array[2] << ", " << theta_array[3] << ", " << theta_array[4] << ")\n"
			  << "phi = (" << phi_array[0] << ", " << phi_array[1] << ", " << phi_array[2] << ", " << phi_array[3] << ", " << phi_array[4] << ")\n"
			  << "global = " << global << std::endl;

	// Run the solver!
	Solver::Summary summary;
	Solve(options, &problem, &summary);
	std::cout << "Final cost = " << summary.final_cost << "\n";
	std::cout << "Final theta = (" << theta_array[0] << ", " << theta_array[1] << ", " << theta_array[2] << ", " << theta_array[3] << ", " << theta_array[4] << ")\n"
			  << "phi = (" << phi_array[0] << ", " << phi_array[1] << ", " << phi_array[2] << ", " << phi_array[3] << ", " << phi_array[4] << ")\n"
			  << "global = " << global << std::endl << std::endl;
  
	system("pause");
	return 0;
}
