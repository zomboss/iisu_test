#include "ICPPCA.hxx"
#include "ceres/ceres.h"

using namespace ceres;

ICPPCA::ICPPCA()
{


}

ICPPCA::ICPPCA(int t, int it, int l, const HandPose &pose, const MatrixXf &mat, const VectorXf &mean, const VectorXf &max, const VectorXf &min)
{
	times = t;
	iter = it;
	length = l;
	bestpose = pose;
	trans_matrix = mat;
	mean_vector = mean;
	min_vector = min;
	max_vector = max;
}

void ICPPCA::updateGlobal(double *para)
{
	SK::Vector3 pos = SK::Vector3((float)para[0], (float)para[1], (float)para[2]);
	SK::Vector3 rot = SK::Vector3((float)para[3], (float)para[4], (float)para[5]);
	bestpose.setPosition(pos);
	bestpose.setRotation(rot);
}

void ICPPCA::updatePose(double *para)
{
	// recover to 26-d parameters
	SK::Vector3 pos_vec = bestpose.getPosition();
	SK::Vector3 rot_vec = bestpose.getRotation();
	Eigen::VectorXf curr_pca_para;
	curr_pca_para.resize(length + 6);
	for(int i = 0; i < length; i++)	curr_pca_para[i] = (float)para[i];
	curr_pca_para[length] = pos_vec[0];	curr_pca_para[length + 1] = pos_vec[1];	curr_pca_para[length + 2] = pos_vec[2];
	curr_pca_para[length + 3] = rot_vec[0];	curr_pca_para[length + 4] = rot_vec[1];	curr_pca_para[length + 5] = rot_vec[2];
	Eigen::VectorXf curr_nor_para = Eigen::VectorXf::Zero(26);
	curr_nor_para = trans_matrix.transpose() * curr_pca_para + mean_vector;
	SK::Array<float> curr_nor_array = MyTools::EigentoSKVector(curr_nor_para);
	bestpose.setAllParameters(curr_nor_array);

}

void ICPPCA::run(const PointCloud<PointXYZRGB> &cloud)
{
	for(int t = 0; t < times; t++)
	{
		if(t == 0)
		{
			// Run global first
			double *global_para = new double[6];
			for(int i = 0; i < 3; i++)	global_para[i] = (double)bestpose.getPosition()[i];
			for(int i = 0; i < 3; i++)	global_para[i + 3] = (double)bestpose.getRotation()[i];
	
			Problem problem;
			ceres::CostFunction* cf_curr = new NumericDiffCostFunction<CF_PCA_Global, CENTRAL, 1, 1, 1, 1, 1, 1, 1> (new CF_PCA_Global(cloud, bestpose));
//			ceres::CostFunction* cf_curr = new AutoDiffCostFunction<CF_PCA_Global, NULL, 1, 1, 1, 1, 1, 1, 1> (new CF_PCA_Global(cloud, bestpose));
			problem.AddResidualBlock(cf_curr, NULL, &global_para[0], &global_para[1], &global_para[2], &global_para[3], &global_para[4], &global_para[5]);
			
			// Set the solver
			Solver::Options options;
			options.max_num_iterations = iter;
			options.linear_solver_type = ceres::DENSE_QR;
			options.minimizer_progress_to_stdout = false;

			// Run the solver
			Solver::Summary summary;
			Solve(options, &problem, &summary);

			// Update the parameter
			updateGlobal(global_para);
			cost = std::sqrt(summary.final_cost);
		}
		else
		{
			// Construct pca parameters
			VectorXf nor_vec = MyTools::SKtoEigenVector(bestpose.getAllParameters());
			VectorXf pca_vec = Eigen::VectorXf::Zero(length + 6);
			pca_vec = trans_matrix * (nor_vec - mean_vector);
			double *pose_para = new double[length];
			for(int i = 0; i < length; i++)	pose_para[i] = (double)pca_vec[i];

			Problem problem;
			ceres::CostFunction* cf_curr;
			switch(length)
			{
			case 1:
				cf_curr = new NumericDiffCostFunction<CF_PCA_Joint, CENTRAL, 1, 1> (new CF_PCA_Joint(cloud, bestpose, trans_matrix.transpose(), mean_vector));
				problem.AddResidualBlock(cf_curr, NULL, &pose_para[0]);
				break;
			case 2:
				cf_curr = new NumericDiffCostFunction<CF_PCA_Joint, CENTRAL, 1, 1, 1> (new CF_PCA_Joint(cloud, bestpose, trans_matrix.transpose(), mean_vector));
				problem.AddResidualBlock(cf_curr, NULL, &pose_para[0], &pose_para[1]);
				break;
			case 3:
				cf_curr = new NumericDiffCostFunction<CF_PCA_Joint, CENTRAL, 1, 1, 1, 1> (new CF_PCA_Joint(cloud, bestpose, trans_matrix.transpose(), mean_vector));
				problem.AddResidualBlock(cf_curr, NULL, &pose_para[0], &pose_para[1], &pose_para[2]);
				break;
			case 4:
				cf_curr = new NumericDiffCostFunction<CF_PCA_Joint, CENTRAL, 1, 1, 1, 1, 1> (new CF_PCA_Joint(cloud, bestpose, trans_matrix.transpose(), mean_vector));
				problem.AddResidualBlock(cf_curr, NULL, &pose_para[0], &pose_para[1], &pose_para[2], &pose_para[3]);
				break;
			case 5:
				cf_curr = new NumericDiffCostFunction<CF_PCA_Joint, CENTRAL, 1, 1, 1, 1, 1, 1> (new CF_PCA_Joint(cloud, bestpose, trans_matrix.transpose(), mean_vector));
				problem.AddResidualBlock(cf_curr, NULL, &pose_para[0], &pose_para[1], &pose_para[2], &pose_para[3], &pose_para[4]);
				break;
			case 6:
				cf_curr = new NumericDiffCostFunction<CF_PCA_Joint, CENTRAL, 1, 1, 1, 1, 1, 1, 1> (new CF_PCA_Joint(cloud, bestpose, trans_matrix.transpose(), mean_vector));
				problem.AddResidualBlock(cf_curr, NULL, &pose_para[0], &pose_para[1], &pose_para[2], &pose_para[3], &pose_para[4], &pose_para[5]);
				break;
			}

			for(int i = 0; i < length; i++)
			{
				problem.SetParameterUpperBound(&pose_para[i], 0, (double)max_vector[i]);
				problem.SetParameterLowerBound(&pose_para[i], 0, (double)min_vector[i]);
			}

			// Set the solver
			Solver::Options options;
			options.max_num_iterations = iter;
			options.linear_solver_type = ceres::DENSE_QR;
			options.minimizer_progress_to_stdout = false;

			// Run the solver
			Solver::Summary summary;
			Solve(options, &problem, &summary);

			// Update the parameter
			updatePose(pose_para);
			cost = std::sqrt(summary.final_cost);
		}


	}
}

HandPose ICPPCA::pureTrans(int length)
{
	VectorXf ori_vec = MyTools::SKtoEigenVector(bestpose.getAllParameters());
	VectorXf pca_vec = Eigen::VectorXf::Zero(length + 6);
	pca_vec = trans_matrix * (ori_vec - mean_vector);
	VectorXf aft_vec = VectorXf::Zero(26);
	aft_vec = trans_matrix.transpose() * pca_vec + mean_vector;
	HandPose newpose = HandPose();
	newpose.setAllParameters(MyTools::EigentoSKVector(aft_vec));
	newpose.setOrientation(bestpose.getOrientation());
	return newpose;
}