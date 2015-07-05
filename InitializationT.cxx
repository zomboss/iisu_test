#include "InitializationT.hxx"

InitializationT::InitializationT()
{

}

InitializationT::InitializationT(int f_num, Eigen::Matrix<float, 3, 5> tips, Eigen::Matrix<float, 3, 5> dirs)
{
	finger_num = f_num;
	pc_tips = tips;
	pc_dirs = dirs;
	for(int i = 0; i < 5; i++)
	{
		theta_array[i] = 0.0;
		phi_array[i] = 0.0;
	}
	cost = 0.0;
	global_array[0] = 0.0;
	global_array[1] = 0.0;
	global_array[2] = 0.0;
}

Vector3 InitializationT::getFingerParameter(int f)
{
	// <Order> 0:thumb, 1:index, 2:middle, 3:ring, 4:little
	switch(f)
	{
	case 0:
		return Vector3(0.0, phi_array[f], theta_array[f]);
	case 1:
	case 2:
	case 3:
	case 4:
		return Vector3(theta_array[f], 0.0, phi_array[f]);
	default:
		return Vector3();
	}
}

void InitializationT::setPCFeature(Eigen::Matrix<float, 3, 5> tips, Eigen::Matrix<float, 3, 5> dirs)
{
	pc_tips = tips;
	pc_dirs = dirs;
}

void InitializationT::setFullResultPose(HandPose &pose)
{
	for(int i = 0; i < 5; i++)
	{
		if(finger_exist[i])
			pose.setFingerPose(i, getFingerParameter(i), SK::Vector3(0, 0, 0), SK::Vector3(0, 0, 0));
		else	// bend fingers
		{
			if(i != 0)	// all assume index(it's same)
				pose.setFingerPose(i, SK::Vector3(INDEX_MCP_FE_UPPERBOUND, 0, 0), SK::Vector3(INDEX_PIP_FE_UPPERBOUND, 0, 0), SK::Vector3(INDEX_DIP_FE_UPPERBOUND, 0, 0));
			else
				pose.setFingerPose(i, SK::Vector3(0, THUMB_MCP_FE_UPPERBOUND, 0), SK::Vector3(THUMB_PIP_FE_UPPERBOUND, 0, 0), SK::Vector3(THUMB_DIP_FE_UPPERBOUND, 0, 0));
		}
	}
	pose.setRotation(SK::Vector3(global_array[0], global_array[1], global_array[2]));
}

void InitializationT::fingerExist(SK::Array<bool> isf)
{
	finger_exist[0] = isf[0];
	finger_exist[1] = isf[1];
	finger_exist[2] = isf[2];
	finger_exist[3] = isf[3];
	finger_exist[4] = isf[4];
}

void InitializationT::goInitail(HandPose &pose, bool show = true)
{
	Problem problem;
	double thetaupper[5] = {THUMB_MCP_AA_UPPERBOUND, INDEX_MCP_FE_UPPERBOUND, MIDDLE_MCP_FE_UPPERBOUND, RING_MCP_FE_UPPERBOUND, LITTLE_MCP_FE_UPPERBOUND}, 
		   thetalower[5] = {THUMB_MCP_AA_LOWERBOUND, INDEX_MCP_FE_LOWERBOUND, MIDDLE_MCP_FE_LOWERBOUND, RING_MCP_FE_LOWERBOUND, LITTLE_MCP_FE_LOWERBOUND}, 
		   phiupper[5] = {THUMB_MCP_FE_UPPERBOUND, INDEX_MCP_AA_UPPERBOUND, MIDDLE_MCP_AA_UPPERBOUND, RING_MCP_AA_UPPERBOUND, LITTLE_MCP_AA_UPPERBOUND}, 
		   philower[5] = {THUMB_MCP_FE_LOWERBOUND, INDEX_MCP_AA_LOWERBOUND, MIDDLE_MCP_AA_LOWERBOUND, RING_MCP_AA_LOWERBOUND, LITTLE_MCP_AA_LOWERBOUND};
	
	if(finger_num == 0)	return;
	for(int i = 0; i < 5; i++)
	{
		if(!finger_exist[i])	continue;
/*		CostFunction* cf_tips_t = new AutoDiffCostFunction<CF_tips_T, 1, 1, 1, 1, 1, 1> (new CF_tips_T(i, pc_tips.col(i), pose));
		problem.AddResidualBlock(cf_tips_t, NULL, &theta_array[i], &phi_array[i], &global_array[0], &global_array[1], &global_array[2]);
		CostFunction* cf_dirs_t = new AutoDiffCostFunction<CF_dirs_T, 1, 1, 1, 1, 1, 1> (new CF_dirs_T(i, pc_dirs.col(i), pose));
		problem.AddResidualBlock(cf_dirs_t, NULL, &theta_array[i], &phi_array[i], &global_array[0], &global_array[1], &global_array[2]);*/
		CostFunction* cf_tips_t = new NumericDiffCostFunction<CF_tips_T, CENTRAL, 1, 1, 1, 1, 1, 1> (new CF_tips_T(i, pc_tips.col(i), pose));
		problem.AddResidualBlock(cf_tips_t, NULL, &theta_array[i], &phi_array[i], &global_array[0], &global_array[1], &global_array[2]);
		CostFunction* cf_dirs_t = new NumericDiffCostFunction<CF_dirs_T, CENTRAL, 1, 1, 1, 1, 1, 1> (new CF_dirs_T(i, pc_dirs.col(i), pose));
		problem.AddResidualBlock(cf_dirs_t, NULL, &theta_array[i], &phi_array[i], &global_array[0], &global_array[1], &global_array[2]);
		problem.SetParameterUpperBound(&theta_array[i], 0, thetaupper[i]);
		problem.SetParameterLowerBound(&theta_array[i], 0, thetalower[i]);
		problem.SetParameterUpperBound(&phi_array[i], 0, phiupper[i]);
		problem.SetParameterLowerBound(&phi_array[i], 0, philower[i]);
	}
	if(show)	cout << "Number of residual blocks: " << problem.NumResidualBlocks() << std::endl;
	problem.SetParameterUpperBound(&global_array[0], 0, M_PI);
	problem.SetParameterLowerBound(&global_array[0], 0, -M_PI);
	problem.SetParameterUpperBound(&global_array[1], 0, M_PI);
	problem.SetParameterLowerBound(&global_array[1], 0, -M_PI);
	problem.SetParameterUpperBound(&global_array[2], 0, M_PI);
	problem.SetParameterLowerBound(&global_array[2], 0, -M_PI);

	// Set the solver
	Solver::Options options;
	options.max_num_iterations = 10;
	options.preconditioner_type = ceres::SCHUR_JACOBI;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.minimizer_progress_to_stdout = show;
	if(show)
		cout << "Initial theta = (" << theta_array[0] << ", " << theta_array[1] << ", " << theta_array[2] << ", " << theta_array[3] << ", " << theta_array[4] << ")\n"
			 << "phi = (" << phi_array[0] << ", " << phi_array[1] << ", " << phi_array[2] << ", " << phi_array[3] << ", " << phi_array[4] << ")\n"
			 << "global = (" << global_array[0] << ", " << global_array[1] << ", " << global_array[2] << ")\n" ;
	
	// Run the solver
	Solver::Summary summary;
	Solve(options, &problem, &summary);
	cost = summary.final_cost;
	if(show)
	{
		cout << "report: " << endl << summary.FullReport() << endl;
		cout << "Final cost = " << cost << "\n";
		cout << "Final theta = (" << theta_array[0] << ", " << theta_array[1] << ", " << theta_array[2] << ", " << theta_array[3] << ", " << theta_array[4] << ")\n"
			 << "phi = (" << phi_array[0] << ", " << phi_array[1] << ", " << phi_array[2] << ", " << phi_array[3] << ", " << phi_array[4] << ")\n"
			 << "global = (" << global_array[0] << ", " << global_array[1] << ", " << global_array[2] << ")\n" ;
	}

	// Set the hand pose
	setFullResultPose(pose);

}