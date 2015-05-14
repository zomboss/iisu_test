#include "Initialization.hxx"

Initialization::Initialization()
{


}

Initialization::Initialization(int num, SK::Array<Vector3> ptips, SK::Array<Vector3> pdirs, HandModel &model)
{
	finger_num = num;
	pc_tips = ptips;
	pc_dirs = pdirs;
	handmodel = model;
	for(int i = 0; i < 5; i++)
	{
		md_tips.pushBack(handmodel.getFingerTips(i));
		md_joints.pushBack(handmodel.getFingerBaseJoint(i));
		theta_array[i] = 0.0;
		phi_array[i] = 0.0;
	}
	cost = 0.0;
	global = 0.0;
	global_array[0] = 0.0;
	global_array[1] = 0.0;
	global_array[2] = 0.0;

}

Vector3 Initialization::getFingerParameter(int f)
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

void Initialization::chPCFeature(SK::Array<int> index)
{
	SK::Array<Vector3> oritips = pc_tips;
	SK::Array<Vector3> oridirs = pc_dirs;
	for(size_t i = 0; i < index.size(); i++)
	{
		pc_tips[i] = oritips[index[i]];
		pc_dirs[i] = oridirs[index[i]];
	}

}

void Initialization::setPCFeature(SK::Array<Vector3> ptips, SK::Array<Vector3> pdirs)
{
	pc_tips = ptips;
	pc_dirs = pdirs;
}

void Initialization::setResultPose(HandPose &pose)
{
	for(int i = 0; i < 5; i++)
	{
		if(finger_exist[i])
			pose.setFingerPose(i, getFingerParameter(i), Vector3(0, 0, 0), Vector3(0, 0, 0));
		else	// bend fingers
		{
			if(i != 0)
				pose.setFingerPose(i, Vector3(M_PI / 2.0, 0, 0), Vector3(M_PI / 1.8, 0, 0), Vector3(M_PI / 2.0, 0, 0));
			else
				pose.setFingerPose(i, Vector3(0, M_PI / 2.0, 0), Vector3(M_PI / 1.8, 0, 0), Vector3(M_PI / 2.0, 0, 0));
		}
	}
	pose.setRotation(Vector3(0, 0, global));
}

void Initialization::setFullResultPose(HandPose &pose)
{
	for(int i = 0; i < 5; i++)
	{
		if(finger_exist[i])
			pose.setFingerPose(i, getFingerParameter(i), Vector3(0, 0, 0), Vector3(0, 0, 0));
		else	// bend fingers
		{
			if(i != 0)
				pose.setFingerPose(i, Vector3(M_PI / 2.0, 0, 0), Vector3(M_PI / 1.8, 0, 0), Vector3(M_PI / 2.0, 0, 0));
			else
				pose.setFingerPose(i, Vector3(0, M_PI / 2.0, 0), Vector3(M_PI / 1.8, 0, 0), Vector3(M_PI / 2.0, 0, 0));
		}
	}
	pose.setRotation(Vector3(global_array[0], global_array[1], global_array[2]));
}

void Initialization::goInitail()
{
	Problem problem;
	
	if(finger_num == 0)	return;
	for(int i = 0; i < 5; i++)
	{
		if(!finger_exist[i])	continue;
		CostFunction* cost_function_x;
		CostFunction* cost_function_y;
		CostFunction* cost_function_z;
		CostFunction* cost_function_d;
		if(i != 0)	// others
		{
			cost_function_x = new NumericDiffCostFunction<CF_x, CENTRAL, 1, 1, 1, 1> (new CF_x(md_tips[i], md_joints[i], pc_tips[i]));
			problem.AddResidualBlock(cost_function_x, NULL, &theta_array[i], &phi_array[i], &global);
			cost_function_y = new NumericDiffCostFunction<CF_y, CENTRAL, 1, 1, 1, 1> (new CF_y(md_tips[i], md_joints[i], pc_tips[i]));
			problem.AddResidualBlock(cost_function_y, NULL, &theta_array[i], &phi_array[i], &global);
			cost_function_z = new NumericDiffCostFunction<CF_z, CENTRAL, 1, 1, 1, 1> (new CF_z(md_tips[i], md_joints[i], pc_tips[i]));
			problem.AddResidualBlock(cost_function_z, NULL, &theta_array[i], &phi_array[i], &global);
			cost_function_d = new NumericDiffCostFunction<CF_d, CENTRAL, 1, 1, 1, 1> (new CF_d(md_tips[i], md_joints[i], pc_tips[i], pc_dirs[i]));
			problem.AddResidualBlock(cost_function_d, NULL, &theta_array[i], &phi_array[i], &global);
			problem.SetParameterUpperBound(&theta_array[i], 0, INDEX_MCP_FE_UPPERBOUND);	// all rest fingers are same
			problem.SetParameterLowerBound(&theta_array[i], 0, INDEX_MCP_FE_LOWERBOUND);
			problem.SetParameterUpperBound(&phi_array[i], 0, INDEX_MCP_AA_UPPERBOUND);
			problem.SetParameterLowerBound(&phi_array[i], 0, INDEX_MCP_AA_LOWERBOUND);
		}
		else		// thumb
		{
			cost_function_x = new NumericDiffCostFunction<CF_x_thumb, CENTRAL, 1, 1, 1, 1> (new CF_x_thumb(md_tips[i], md_joints[i], pc_tips[i]));
			problem.AddResidualBlock(cost_function_x, NULL, &theta_array[i], &phi_array[i], &global);
			cost_function_y = new NumericDiffCostFunction<CF_y_thumb, CENTRAL, 1, 1, 1, 1> (new CF_y_thumb(md_tips[i], md_joints[i], pc_tips[i]));
			problem.AddResidualBlock(cost_function_y, NULL, &theta_array[i], &phi_array[i], &global);
			cost_function_z = new NumericDiffCostFunction<CF_z_thumb, CENTRAL, 1, 1, 1, 1> (new CF_z_thumb(md_tips[i], md_joints[i], pc_tips[i]));
			problem.AddResidualBlock(cost_function_z, NULL, &theta_array[i], &phi_array[i], &global);
			cost_function_d = new NumericDiffCostFunction<CF_d_thumb, CENTRAL, 1, 1, 1, 1> (new CF_d_thumb(md_tips[i], md_joints[i], pc_tips[i], pc_dirs[i]));
			problem.AddResidualBlock(cost_function_d, NULL, &theta_array[i], &phi_array[i], &global);
			problem.SetParameterUpperBound(&theta_array[i], 0, THUMB_MCP_AA_UPPERBOUND);
			problem.SetParameterLowerBound(&theta_array[i], 0, THUMB_MCP_AA_LOWERBOUND);
			problem.SetParameterUpperBound(&phi_array[i], 0, THUMB_MCP_FE_UPPERBOUND);
			problem.SetParameterLowerBound(&phi_array[i], 0, THUMB_MCP_FE_LOWERBOUND);
		}
	}
//	cout << "Number of residual blocks: " << problem.NumResidualBlocks() << std::endl;
	problem.SetParameterUpperBound(&global, 0, M_PI);
	problem.SetParameterLowerBound(&global, 0, -M_PI);

	// Set the solver
	Solver::Options options;
	options.max_num_iterations = 50;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;
/*	cout << "Initial theta = (" << theta_array[0] << ", " << theta_array[1] << ", " << theta_array[2] << ", " << theta_array[3] << ", " << theta_array[4] << ")\n"
		 << "phi = (" << phi_array[0] << ", " << phi_array[1] << ", " << phi_array[2] << ", " << phi_array[3] << ", " << phi_array[4] << ")\n"
		 << "global = " << global << std::endl;*/
	
	// Run the solver
	Solver::Summary summary;
	Solve(options, &problem, &summary);
	cost = summary.final_cost;
/*	cout << "Final cost = " << cost << "\n";
	cout << "Final theta = (" << theta_array[0] << ", " << theta_array[1] << ", " << theta_array[2] << ", " << theta_array[3] << ", " << theta_array[4] << ")\n"
		 << "phi = (" << phi_array[0] << ", " << phi_array[1] << ", " << phi_array[2] << ", " << phi_array[3] << ", " << phi_array[4] << ")\n"
		 << "global = " << global << std::endl << std::endl;*/

}

void Initialization::goInitail(HandPose &pose)
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
		CostFunction* cf_dis_tips = new NumericDiffCostFunction<CF_dis_tips, CENTRAL, 1, 1, 1, 1> (new CF_dis_tips(i, pc_tips[i]));
		problem.AddResidualBlock(cf_dis_tips, NULL, &theta_array[i], &phi_array[i], &global);
		CostFunction* cf_dir_tips = new NumericDiffCostFunction<CF_dir_tips, CENTRAL, 1, 1, 1, 1> (new CF_dir_tips(i, pc_dirs[i]));
		problem.AddResidualBlock(cf_dir_tips, NULL, &theta_array[i], &phi_array[i], &global);
		problem.SetParameterUpperBound(&theta_array[i], 0, thetaupper[i]);
		problem.SetParameterLowerBound(&theta_array[i], 0, thetalower[i]);
		problem.SetParameterUpperBound(&phi_array[i], 0, phiupper[i]);
		problem.SetParameterLowerBound(&phi_array[i], 0, philower[i]);
	}
	cout << "Number of residual blocks: " << problem.NumResidualBlocks() << std::endl;
	problem.SetParameterUpperBound(&global, 0, M_PI);
	problem.SetParameterLowerBound(&global, 0, -M_PI);

	// Set the solver
	Solver::Options options;
	options.max_num_iterations = 50;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;
	cout << "Initial theta = (" << theta_array[0] << ", " << theta_array[1] << ", " << theta_array[2] << ", " << theta_array[3] << ", " << theta_array[4] << ")\n"
		 << "phi = (" << phi_array[0] << ", " << phi_array[1] << ", " << phi_array[2] << ", " << phi_array[3] << ", " << phi_array[4] << ")\n"
		 << "global = " << global << std::endl;
	
	// Run the solver
	Solver::Summary summary;
	Solve(options, &problem, &summary);
	cost = summary.final_cost;
	cout << "Final cost = " << cost << "\n";
	cout << "Final theta = (" << theta_array[0] << ", " << theta_array[1] << ", " << theta_array[2] << ", " << theta_array[3] << ", " << theta_array[4] << ")\n"
		 << "phi = (" << phi_array[0] << ", " << phi_array[1] << ", " << phi_array[2] << ", " << phi_array[3] << ", " << phi_array[4] << ")\n"
		 << "global = " << global << std::endl;

	// Set the hand pose
	setResultPose(pose);
}

void Initialization::goFullInitail(HandPose &pose, bool show = true)
{
	Problem problem;
	double thetaupper[5] = {THUMB_MCP_AA_UPPERBOUND, INDEX_MCP_FE_UPPERBOUND, MIDDLE_MCP_FE_UPPERBOUND, RING_MCP_FE_UPPERBOUND, LITTLE_MCP_FE_UPPERBOUND}, 
		   thetalower[5] = {THUMB_MCP_AA_LOWERBOUND, INDEX_MCP_FE_LOWERBOUND, MIDDLE_MCP_FE_LOWERBOUND, RING_MCP_FE_LOWERBOUND, LITTLE_MCP_FE_LOWERBOUND}, 
		   phiupper[5] = {THUMB_MCP_FE_UPPERBOUND, INDEX_MCP_AA_UPPERBOUND, MIDDLE_MCP_AA_UPPERBOUND, RING_MCP_AA_UPPERBOUND, LITTLE_MCP_AA_UPPERBOUND}, 
		   philower[5] = {THUMB_MCP_FE_LOWERBOUND, INDEX_MCP_AA_LOWERBOUND, MIDDLE_MCP_AA_LOWERBOUND, RING_MCP_AA_LOWERBOUND, LITTLE_MCP_AA_LOWERBOUND};

	Vector3 pos = pose.getPosition();
	Vector3 ori = pose.getOrientation();
	
	if(finger_num == 0)	return;
	for(int i = 0; i < 5; i++)
	{
		if(!finger_exist[i])	continue;
		CostFunction* cf_dis_tips_full = new NumericDiffCostFunction<CF_dis_tips_full, CENTRAL, 1, 1, 1, 1, 1, 1> (new CF_dis_tips_full(i, pc_tips[i], pos, ori));
		problem.AddResidualBlock(cf_dis_tips_full, NULL, &theta_array[i], &phi_array[i], &global_array[0], &global_array[1], &global_array[2]);
		CostFunction* cf_dir_tips_full = new NumericDiffCostFunction<CF_dir_tips_full, CENTRAL, 1, 1, 1, 1, 1, 1> (new CF_dir_tips_full(i, pc_dirs[i], pos, ori));
		problem.AddResidualBlock(cf_dir_tips_full, NULL, &theta_array[i], &phi_array[i], &global_array[0], &global_array[1], &global_array[2]);
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
	options.linear_solver_type = ceres::DENSE_QR;
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
		cout << "Final cost = " << cost << "\n";
		cout << "Final theta = (" << theta_array[0] << ", " << theta_array[1] << ", " << theta_array[2] << ", " << theta_array[3] << ", " << theta_array[4] << ")\n"
			 << "phi = (" << phi_array[0] << ", " << phi_array[1] << ", " << phi_array[2] << ", " << phi_array[3] << ", " << phi_array[4] << ")\n"
			 << "global = (" << global_array[0] << ", " << global_array[1] << ", " << global_array[2] << ")\n" ;
	}

	// Set the hand pose
	setResultPose(pose);
}

void Initialization::fingerExist(SK::Array<bool> isf)
{
	finger_exist[0] = isf[0];
	finger_exist[1] = isf[1];
	finger_exist[2] = isf[2];
	finger_exist[3] = isf[3];
	finger_exist[4] = isf[4];
}