#include "HandPose.hxx"

HandPose::HandPose()
{
	HandPose(Vector3(0,0,0), Vector3(0,0,0), Vector3(0,0,0), 
			 Vector3(0,0,0), Vector3(0,0,0), Vector3(0,0,0), 
			 Vector3(0,0,0), Vector3(0,0,0), Vector3(0,0,0), 
			 Vector3(0,0,0), Vector3(0,0,0), Vector3(0,0,0), 
			 Vector3(0,0,0), Vector3(0,0,0), Vector3(0,0,0), 
			 Vector3(0,0,0), Vector3(0,0,1), Vector3(0,0,0));
}

HandPose::HandPose(Vector3 thumb_base, Vector3 thumb_mid, Vector3 thumb_top, 
				   Vector3 index_base, Vector3 index_mid, Vector3 index_top, 
				   Vector3 middle_base, Vector3 middle_mid, Vector3 middle_top, 
				   Vector3 ring_base, Vector3 ring_mid, Vector3 ring_top, 
				   Vector3 little_base, Vector3 little_mid, Vector3 little_top)
{
	j_thumb_base = thumb_base; j_thumb_mid = thumb_mid; j_thumb_top = thumb_top;
	j_index_base = index_base; j_index_mid = index_mid; j_index_top = index_top;
	j_middle_base = middle_base; j_middle_mid = middle_mid; j_middle_top = middle_top;
	j_ring_base = ring_base; j_ring_mid = ring_mid; j_ring_top = ring_top;
	j_little_base = little_base; j_little_mid = little_mid; j_little_top = little_top;
	global_pos = Vector3(0,0,0);
	global_ori = Vector3(0,0,1);
	global_rot = Vector3(0,0,0);
}

HandPose::HandPose(Vector3 thumb_base, Vector3 thumb_mid, Vector3 thumb_top, 
				   Vector3 index_base, Vector3 index_mid, Vector3 index_top, 
				   Vector3 middle_base, Vector3 middle_mid, Vector3 middle_top, 
				   Vector3 ring_base, Vector3 ring_mid, Vector3 ring_top, 
				   Vector3 little_base, Vector3 little_mid, Vector3 little_top,
				   Vector3 position, Vector3 orientation)
{
	j_thumb_base = thumb_base; j_thumb_mid = thumb_mid; j_thumb_top = thumb_top;
	j_index_base = index_base; j_index_mid = index_mid; j_index_top = index_top;
	j_middle_base = middle_base; j_middle_mid = middle_mid; j_middle_top = middle_top;
	j_ring_base = ring_base; j_ring_mid = ring_mid; j_ring_top = ring_top;
	j_little_base = little_base; j_little_mid = little_mid; j_little_top = little_top;
	global_pos = position;
	global_ori = orientation;
	global_rot = Vector3(0,0,0);
}

HandPose::HandPose(Vector3 thumb_base, Vector3 thumb_mid, Vector3 thumb_top, 
				   Vector3 index_base, Vector3 index_mid, Vector3 index_top, 
				   Vector3 middle_base, Vector3 middle_mid, Vector3 middle_top, 
				   Vector3 ring_base, Vector3 ring_mid, Vector3 ring_top, 
				   Vector3 little_base, Vector3 little_mid, Vector3 little_top,
				   Vector3 position, Vector3 orientation, Vector3 rotation)
{
	j_thumb_base = thumb_base; j_thumb_mid = thumb_mid; j_thumb_top = thumb_top;
	j_index_base = index_base; j_index_mid = index_mid; j_index_top = index_top;
	j_middle_base = middle_base; j_middle_mid = middle_mid; j_middle_top = middle_top;
	j_ring_base = ring_base; j_ring_mid = ring_mid; j_ring_top = ring_top;
	j_little_base = little_base; j_little_mid = little_mid; j_little_top = little_top;
	global_pos = position;
	global_ori = orientation;
	global_rot = rotation;
}

void HandPose::applyPose(HandModel& model)
{
	Array<Vector3> thumbarray;
	thumbarray.pushBack(j_thumb_base); thumbarray.pushBack(j_thumb_mid); thumbarray.pushBack(j_thumb_top);
	model.bentThumb(thumbarray);

	Array<Vector3> indexarray;
	indexarray.pushBack(j_index_base); indexarray.pushBack(j_index_mid); indexarray.pushBack(j_index_top);
	model.bentFinger(2, indexarray);

	Array<Vector3> middlearray;
	middlearray.pushBack(j_middle_base); middlearray.pushBack(j_middle_mid); middlearray.pushBack(j_middle_top);
	model.bentFinger(3, middlearray);

	Array<Vector3> ringarray;
	ringarray.pushBack(j_ring_base); ringarray.pushBack(j_ring_mid); ringarray.pushBack(j_ring_top);
	model.bentFinger(4, ringarray);

	Array<Vector3> littlearray;
	littlearray.pushBack(j_little_base); littlearray.pushBack(j_little_mid); littlearray.pushBack(j_little_top);
	model.bentFinger(5, littlearray);

	model.rotateHand(global_rot);
	if(global_ori[0] != 0 && global_ori[1] != 0 && global_ori[2] != 1)
		model.movetoGlobal(global_pos, global_ori);
	else
		model.moveHand(global_pos);
}

void HandPose::applyPose_f(HandModel& model)
{
	Array<Vector3> thumbarray;
	thumbarray.pushBack(j_thumb_base); thumbarray.pushBack(j_thumb_mid); thumbarray.pushBack(j_thumb_top);
	model.bentThumb(thumbarray);

	Array<Vector3> indexarray;
	indexarray.pushBack(j_index_base); indexarray.pushBack(j_index_mid); indexarray.pushBack(j_index_top);
	model.bentFinger(2, indexarray);

	Array<Vector3> middlearray;
	middlearray.pushBack(j_middle_base); middlearray.pushBack(j_middle_mid); middlearray.pushBack(j_middle_top);
	model.bentFinger(3, middlearray);

	Array<Vector3> ringarray;
	ringarray.pushBack(j_ring_base); ringarray.pushBack(j_ring_mid); ringarray.pushBack(j_ring_top);
	model.bentFinger(4, ringarray);

	Array<Vector3> littlearray;
	littlearray.pushBack(j_little_base); littlearray.pushBack(j_little_mid); littlearray.pushBack(j_little_top);
	model.bentFinger(5, littlearray);/**/

	//if(model.getGlobaltrans() != global_ori || model.getGlobalpos() != global_pos)
	//cout << model.moveHand(global_pos, global_ori) << endl;
	//model.rotateHand(global_rot);
	//model.moveHand(global_pos);
}

Array<Vector3> HandPose::getThumbPose()
{
	Array<Vector3> thumbarray;
	thumbarray.pushBack(j_thumb_base); thumbarray.pushBack(j_thumb_mid); thumbarray.pushBack(j_thumb_top);
	return thumbarray;
}

Array<Vector3> HandPose::getIndexPose()
{
	Array<Vector3> indexarray;
	indexarray.pushBack(j_index_base); indexarray.pushBack(j_index_mid); indexarray.pushBack(j_index_top);
	return indexarray;
}

Array<Vector3> HandPose::getMiddlePose()
{
	Array<Vector3> middlearray;
	middlearray.pushBack(j_middle_base); middlearray.pushBack(j_middle_mid); middlearray.pushBack(j_middle_top);
	return middlearray;
}

Array<Vector3> HandPose::getRingPose()
{
	Array<Vector3> ringarray;
	ringarray.pushBack(j_ring_base); ringarray.pushBack(j_ring_mid); ringarray.pushBack(j_ring_top);
	return ringarray;
}

Array<Vector3> HandPose::getLittlePose()
{
	Array<Vector3> littlearray;
	littlearray.pushBack(j_little_base); littlearray.pushBack(j_little_mid); littlearray.pushBack(j_little_top);
	return littlearray;

}

Array<float> HandPose::getAllParameters()
{
	Array<float> parameter;
	parameter.pushBack(j_thumb_base[1]);parameter.pushBack(j_thumb_base[2]);
	parameter.pushBack(j_thumb_mid[0]);
	parameter.pushBack(j_thumb_top[0]);
	parameter.pushBack(j_index_base[0]);parameter.pushBack(j_index_base[2]);
	parameter.pushBack(j_index_mid[0]);
	parameter.pushBack(j_index_top[0]);
	parameter.pushBack(j_middle_base[0]);parameter.pushBack(j_middle_base[2]);
	parameter.pushBack(j_middle_mid[0]);
	parameter.pushBack(j_middle_top[0]);
	parameter.pushBack(j_ring_base[0]);parameter.pushBack(j_ring_base[2]);
	parameter.pushBack(j_ring_mid[0]);
	parameter.pushBack(j_ring_top[0]);
	parameter.pushBack(j_little_base[0]);parameter.pushBack(j_little_base[2]);
	parameter.pushBack(j_little_mid[0]);
	parameter.pushBack(j_little_top[0]);
	parameter.pushBack(global_pos[0]);parameter.pushBack(global_pos[1]);parameter.pushBack(global_pos[2]);
	parameter.pushBack(global_rot[0]);parameter.pushBack(global_rot[1]);parameter.pushBack(global_rot[2]);
	return parameter;
}

void HandPose::setAllParameters(Array<float> newparameter)
{
	j_thumb_base[1] = validFingers(0, newparameter[0]); j_thumb_base[2] = validFingers(1, newparameter[1]);
	j_thumb_mid[0] =  validFingers(2, newparameter[2]);
	j_thumb_top[0] = validFingers(3, newparameter[3]);
	j_index_base[0] = validFingers(4, newparameter[4]); j_index_base[2] = validFingers(5, newparameter[5]);
	j_index_mid[0] = validFingers(6, newparameter[6]);
	j_index_top[0] = validFingers(7, newparameter[7]);
	j_middle_base[0] = validFingers(8, newparameter[8]); j_middle_base[2] = validFingers(9, newparameter[9]);
	j_middle_mid[0] = validFingers(10, newparameter[10]);
	j_middle_top[0] = validFingers(11, newparameter[11]);
	j_ring_base[0] = validFingers(12, newparameter[12]); j_ring_base[2] = validFingers(13, newparameter[13]);
	j_ring_mid[0] = validFingers(14, newparameter[14]);
	j_ring_top[0] = validFingers(15, newparameter[15]);
	j_little_base[0] = validFingers(16, newparameter[16]); j_little_base[2] = validFingers(17, newparameter[17]);
	j_little_mid[0] = validFingers(18, newparameter[18]);
	j_little_top[0] = validFingers(19, newparameter[19]);
	global_pos[0] = newparameter[20]; global_pos[1] = newparameter[21]; global_pos[2] = newparameter[22];
	global_rot[0] = newparameter[23]; global_rot[1] = newparameter[24]; global_rot[2] = newparameter[25];
}

void HandPose::setAllParameters(Array<float> newparameter, bool isvalid)
{
	j_thumb_base[1] = newparameter[0]; j_thumb_base[2] = newparameter[1];
	j_thumb_mid[0] =  newparameter[2];
	j_thumb_top[0] = newparameter[3];
	j_index_base[0] = newparameter[4]; j_index_base[2] = newparameter[5];
	j_index_mid[0] = newparameter[6];
	j_index_top[0] = newparameter[7];
	j_middle_base[0] = newparameter[8]; j_middle_base[2] = newparameter[9];
	j_middle_mid[0] = newparameter[10];
	j_middle_top[0] = newparameter[11];
	j_ring_base[0] = newparameter[12]; j_ring_base[2] = newparameter[13];
	j_ring_mid[0] = newparameter[14];
	j_ring_top[0] = newparameter[15];
	j_little_base[0] = newparameter[16]; j_little_base[2] = newparameter[17];
	j_little_mid[0] = newparameter[18];
	j_little_top[0] = newparameter[19];
	global_pos[0] = newparameter[20]; global_pos[1] = newparameter[21]; global_pos[2] = newparameter[22];
	global_rot[0] = newparameter[23]; global_rot[1] = newparameter[24]; global_rot[2] = newparameter[25];
}

void HandPose::setFingerPose(int f, Vector3 base, Vector3 mid, Vector3 top)
{
	switch(f)
	{
	case 0:
		j_thumb_base = base;
		j_thumb_mid = mid;
		j_thumb_top = top;
		break;
	case 1:
		j_index_base = base;
		j_index_mid = mid;
		j_index_top = top;
		break;
	case 2:
		j_middle_base = base;
		j_middle_mid = mid;
		j_middle_top = top;
		break;
	case 3:
		j_ring_base = base;
		j_ring_mid = mid;
		j_ring_top = top;
		break;
	case 4:
		j_little_base = base;
		j_little_mid = mid;
		j_little_top = top;
		break;
	default:
		cout << "Error in setFingerPose" << endl;
		break;
	}
}

void HandPose::setFingerParameter(int f, int j, float theta)
{
	switch(f)
	{
	case 0:
		if(j == 0)		j_thumb_base[1] = theta;
		else if(j == 1)	j_thumb_base[2] = theta;
		else if(j == 2)	j_thumb_mid[0] = theta;
		else if(j == 3)	j_thumb_top[0] = theta;
		else			cout << "Error in setFingerJointPose" << endl;
		break;
	case 1:
		if(j == 0)		j_index_base[0] = theta;
		else if(j == 1)	j_index_base[2] = theta;
		else if(j == 2)	j_index_mid[0] = theta;
		else if(j == 3)	j_index_top[0] = theta;
		else			cout << "Error in setFingerJointPose" << endl;
		break;
	case 2:
		if(j == 0)		j_middle_base[0] = theta;
		else if(j == 1)	j_middle_base[2] = theta;
		else if(j == 2)	j_middle_mid[0] = theta;
		else if(j == 3)	j_middle_top[0] = theta;
		else			cout << "Error in setFingerJointPose" << endl;
		break;
	case 3:
		if(j == 0)		j_ring_base[0] = theta;
		else if(j == 1)	j_ring_base[2] = theta;
		else if(j == 2)	j_ring_mid[0] = theta;
		else if(j == 3)	j_ring_top[0] = theta;
		else			cout << "Error in setFingerJointPose" << endl;
		break;
	case 4:
		if(j == 0)		j_little_base[0] = theta;
		else if(j == 1)	j_little_base[2] = theta;
		else if(j == 2)	j_little_mid[0] = theta;
		else if(j == 3)	j_little_top[0] = theta;
		else			cout << "Error in setFingerJointPose" << endl;
		break;
	default:
		cout << "Error in setFingerParameter" << endl;
		break;
	}

}

void HandPose::setGlobalParameter(int para, float theta)
{
	switch(para)
	{
	case 0:
		global_pos[0] = theta;
		break;
	case 1:
		global_pos[1] = theta;
		break;
	case 2:
		global_pos[2] = theta;
		break;
	case 3:
		global_rot[0] = theta;
		break;
	case 4:
		global_rot[1] = theta;
		break;
	case 5:
		global_rot[2] = theta;
		break;
	default:
		cout << "Error in setGlobalParameter" << endl;
		break;
	}
}

float HandPose::validFingers(int para, float value)
{
	// check limitation with only parameter
	float ubound[20] = {THUMB_MCP_FE_UPPERBOUND, THUMB_MCP_AA_UPPERBOUND, THUMB_PIP_FE_UPPERBOUND, THUMB_DIP_FE_UPPERBOUND, 
						INDEX_MCP_FE_UPPERBOUND, INDEX_MCP_AA_UPPERBOUND, INDEX_PIP_FE_UPPERBOUND, INDEX_DIP_FE_UPPERBOUND, 
						MIDDLE_MCP_FE_UPPERBOUND, MIDDLE_MCP_AA_UPPERBOUND, MIDDLE_PIP_FE_UPPERBOUND, MIDDLE_DIP_FE_UPPERBOUND, 
						RING_MCP_FE_UPPERBOUND, RING_MCP_AA_UPPERBOUND, RING_PIP_FE_UPPERBOUND, RING_DIP_FE_UPPERBOUND, 
						LITTLE_MCP_FE_UPPERBOUND, LITTLE_MCP_AA_UPPERBOUND, LITTLE_PIP_FE_UPPERBOUND, LITTLE_DIP_FE_UPPERBOUND};
	float lbound[20] = {THUMB_MCP_FE_LOWERBOUND, THUMB_MCP_AA_LOWERBOUND, THUMB_PIP_FE_LOWERBOUND, THUMB_DIP_FE_LOWERBOUND, 
						INDEX_MCP_FE_LOWERBOUND, INDEX_MCP_AA_LOWERBOUND, INDEX_PIP_FE_LOWERBOUND, INDEX_DIP_FE_LOWERBOUND, 
						MIDDLE_MCP_FE_LOWERBOUND, MIDDLE_MCP_AA_LOWERBOUND, MIDDLE_PIP_FE_LOWERBOUND, MIDDLE_DIP_FE_LOWERBOUND, 
						RING_MCP_FE_LOWERBOUND, RING_MCP_AA_LOWERBOUND, RING_PIP_FE_LOWERBOUND, RING_DIP_FE_LOWERBOUND, 
						LITTLE_MCP_FE_LOWERBOUND, LITTLE_MCP_AA_LOWERBOUND, LITTLE_PIP_FE_LOWERBOUND, LITTLE_DIP_FE_LOWERBOUND};
	if(ubound[para] < value)	return ubound[para];
	if(lbound[para] > value)	return lbound[para];
	return value;
}

float HandPose::validFingers(int finger, int para, float value)
{
	switch(finger)
	{
	case 1:
		switch(para)
		{
		case 0:
			if(value < INDEX_MCP_FE_LOWERBOUND)		return INDEX_MCP_FE_LOWERBOUND;
			if(value > INDEX_MCP_FE_UPPERBOUND)		return INDEX_MCP_FE_UPPERBOUND;
			return value;
		case 1:
			if(value < INDEX_MCP_AA_LOWERBOUND)		return INDEX_MCP_AA_LOWERBOUND;
			if(value > INDEX_MCP_AA_UPPERBOUND)		return INDEX_MCP_AA_UPPERBOUND;
			return value;
		case 2:
			if(value < INDEX_PIP_FE_LOWERBOUND)		return INDEX_PIP_FE_LOWERBOUND;
			if(value > INDEX_PIP_FE_UPPERBOUND)		return INDEX_PIP_FE_UPPERBOUND;
			return value;
		case 3:
			if(value < INDEX_DIP_FE_LOWERBOUND)		return INDEX_DIP_FE_LOWERBOUND;
			if(value > INDEX_DIP_FE_UPPERBOUND)		return INDEX_DIP_FE_UPPERBOUND;
			return value;
		default:
			return -2;
		}
	case 2:
		switch(para)
		{
		case 0:
			if(value < MIDDLE_MCP_FE_LOWERBOUND)	return MIDDLE_MCP_FE_LOWERBOUND;
			if(value > MIDDLE_MCP_FE_UPPERBOUND)	return MIDDLE_MCP_FE_UPPERBOUND;
			return value;
		case 1:
			if(value < MIDDLE_MCP_AA_LOWERBOUND)	return MIDDLE_MCP_AA_LOWERBOUND;
			if(value > MIDDLE_MCP_AA_UPPERBOUND)	return MIDDLE_MCP_AA_UPPERBOUND;
			return value;
		case 2:
			if(value < MIDDLE_PIP_FE_LOWERBOUND)	return MIDDLE_PIP_FE_LOWERBOUND;
			if(value > MIDDLE_PIP_FE_UPPERBOUND)	return MIDDLE_PIP_FE_UPPERBOUND;
			return value;
		case 3:
			if(value < MIDDLE_DIP_FE_LOWERBOUND)	return MIDDLE_DIP_FE_LOWERBOUND;
			if(value > MIDDLE_DIP_FE_UPPERBOUND)	return MIDDLE_DIP_FE_UPPERBOUND;
			return value;
		default:
			return -2;
		}
	case 3:
		switch(para)
		{
		case 0:
			if(value < RING_MCP_FE_LOWERBOUND)		return RING_MCP_FE_LOWERBOUND;
			if(value > RING_MCP_FE_UPPERBOUND)		return RING_MCP_FE_UPPERBOUND;
			return value;
		case 1:
			if(value < RING_MCP_AA_LOWERBOUND)		return RING_MCP_AA_LOWERBOUND;
			if(value > RING_MCP_AA_UPPERBOUND)		return RING_MCP_AA_UPPERBOUND;
			return value;
		case 2:
			if(value < RING_PIP_FE_LOWERBOUND)		return RING_PIP_FE_LOWERBOUND;
			if(value > RING_PIP_FE_UPPERBOUND)		return RING_PIP_FE_UPPERBOUND;
			return value;
		case 3:
			if(value < RING_DIP_FE_LOWERBOUND)		return RING_DIP_FE_LOWERBOUND;
			if(value > RING_DIP_FE_UPPERBOUND)		return RING_DIP_FE_UPPERBOUND;
			return value;
		default:
			return -2;
		}
	case 4:
		switch(para)
		{
		case 0:
			if(value < LITTLE_MCP_FE_LOWERBOUND)	return LITTLE_MCP_FE_LOWERBOUND;
			if(value > LITTLE_MCP_FE_UPPERBOUND)	return LITTLE_MCP_FE_UPPERBOUND;
			return value;
		case 1:
			if(value < LITTLE_MCP_AA_LOWERBOUND)	return LITTLE_MCP_AA_LOWERBOUND;
			if(value > LITTLE_MCP_AA_UPPERBOUND)	return LITTLE_MCP_AA_UPPERBOUND;
			return value;
		case 2:
			if(value < LITTLE_PIP_FE_LOWERBOUND)	return LITTLE_PIP_FE_LOWERBOUND;
			if(value > LITTLE_PIP_FE_UPPERBOUND)	return LITTLE_PIP_FE_UPPERBOUND;
			return value;
		case 3:
			if(value < LITTLE_DIP_FE_LOWERBOUND)	return LITTLE_DIP_FE_LOWERBOUND;
			if(value > LITTLE_DIP_FE_UPPERBOUND)	return LITTLE_DIP_FE_UPPERBOUND;
			return value;
		default:
			return -2;
		}
	case 0:
		switch(para)
		{
		case 0:
			if(value < THUMB_MCP_FE_LOWERBOUND)		return THUMB_MCP_FE_LOWERBOUND;
			if(value > THUMB_MCP_FE_UPPERBOUND)		return THUMB_MCP_FE_UPPERBOUND;
			return value;
		case 1:
			if(value < THUMB_MCP_AA_LOWERBOUND)		return THUMB_MCP_AA_LOWERBOUND;
			if(value > THUMB_MCP_AA_UPPERBOUND)		return THUMB_MCP_AA_UPPERBOUND;
			return value;
		case 2:
			if(value < THUMB_PIP_FE_LOWERBOUND)		return THUMB_PIP_FE_LOWERBOUND;
			if(value > THUMB_PIP_FE_UPPERBOUND)		return THUMB_PIP_FE_UPPERBOUND;
			return value;
		case 3:
			if(value < THUMB_DIP_FE_LOWERBOUND)		return THUMB_DIP_FE_LOWERBOUND;
			if(value > THUMB_DIP_FE_UPPERBOUND)		return THUMB_DIP_FE_UPPERBOUND;
			return value;
		default:
			return -2;
		}
	default:
		return -2;
	}
}

double HandPose::getBound(int i, bool ud)
{
	double ubound[20] = {THUMB_MCP_FE_UPPERBOUND, THUMB_MCP_AA_UPPERBOUND, THUMB_PIP_FE_UPPERBOUND, THUMB_DIP_FE_UPPERBOUND, 
						INDEX_MCP_FE_UPPERBOUND, INDEX_MCP_AA_UPPERBOUND, INDEX_PIP_FE_UPPERBOUND, INDEX_DIP_FE_UPPERBOUND, 
						MIDDLE_MCP_FE_UPPERBOUND, MIDDLE_MCP_AA_UPPERBOUND, MIDDLE_PIP_FE_UPPERBOUND, MIDDLE_DIP_FE_UPPERBOUND, 
						RING_MCP_FE_UPPERBOUND, RING_MCP_AA_UPPERBOUND, RING_PIP_FE_UPPERBOUND, RING_DIP_FE_UPPERBOUND, 
						LITTLE_MCP_FE_UPPERBOUND, LITTLE_MCP_AA_UPPERBOUND, LITTLE_PIP_FE_UPPERBOUND, LITTLE_DIP_FE_UPPERBOUND};
	double lbound[20] = {THUMB_MCP_FE_LOWERBOUND, THUMB_MCP_AA_LOWERBOUND, THUMB_PIP_FE_LOWERBOUND, THUMB_DIP_FE_LOWERBOUND, 
						INDEX_MCP_FE_LOWERBOUND, INDEX_MCP_AA_LOWERBOUND, INDEX_PIP_FE_LOWERBOUND, INDEX_DIP_FE_LOWERBOUND, 
						MIDDLE_MCP_FE_LOWERBOUND, MIDDLE_MCP_AA_LOWERBOUND, MIDDLE_PIP_FE_LOWERBOUND, MIDDLE_DIP_FE_LOWERBOUND, 
						RING_MCP_FE_LOWERBOUND, RING_MCP_AA_LOWERBOUND, RING_PIP_FE_LOWERBOUND, RING_DIP_FE_LOWERBOUND, 
						LITTLE_MCP_FE_LOWERBOUND, LITTLE_MCP_AA_LOWERBOUND, LITTLE_PIP_FE_LOWERBOUND, LITTLE_DIP_FE_LOWERBOUND};
	
	if(ud)	return ubound[i];
	else	return lbound[i];

}