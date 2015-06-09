#include <pcl/common/transforms.h>
#include "HandModel.hxx"
#include "MyTools.hxx"

using namespace pcl;
using namespace Eigen;

HandModel::HandModel()
{
	// build sphere plam
	models.pushBack(Sphere());													relatedmap[0] = -1;
	models.pushBack(Sphere(Vector3(20 - 30,0 - 28,0), 12, &models[0], "plam2-1"));		relatedmap[1] = 0;			// 30,28,0
	models.pushBack(Sphere(Vector3(40 - 30,0 - 28,0), 12, &models[1], "plam3-1"));		relatedmap[2] = 1;
	models.pushBack(Sphere(Vector3(60 - 30,0 - 28,0), 12, &models[2], "plam4-1"));		relatedmap[3] = 2;
	models.pushBack(Sphere(Vector3(0 - 30,15 - 28,0), 12, &models[0], "plam1-2"));		relatedmap[4] = 0;
	models.pushBack(Sphere(Vector3(20 - 30,18 - 28,0), 12, &models[4], "plam2-2"));		relatedmap[5] = 4;
	models.pushBack(Sphere(Vector3(40 - 30,18 - 28,0), 12, &models[5], "plam3-2"));		relatedmap[6] = 5;
	models.pushBack(Sphere(Vector3(60 - 30,10 - 28,0), 12, &models[6], "plam4-2"));		relatedmap[7] = 6;
	models.pushBack(Sphere(Vector3(0 - 30,35 - 28,0), 12, &models[4], "plam1-3"));		relatedmap[8] = 4;
	models.pushBack(Sphere(Vector3(20 - 30,38 - 28,0), 12, &models[8], "plam2-3"));		relatedmap[9] = 8;
	models.pushBack(Sphere(Vector3(40 - 30,38 - 28,0), 12, &models[9], "plam3-3"));		relatedmap[10] = 9;
	models.pushBack(Sphere(Vector3(60 - 30,30 - 28,0), 12, &models[10], "plam4-3"));		relatedmap[11] = 10;
	models.pushBack(Sphere(Vector3(0 - 30,55 - 28,0), 12, &models[8], "plam1-4"));		relatedmap[12] = 8;
	models.pushBack(Sphere(Vector3(20 - 30,58 - 28,0), 12, &models[12], "plam2-4"));		relatedmap[13] = 12;
	models.pushBack(Sphere(Vector3(40 - 30,58 - 28,0), 12, &models[13], "plam3-4"));		relatedmap[14] = 13;
	models.pushBack(Sphere(Vector3(60 - 30,50 - 28,0), 12, &models[14], "plam4-4"));		relatedmap[15] = 14;

	// build sphere index (assume plam 1-4 is root)
	models.pushBack(Sphere(Vector3(-4 - 30,70 - 28,0), 11, &models[12], "index_b1"));		relatedmap[16] = 12;
	models.pushBack(Sphere(Vector3(-4 - 30,85 - 28,0), 11, &models[16], "index_b2"));		relatedmap[17] = 16;
	models.pushBack(Sphere(Vector3(-4 - 30,100 - 28,0), 10, &models[17], "index_m1"));		relatedmap[18] = 17;
	models.pushBack(Sphere(Vector3(-4 - 30,115 - 28,0), 10, &models[18], "index_m2"));	relatedmap[19] = 18;
	models.pushBack(Sphere(Vector3(-4 - 30,130 - 28,0), 10, &models[19], "index_t1"));	relatedmap[20] = 19;
	models.pushBack(Sphere(Vector3(-4 - 30,140 - 28,0), 10, &models[20], "index_t2", Vector3(255,0,0)));	relatedmap[21] = 20;

	// build sphere middle (assume plam 2-4 is root)
	models.pushBack(Sphere(Vector3(18 - 30,77 - 28,0), 11, &models[13], "middle_b1"));	relatedmap[22] = 13;
	models.pushBack(Sphere(Vector3(18 - 30,92 - 28,0), 11, &models[22], "middle_b2"));	relatedmap[23] = 22;
	models.pushBack(Sphere(Vector3(18 - 30,107 - 28,0), 10, &models[23], "middle_m1"));	relatedmap[24] = 23;
	models.pushBack(Sphere(Vector3(18 - 30,122 - 28,0), 10, &models[24], "middle_m2"));	relatedmap[25] = 24;
	models.pushBack(Sphere(Vector3(18 - 30,137 - 28,0), 10, &models[25], "middle_t1"));	relatedmap[26] = 25;
	models.pushBack(Sphere(Vector3(18 - 30,147 - 28,0), 10, &models[26], "middle_t2", Vector3(255,0,0)));	relatedmap[27] = 26;

	// build sphere ring (assume plam 3-4 is root)
	models.pushBack(Sphere(Vector3(42 - 30,70 - 28,0), 11, &models[14], "ring_b1"));		relatedmap[28] = 14;
	models.pushBack(Sphere(Vector3(42 - 30,85 - 28,0), 11, &models[29], "ring_b2"));		relatedmap[29] = 28;
	models.pushBack(Sphere(Vector3(42 - 30,100 - 28,0), 10, &models[30], "ring_m1"));		relatedmap[30] = 29;
	models.pushBack(Sphere(Vector3(42 - 30,115 - 28,0), 10, &models[31], "ring_m2"));		relatedmap[31] = 30;
	models.pushBack(Sphere(Vector3(42 - 30,130 - 28,0), 10, &models[32], "ring_t1"));		relatedmap[32] = 31;
	models.pushBack(Sphere(Vector3(42 - 30,140 - 28,0), 10, &models[33], "ring_t2", Vector3(255,0,0)));		relatedmap[33] = 32;

	// build sphere little (assume plam 4-4 is root)
	models.pushBack(Sphere(Vector3(64 - 30,65 - 28,0), 11, &models[15], "little_b1"));	relatedmap[34] = 15;
	models.pushBack(Sphere(Vector3(64 - 30,72.5 - 28,0), 11, &models[34], "little_b2"));	relatedmap[35] = 34;
	models.pushBack(Sphere(Vector3(64 - 30,85 - 28,0), 10, &models[35], "little_m1"));	relatedmap[36] = 35;
	models.pushBack(Sphere(Vector3(64 - 30,95 - 28,0), 10, &models[36], "little_m2"));	relatedmap[37] = 36;
	models.pushBack(Sphere(Vector3(64 - 30,105 - 28,0), 10, &models[37], "little_t1"));	relatedmap[38] = 37;
	models.pushBack(Sphere(Vector3(64 - 30,115 - 28,0), 10, &models[38], "little_t2", Vector3(255,0,0)));	relatedmap[39] = 38;

	// build sphere thumb (assume plam 1-1 is root)
	models.pushBack(Sphere(Vector3(10 - 30,5 - 28,0), 16, &models[0], "thumb_b1"));		relatedmap[40] = 0;// 10 10
	models.pushBack(Sphere(Vector3(0 - 30,15 - 28,0), 16, &models[40], "thumb_b2"));	relatedmap[41] = 40;// 10 10
	models.pushBack(Sphere(Vector3(-10 - 30,25 - 28,0), 15, &models[41], "thumb_b3"));	relatedmap[42] = 41;// 10 10
	models.pushBack(Sphere(Vector3(-18 - 30,35 - 28,0), 15, &models[42], "thumb_b4"));	relatedmap[43] = 42;// 8 10
	models.pushBack(Sphere(Vector3(-26 - 30,45 - 28,0), 12, &models[43], "thumb_m1"));	relatedmap[44] = 43;// 8 10
	models.pushBack(Sphere(Vector3(-34 - 30,55 - 28,0), 11, &models[44], "thumb_m2"));	relatedmap[45] = 44;// 8 10
	models.pushBack(Sphere(Vector3(-42 - 30,65 - 28,0), 12, &models[45], "thumb_t1"));	relatedmap[46] = 45;// 8 10
	models.pushBack(Sphere(Vector3(-50 - 30,75 - 28,0), 10, &models[46], "thumb_t2", Vector3(255,0,0)));	relatedmap[47] = 46;/**/

	globalup = Vector3(0,1,0);
	globalori = Vector3(0,0,1);
	globalpos = Vector3(0,0,0);
}

Matrix4 HandModel::movetoGlobal(Vector3 pos, Vector3 ori)
{
	Vector3 origin = Vector3(0,0,1);
	if(globalpos == pos && origin == ori)
		return  Matrix4::IDENTITY;
	ori = ori.normalizedCopy();
	Vector3 normal = ori.cross(origin);
	float len = normal.length();
	float d = origin.dot(ori);
	Matrix3 skewv = Matrix3(0, -normal[2], normal[1], normal[2], 0, -normal[0], -normal[1], normal[0], 0);
	Matrix3 rot = Matrix3(1,0,0,0,1,0,0,0,1) + skewv + ((1 - d) / len) * skewv * skewv;
	Matrix4 trans = MyTools::translation(pos - globalpos) * MyTools::matrix3to4(rot);
	
	// Rotate all sphere
	for(size_t i = 0; i < models.size(); i++)
	{
		Vector3 newpos = trans.multiplyByPoint(models[i].getCenter());
		models[i].setCenter(newpos);
	}
	// Recompute ori, up and position
	globalup = trans.multiplyByVector(globalup);
	globalup = globalup.normalizedCopy();
	globalori = trans.multiplyByVector(globalori);
	globalori = globalori.normalizedCopy();
	globalpos = trans.multiplyByPoint(globalpos);
	return trans;
}

Matrix4 HandModel::moveHand(Vector3 pos, Vector3 ori)
{
	if(globalpos == pos && globalori == ori)
		return  Matrix4::IDENTITY;
	ori = ori.normalizedCopy();
	Vector3 normal = ori.cross(globalori);
	float len = normal.length();
	float d = globalori.dot(ori);
	Matrix3 skewv = Matrix3(0, -normal[2], normal[1], normal[2], 0, -normal[0], -normal[1], normal[0], 0);
	Matrix3 rot = Matrix3(1,0,0,0,1,0,0,0,1) + skewv + ((1 - d) / len) * skewv * skewv;
	Matrix4 trans = MyTools::translation(pos - globalpos) * MyTools::matrix3to4(rot);
	
	// Rotate all sphere
	for(size_t i = 0; i < models.size(); i++)
	{
		Vector3 newpos = trans.multiplyByPoint(models[i].getCenter());
		models[i].setCenter(newpos);
	}
	// Recompute ori, up and position
	//globalup = rot * globalup;
	globalup = trans.multiplyByVector(globalup);
	globalup = globalup.normalizedCopy();
	//globalori = rot * globalori;
	globalori = trans.multiplyByVector(globalori);
	globalori = globalori.normalizedCopy();
	globalpos = trans.multiplyByPoint(globalpos);
	return trans;
}

Matrix4 HandModel::moveHand(Vector3 pos)
{
	Matrix4 trans = MyTools::translation(pos - globalpos);

	// Rotate all sphere
	for(size_t i = 0; i < models.size(); i++)
	{
		Vector3 newpos = trans.multiplyByPoint(models[i].getCenter());
		models[i].setCenter(newpos);
	}
	// Recompute position only
	globalpos = trans.multiplyByPoint(globalpos);
	return trans;
}

void HandModel::rotateHand(Vector3 angle)
{
	// rotate by the normal, and assume that the normal is z axis
	Matrix4 rot = Matrix4::IDENTITY;
	rot.fromRotationVector(angle);
	for(size_t i = 0; i < models.size(); i++)
	{
		Vector3 newpos = rot.multiplyByPoint(models[i].getCenter());
		models[i].setCenter(newpos);
	}
	// change the up vector
	globalup = rot.multiplyByVector(globalup);
	globalup = globalup.normalizedCopy();
	globalori = rot.multiplyByVector(globalori);
	globalori = globalori.normalizedCopy();
}

void HandModel::WorldtoObject()
{
	// Useless now!!!

	// back to origin
	//moveHand(Vector3(0, 0, 0), Vector3(0, 0, 1));
	// take y axis as up position
	//Vector3 orirot = Vector3(0, 1, 0);
	//if(globalup == orirot)	return;

}

void HandModel::ObjecttoWorld(Vector3 pos, Vector3 ori, Vector3 rot)
{
	// Useless now!!!

}

void HandModel::bentFinger(int fin, SK::Array<Vector3> angle)
{
	// index spheres: 16~21, 16: first joint, 18: mid joint, 20: last joint, 21:tips
	// middle spheres: 22~27, 22: first joint, 24: mid joint, 26: last joint, 27:tips
	// ring spheres: 28~33, 28: first joint, 30: mid joint, 32: last joint, 33:tips
	// little spheres: 34~39, 34: first joint, 36: mid joint, 38: last joint, 39:tips
	int rootarray[3], tips;
	switch(fin)
	{
	case 2:
		rootarray[0] = 16;rootarray[1] = 18;rootarray[2] = 20;tips = 21;
		break;
	case 3:
		rootarray[0] = 22;rootarray[1] = 24;rootarray[2] = 26;tips = 27;
		break;
	case 4:
		rootarray[0] = 28;rootarray[1] = 30;rootarray[2] = 32;tips = 33;
		break;
	case 5:
		rootarray[0] = 34;rootarray[1] = 36;rootarray[2] = 38;tips = 39;
		break;
	}
	// Set last joint as root, rotate all spheres
	// And then set mid joint as root, rotate spheres
	// Finally, set the first joint as root, rotate all spheres (all movements have to move back)	
	for(int t = 2; t >= 0; t--)
	{
		Vector3 tmproot = models[rootarray[t]].getCenter();
		Matrix4 pretrans = MyTools::translation(-tmproot);
		Matrix4 rot = Matrix4::IDENTITY;
		rot.fromRotationVector(angle[t]);
		Matrix4 retrans = MyTools::translation(tmproot);
		for(int i = rootarray[t]; i <= tips; i++)
		{
			Matrix4 tmptrans = retrans * rot * pretrans;
			Vector3 newpos = tmptrans.multiplyByPoint(models[i].getCenter());
			models[i].setCenter(newpos);
		}
	}

}

void HandModel::bentThumb(SK::Array<Vector3> angle)
{
	// the main axis = (2,1,-2)
	// Quaternion form:, given axis A(Ax,Ay,Az) and theta, s = sin(theta/2), w = cos(theta/2), x = s*Ax, y = s*Ay, z = s*Az
	// angle form: (below), (theta1, 0, 0), (theta2, 0, 0)
	// thumb spheres: 40~47, 40: first joint, 44: mid joint, 47: last joint, 47:tips
	float theta[3] = {0, angle[1][0], angle[2][0]};
	int rootarray[3] = {40, 44, 46}, tips = 47;
	for(int t = 2; t >= 1; t--)		// 1!!!!
	{
		// Quaternion building
		float s = sin(theta[t] / 2.0), w = cos(theta[t] / 2.0);
		float x =  s * 2 / 3, y = s / 3, z =  s * 2 / -3;
		SK::Quaternion quat = SK::Quaternion(w, x, y, z);
		Matrix4 rot = quat.toMatrix4();

		Vector3 tmproot = models[rootarray[t]].getCenter();
		Matrix4 pretrans = MyTools::translation(-tmproot);
		Matrix4 retrans = MyTools::translation(tmproot);
		for(int i = rootarray[t]; i <= tips; i++)
		{
			Matrix4 tmptrans = retrans * rot * pretrans;
			Vector3 newpos = tmptrans.multiplyByPoint(models[i].getCenter());
			models[i].setCenter(newpos);
		}
	}
	//the first joint rotatation axis: y and z (0, phi1, phi2)
	Vector3 tmproot_st = models[rootarray[0]].getCenter();
	Matrix4 pretrans_st = MyTools::translation(-tmproot_st);
	Matrix4 rot_st = Matrix4::IDENTITY;
	rot_st.fromRotationVector(angle[0]);
	Matrix4 retrans_st = MyTools::translation(tmproot_st);
	for(int i = rootarray[0]; i <= tips; i++)
	{
		Matrix4 tmptrans = retrans_st * rot_st * pretrans_st;
		Vector3 newpos = tmptrans.multiplyByPoint(models[i].getCenter());
		models[i].setCenter(newpos);
	}
}

SK::Array<int> HandModel::getAllFingersIndex()
{
	SK::Array<int> fingerarray;
	for(int i = 16; i < 48; i++)
		fingerarray.pushBack(i);
	return fingerarray;
}

SK::Array<Sphere> HandModel::getNeighbors(int index)
{
	// index follow the hand model
	SK::Array<Sphere> cand;
	// thumb condition: consider index
	if(index >= 40 && index < 48)
	{
		for(int i = 16; i < 22; i++)
			cand.pushBack(models[i]);
	}

	// index condition: consider thumb, middle
	else if(index >= 16 && index < 22)
	{
		for(int i = 40; i < 48; i++)
			cand.pushBack(models[i]);
		for(int i = 22; i < 28; i++)
			cand.pushBack(models[i]);
	}

	// middle condition: consider index, ring
	else if(index >= 22 && index < 28)
	{
		for(int i = 16; i < 22; i++)
			cand.pushBack(models[i]);
		for(int i = 28; i < 34; i++)
			cand.pushBack(models[i]);
	}

	// ring condition: consider middle, little
	else if(index >= 28 && index < 34)
	{
		for(int i = 22; i < 28; i++)
			cand.pushBack(models[i]);
		for(int i = 34; i < 40; i++)
			cand.pushBack(models[i]);
	}

	// little condition: consider ring
	else if(index >= 34 && index < 40)
	{
		for(int i = 28; i < 34; i++)
			cand.pushBack(models[i]);
	}

	return cand;
}

Vector3 HandModel::getFingerTips(int index)
{
	// 0: rhumb, 1: index, 2: middle, 3: ring, 4: little
	switch(index)
	{
	case 0:
		return models[47].getCenter();
	case 1:
		return models[21].getCenter();
	case 2:
		return models[27].getCenter();
	case 3:
		return models[33].getCenter();
	case 4:
		return models[39].getCenter();
	default:
		cout << "Error occur in getFingerTips" << endl;
		return Vector3();
	}
}

Vector3 HandModel::getFingerBaseJoint(int index)
{
	// 0: rhumb, 1: index, 2: middle, 3: ring, 4: little
	switch(index)
	{
	case 0:
		return models[40].getCenter();
	case 1:
		return models[16].getCenter();
	case 2:
		return models[22].getCenter();
	case 3:
		return models[28].getCenter();
	case 4:
		return models[34].getCenter();
	default:
		cout << "Error occur in getFingerBaseJoint" << endl;
		return Vector3();
	}
}

Vector3 HandModel::getFingerMiddleJoint(int index)
{
	// 0: rhumb, 1: index, 2: middle, 3: ring, 4: little
	switch(index)
	{
	case 0:
		return models[44].getCenter();
	case 1:
		return models[18].getCenter();
	case 2:
		return models[24].getCenter();
	case 3:
		return models[30].getCenter();
	case 4:
		return models[36].getCenter();
	default:
		cout << "Error occur in getFingerMiddleJoint" << endl;
		return Vector3();
	}
}

Vector3 HandModel::getFingerTopJoint(int index)
{
	// 0: rhumb, 1: index, 2: middle, 3: ring, 4: little
	switch(index)
	{
	case 0:
		return models[46].getCenter();
	case 1:
		return models[20].getCenter();
	case 2:
		return models[26].getCenter();
	case 3:
		return models[32].getCenter();
	case 4:
		return models[38].getCenter();
	default:
		cout << "Error occur in getFingerTopJoint" << endl;
		return Vector3();
	}
}

SK::Array<ModelCoefficients> HandModel::getSkeleton()
{
	SK::Array<ModelCoefficients> skeleton;
	
	// finger skeleton
	int finger_index[5][4] = {{40, 44, 46, 47}, {16, 18, 20, 21}, {22, 24, 26, 27}, {28, 30, 32, 33}, {34, 36, 38, 39}};
	for(int f = 0; f < 5; f++)
	{
		for(int n = 0; n < 3; n++)
		{
			ModelCoefficients finger_coeff;
			finger_coeff.values.resize(7);
			finger_coeff.values[0] = models[finger_index[f][n]].getCenter()[0];
			finger_coeff.values[1] = models[finger_index[f][n]].getCenter()[1];
			finger_coeff.values[2] = models[finger_index[f][n]].getCenter()[2];
			finger_coeff.values[3] = models[finger_index[f][n + 1]].getCenter()[0] - models[finger_index[f][n]].getCenter()[0];
			finger_coeff.values[4] = models[finger_index[f][n + 1]].getCenter()[1] - models[finger_index[f][n]].getCenter()[1];
			finger_coeff.values[5] = models[finger_index[f][n + 1]].getCenter()[2] - models[finger_index[f][n]].getCenter()[2];
			finger_coeff.values[6] = 1;
			skeleton.pushBack(finger_coeff);
		}
	}
	
	// palm skeleton
	Vector3 center = Vector3();
	center[0] = (models[5].getCenter()[0] + models[6].getCenter()[0] + models[9].getCenter()[0] + models[10].getCenter()[0]) / 4.0;
	center[1] = (models[5].getCenter()[1] + models[6].getCenter()[1] + models[9].getCenter()[1] + models[10].getCenter()[1]) / 4.0;
	center[2] = (models[5].getCenter()[2] + models[6].getCenter()[2] + models[9].getCenter()[2] + models[10].getCenter()[2]) / 4.0;
	int palm_index[5] = {40, 16, 22, 28, 34};
	for(int n = 0; n < 5; n++)
	{
		ModelCoefficients palm_coeff;
		palm_coeff.values.resize(7);
		palm_coeff.values[0] = center[0];
		palm_coeff.values[1] = center[1];
		palm_coeff.values[2] = center[2];
		palm_coeff.values[3] = models[palm_index[n]].getCenter()[0] - center[0];
		palm_coeff.values[4] = models[palm_index[n]].getCenter()[1] - center[1];
		palm_coeff.values[5] = models[palm_index[n]].getCenter()[2] - center[2];
		palm_coeff.values[6] = 1;
		skeleton.pushBack(palm_coeff);
	}

	return skeleton;
}

/*template <typename T>
SK::Array<T> HandModel::getQuaternionfromVec(Eigen::Matrix<T, 3, 1> &vec)
{
	T sqrAngle = vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2];
	T angle = sqrt(sqrAngle);
	SK::Array<T> para;
	
	if (abs(angle) < T(0.00001))
	{
		para.pushBack(T(1.0));
		para.pushBack(T(0.0));
		para.pushBack(T(0.0));
		para.pushBack(T(0.0));
	}
	else
	{
		T halfAngle = T(0.5) * angle;
		T sinHalfAngle = sin(halfAngle);

		para.pushBack(cos(halfAngle));
		para.pushBack(sinHalfAngle * vec[0] / angle);
		para.pushBack(sinHalfAngle * vec[1] / angle);
		para.pushBack(sinHalfAngle * vec[2] / angle);
	}

	return para;
}*/

/*template <typename T>
SK::Array<Eigen::Matrix<T, 3, 1>> HandModel::transformT(SK::Array<T> &para)
{
	// return 48 new point positions
	SK::Array<Eigen::Matrix<T, 3, 1>> t_point_list;
	t_point_list.resize(SPHERE_NUM);
	Eigen::Transform global_trans = Eigen::Translation<T, 3>(para[20], para[21], para[22]);
	SK::Array<T> q_para = getQuaternionfromVec(Eigen::Matrix<T, 3, 1>(para[23], para[24], para[25]));
	Eigen::Transform global_rot = Eigen::Quaternion<T, 3>(q_para[0], q_para[1], q_para[2], q_para[3]);

	// finger without thumb
	for(int fin = 0; fin < 4; fin++)
	{
		// index spheres: 16~21, 16: first joint, 18: mid joint, 20: last joint, 21:tips
		// middle spheres: 22~27, 22: first joint, 24: mid joint, 26: last joint, 27:tips
		// ring spheres: 28~33, 28: first joint, 30: mid joint, 32: last joint, 33:tips
		// little spheres: 34~39, 34: first joint, 36: mid joint, 38: last joint, 39:tips
		int rootarray[3], tips;
		SK::Array<Eigen::Matrix<T, 3, 1>> angle;
		switch(fin)
		{
		case 0:	// index
			rootarray[0] = 16;rootarray[1] = 18;rootarray[2] = 20;tips = 21;
			angle.pushBack(Eigen::Matrix<T, 3, 1>(para[4], T(0), para[5]));
			angle.pushBack(Eigen::Matrix<T, 3, 1>(para[6], T(0), T(0)));
			angle.pushBack(Eigen::Matrix<T, 3, 1>(para[7], T(0), T(0)));
			break;
		case 1:	// middle
			rootarray[0] = 22;rootarray[1] = 24;rootarray[2] = 26;tips = 27;
			angle.pushBack(Eigen::Matrix<T, 3, 1>(para[8], T(0), para[9]));
			angle.pushBack(Eigen::Matrix<T, 3, 1>(para[10], T(0), T(0)));
			angle.pushBack(Eigen::Matrix<T, 3, 1>(para[11], T(0), T(0)));
			break;
		case 2:	// ring
			rootarray[0] = 28;rootarray[1] = 30;rootarray[2] = 32;tips = 33;
			angle.pushBack(Eigen::Matrix<T, 3, 1>(para[12], T(0), para[13]));
			angle.pushBack(Eigen::Matrix<T, 3, 1>(para[14], T(0), T(0)));
			angle.pushBack(Eigen::Matrix<T, 3, 1>(para[15], T(0), T(0)));
			break;
		case 3:	// little
			rootarray[0] = 34;rootarray[1] = 36;rootarray[2] = 38;tips = 39;
			angle.pushBack(Eigen::Matrix<T, 3, 1>(para[16], T(0), para[17]));
			angle.pushBack(Eigen::Matrix<T, 3, 1>(para[18], T(0), T(0)));
			angle.pushBack(Eigen::Matrix<T, 3, 1>(para[19], T(0), T(0)));
			break;
		}

		// initial spheres
		for(int i = 0; i < 6; i++)	// basic?
			t_point_list[16 + fin * 6 + i] = Eigen::Matrix<T, 3, 1>(T(models[16 + fin * 6 + i].getCenter()[0]), 
																	T(models[16 + fin * 6 + i].getCenter()[1]), 
																	T(models[16 + fin * 6 + i].getCenter()[2]));

		// Set last joint as root, rotate all spheres
		// And then set mid joint as root, rotate spheres
		// Finally, set the first joint as root, rotate all spheres (all movements have to move back)	
		for(int t = 2; t >= 0; t--)
		{
			// T and -T
			Eigen::Matrix<T, 3, 1> tmp_root(T(models[rootarray[t]].getCenter()[0]), T(models[rootarray[t]].getCenter()[1]), T(models[rootarray[t]].getCenter()[2])); 
			Eigen::Transform pre_trans = Eigen::Translation<T, 3>(-tmp_root);
			Eigen::Transform re_trans = Eigen::Translation<T, 3>(tmp_root);
			// R
			SK::Array<T> q_para_fin = getQuaternionfromVec(angle[t]);
			Eigen::Transform rot = Eigen::Quaternion<T, 3>(q_para_fin[0], q_para_fin[1], q_para_fin[2], q_para_fin[3]);
			
			// transform
			for(int i = rootarray[t]; i <= tips; i++)
				t_point_list[i] = re_trans * rot * pre_trans * t_point_list[i];
		}
	}

	// thumb
	{
		// the main axis = (2,1,-2)
		// Quaternion form:, given axis A(Ax,Ay,Az) and theta, s = sin(theta/2), w = cos(theta/2), x = s*Ax, y = s*Ay, z = s*Az
		// angle form: (below), (theta1, 0, 0), (theta2, 0, 0)
		// thumb spheres: 40~47, 40: first joint, 44: mid joint, 47: last joint, 47:tips
		T theta[3] = {T(0), T(para[2]), T(para[3])};
		int rootarray[3] = {40, 44, 46}, tips = 47;
		for(int t = 2; t >= 1; t--)		// 1!!!!
		{
			// T and -T
			Eigen::Matrix<T, 3, 1> tmp_root(T(models[rootarray[t]].getCenter()[0]), T(models[rootarray[t]].getCenter()[1]), T(models[rootarray[t]].getCenter()[2])); 
			Eigen::Transform pre_trans = Eigen::Translation<T, 3>(-tmp_root);
			Eigen::Transform re_trans = Eigen::Translation<T, 3>(tmp_root);
			// R (Quaternion building)
			T s = sin(theta[t] / T(2.0)), w = cos(theta[t] / T(2.0));
			T x =  s * T(2) / T(3), y = s / T(3), z =  s * T(2) / T(-3);
			Eigen::Transform rot = Eigen::Quaternion<T, 3>(w, x, y, z);
			
			// transform
			for(int i = rootarray[t]; i <= tips; i++)
				t_point_list[i] = re_trans * rot * pre_trans * t_point_list[i];
		}
		
		//the first joint rotatation axis: y and z (0, phi1, phi2)
		// T and -T
		Eigen::Matrix<T, 3, 1> tmp_root_st(T(models[rootarray[0]].getCenter()[0]), T(models[rootarray[0]].getCenter()[1]), T(models[rootarray[0]].getCenter()[2])); 
		Eigen::Transform pre_trans_st = Eigen::Translation<T, 3>(-tmp_root_st);
		Eigen::Transform re_trans_st = Eigen::Translation<T, 3>(tmp_root_st);
		// R
		Eigen::Matrix<T, 3, 1> root_angle((T(0), para[0], para[1]));
		SK::Array<T> q_para_fin = getQuaternionfromVec(root_angle);
		Eigen::Transform rot_st = Eigen::Quaternion<T, 3>(q_para_fin[0], q_para_fin[1], q_para_fin[2], q_para_fin[3]);

		// transform
		for(int i = rootarray[0]; i <= tips; i++)
			t_point_list[i] = retrans_st * rot_st * pretrans_st * t_point_list[i];
	}

	// palm and global transform
	for(int i = 0; i < SPHERE_NUM; i++)
	{
		// rotation first and the move
		if(i < 16)
			t_point_list[i] = Eigen::Matrix<T, 3, 1>(T(models[i].getCenter()[0]), T(models[i].getCenter()[1]), T(models[i].getCenter()[2])); 
		t_point_list[i] = global_trans * global_rot * t_point_list[i];
	}

	return t_point_list;
}*/
