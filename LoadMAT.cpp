#include <pcl/visualization/cloud_viewer.h>
#include <EasiiSDK/Iisu.h>
#include <iostream>
#include <fstream>
#include "mat.h"
#include "MyTools.hxx"
#include "HandPose.hxx"
#include "HandModel.hxx"
#include "DataDriven.hxx"

using namespace std;
using namespace SK;
using namespace SK::Easii;

// for testing
const bool skel = false;

boost::shared_ptr<visualization::PCLVisualizer> viewer (new visualization::PCLVisualizer ("3D Viewer"));

void addHandModel(HandModel &handmodel, bool isdot)
{
	SK::Array<Sphere> sgroup = handmodel.getFullHand();
	if(isdot)
	{
		int jointspot[] = {16, 18, 20, 21, 22, 24, 26, 27, 28, 30, 32, 33, 34, 36, 38, 39, 40, 44, 46, 47};
		for(size_t i = 0; i < 20; i++)
			viewer->addSphere(MyTools::vectortoPointXYZ(sgroup[jointspot[i]].getCenter()), 3, 
							  sgroup[jointspot[i]].getColor()[0], sgroup[jointspot[i]].getColor()[1], sgroup[jointspot[i]].getColor()[2], 
							  sgroup[jointspot[i]].getName());
	}
	else
		for(size_t i = 0; i < sgroup.size(); i++)
			viewer->addSphere(MyTools::vectortoPointXYZ(sgroup[i].getCenter()), sgroup[i].getRadius(), sgroup[i].getColor()[0], sgroup[i].getColor()[1], sgroup[i].getColor()[2], sgroup[i].getName());
}

void updateHandModel(HandModel &handmodel, bool isdot)
{
	SK::Array<Sphere> sgroup = handmodel.getFullHand();
	if(isdot)
	{
		int jointspot[] = {16, 18, 20, 21, 22, 24, 26, 27, 28, 30, 32, 33, 34, 36, 38, 39, 40, 44, 46, 47};
		for(size_t i = 0; i < 20; i++)
			viewer->updateSphere(MyTools::vectortoPointXYZ(sgroup[jointspot[i]].getCenter()), 3, 
								 sgroup[jointspot[i]].getColor()[0], sgroup[jointspot[i]].getColor()[1], sgroup[jointspot[i]].getColor()[2], 
								 sgroup[jointspot[i]].getName());
	}
	else
		for(size_t i = 0; i < sgroup.size(); i++)
			viewer->updateSphere(MyTools::vectortoPointXYZ(sgroup[i].getCenter()), sgroup[i].getRadius(), sgroup[i].getColor()[0], sgroup[i].getColor()[1], sgroup[i].getColor()[2], sgroup[i].getName());

}

void addHandSkeleton(HandModel &handmodel)
{
	SK::Array<ModelCoefficients> coeff = handmodel.getSkeleton();
	for(size_t i = 0; i < coeff.size(); i++)
	{
		string name = "cylinder ";
		name = name + to_string(static_cast<long long>(i));
		viewer->addCylinder(coeff[i], name);
	}
}

void updateHandSkeleton(HandModel &handmodel)
{
	SK::Array<ModelCoefficients> coeff = handmodel.getSkeleton();
	for(size_t i = 0; i < coeff.size(); i++)
	{
		string name = "cylinder ";
		name = name + to_string(static_cast<long long>(i));
		viewer->removeShape(name);
		viewer->addCylinder(coeff[i], name);
	}
}

int main(int argc, char **argv)
{

	// Viewer initialization
	viewer->setBackgroundColor (0, 0, 0);
	viewer->initCameraParameters();
	viewer->addCoordinateSystem (1.0);
	
	// get dataset
	DataDriven data_driven = DataDriven("dataset/Trial01_General.mat");
/*	int count = 0;
	SK::Array<SK::Array<float>> dataset;
	ifstream infile("dataset/Trial03_General.mat");
	float tmpdata[26];
	int tmplate[] = {7, 6, 8, 9, 11, 10, 12, 13, 15, 14, 16, 17, 19, 18, 20, 21, 23, 22, 24, 25};
	while(infile >> tmpdata[0] >> tmpdata[1] >> tmpdata[2] >> tmpdata[3] >> tmpdata[4] >> tmpdata[5] >> tmpdata[6] >> tmpdata[7] 
				 >> tmpdata[8] >> tmpdata[9] >> tmpdata[10] >> tmpdata[11] >> tmpdata[12] >> tmpdata[13] >> tmpdata[14] >> tmpdata[15]
				 >> tmpdata[16] >> tmpdata[17] >> tmpdata[18] >> tmpdata[19] >> tmpdata[20] >> tmpdata[21] >> tmpdata[22] >> tmpdata[23]
				 >> tmpdata[24] >> tmpdata[25])
	{
		SK::Array<float> tmpset;
//		cout << "line " << count <<": ";
		for(int i = 0; i < 26; i++)
		{
//			cout << tmpdata[i] << " ";
			// Build parameter properly
			if(i < 20)
			{
				// Need to inverse x parameters: 
				if(i != 1 && i != 5 && i != 9 && i != 13 && i != 17)
					tmpset.pushBack(tmpdata[tmplate[i]] * -1);
				else
					tmpset.pushBack(tmpdata[tmplate[i]]);
			}
			else
			{
//				tmpset.pushBack(0.0);
				tmpset.pushBack(tmpdata[i - 20]);
			}
		}
//		cout << endl;
		dataset.pushBack(tmpset);
		count++;
	}*/
	SK::Array<SK::Array<float>> dataset = data_driven.getDataSet();
	cout << "loading dataset done, set size = " << data_driven.getDataSetSize() << endl;
	data_driven.startPCA(10);


	HandModel handmodel = HandModel();
	addHandModel(handmodel, skel);
	if(skel)	addHandSkeleton(handmodel);

	// Main while
	int frame = 0;
	while(!viewer->wasStopped())
	{
//		cout << "frame = " << frame << endl;
		// Hand Model & pose initialization
		handmodel = HandModel();
		HandPose handpose = HandPose();

		// Apply the new parameters
		SK::Array<float> curr_para = dataset[frame];
		handpose.setAllParameters(curr_para, false);
		handpose.applyPose(handmodel);

		updateHandModel(handmodel, skel);
		if(skel)	updateHandSkeleton(handmodel);
		
		viewer->spinOnce(1);
		frame = (frame + 1) % dataset.size();
	}

	return 0;

}