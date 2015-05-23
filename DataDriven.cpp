#include <fstream>
#include <iomanip>
#include <vector>
#include "DataDriven.hxx"
#include "MyTools.hxx"

const char* filenamelist[] = {"dataset/Trial01_General.mat", 
							  "dataset/Trial02_General.mat", 
							  "dataset/Trial03_General.mat", 
							  "dataset/Trial04_General.mat", 
							  "dataset/Trial05_General.mat", 
							  "dataset/Trial07_General.mat", 
							  "dataset/Trial08_General.mat", 
							  "dataset/Trial01_Interaction.mat", 
							  "dataset/Trial02_Interaction.mat", 
							  "dataset/Trial03_Interaction.mat", 
							  "dataset/Trial04_Interaction.mat", 
							  "dataset/Trial05_Interaction.mat", 
							  "dataset/Trial06_Interaction.mat", 
							  "dataset/Trial07_Interaction.mat", 
							  "dataset/Trial08_Interaction.mat", 
							  "dataset/Trial09_Interaction.mat", 
							  "dataset/Trial02_Language.mat"};

DataDriven::DataDriven()
{
	m = 26;
	for(int d = 0; d < 17; d++)
	{
		ifstream infile(filenamelist[d]);
		float tmpdata[26];
		int tmplate[] = {7, 6, 8, 9, 11, 10, 12, 13, 15, 14, 16, 17, 19, 18, 20, 21, 23, 22, 24, 25};
		while(infile >> tmpdata[0] >> tmpdata[1] >> tmpdata[2] >> tmpdata[3] >> tmpdata[4] >> tmpdata[5] >> tmpdata[6] >> tmpdata[7] 
					 >> tmpdata[8] >> tmpdata[9] >> tmpdata[10] >> tmpdata[11] >> tmpdata[12] >> tmpdata[13] >> tmpdata[14] >> tmpdata[15]
					 >> tmpdata[16] >> tmpdata[17] >> tmpdata[18] >> tmpdata[19] >> tmpdata[20] >> tmpdata[21] >> tmpdata[22] >> tmpdata[23]
					 >> tmpdata[24] >> tmpdata[25])
		{
			SK::Array<float> tmpset;
	//		cout << "line " << count <<": ";
			for(int i = 0; i < 20; i++)
			{
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
					tmpset.pushBack(tmpdata[i - 20]);
			}
	//		cout << endl;
			dataset.pushBack(tmpset);
		}
	}
	n = dataset.size();
}

DataDriven::DataDriven(int pick_num)
{
	m = 26;
	cout << "choose " << filenamelist[pick_num] << "as dataset..." << endl;
	ifstream infile(filenamelist[pick_num]);
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
				tmpset.pushBack(tmpdata[i - 20]);
		}
//		cout << endl;
		dataset.pushBack(tmpset);
	}
	n = dataset.size();
	
}

DataDriven::DataDriven(char* filename)
{
	m = 26;
	
	ifstream infile(filename);
	float tmpdata[26];
	int tmplate[] = {7, 6, 8, 9, 11, 10, 12, 13, 15, 14, 16, 17, 19, 18, 20, 21, 23, 22, 24, 25};
	while(infile >> tmpdata[0] >> tmpdata[1] >> tmpdata[2] >> tmpdata[3] >> tmpdata[4] >> tmpdata[5] >> tmpdata[6] >> tmpdata[7] 
				 >> tmpdata[8] >> tmpdata[9] >> tmpdata[10] >> tmpdata[11] >> tmpdata[12] >> tmpdata[13] >> tmpdata[14] >> tmpdata[15]
				 >> tmpdata[16] >> tmpdata[17] >> tmpdata[18] >> tmpdata[19] >> tmpdata[20] >> tmpdata[21] >> tmpdata[22] >> tmpdata[23]
				 >> tmpdata[24] >> tmpdata[25])
	{
		SK::Array<float> tmpset;
//		cout << "line " << count <<": ";
		for(int i = 0; i < 20; i++)
		{
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
				tmpset.pushBack(tmpdata[i - 20]);
		}
//		cout << endl;
		dataset.pushBack(tmpset);
	}
	n = dataset.size();
	
}

void DataDriven::startPCA(int pc_d)
{
	l = pc_d;
	int joint_m = m - 6;
	// assgin dataset into matrix
	MatrixXf data_matrix = MatrixXf::Zero(joint_m, n);
	for(int i = 0; i < n; i++)
	{
		VectorXf tmpvec = MyTools::SKtoEigenVector(dataset[i]);
		VectorXf tmpvec2 = tmpvec.segment(0, joint_m);
		data_matrix.col(i) = tmpvec2;
	}

	typedef pair<float, int> myPair;
	typedef vector<myPair> PermutationIndices;

	// Switch to zero-mean matrix
	for (int i = 0; i < data_matrix.cols(); i++)
	{
		float mean = (data_matrix.col(i).sum())/ joint_m;		 //compute mean
		mean_vector  = VectorXf::Constant(joint_m, mean); // create a vector with constant value = mean
		data_matrix.col(i) -= mean_vector;
		// cout << meanVector.transpose() << "\n" << DataPoints.col(i).transpose() << "\n\n";
	}

	// Get the covariance matrix
	cov_matrix = MatrixXf::Zero(joint_m, joint_m);
	cov_matrix = (1 / (float) n) * data_matrix * data_matrix.transpose();

	// Compute the eigenvalue on the cov Matrix
	EigenSolver<MatrixXf> m_solve(cov_matrix);
	
	cout << "PCA computation done." << endl;
	VectorXf eigenvalues = VectorXf::Zero(joint_m);
	eigenvalues = m_solve.eigenvalues().real();

	MatrixXf eigenVectors = MatrixXf::Zero(n, joint_m);  // matrix (n x m) (points, dims)
	eigenVectors = m_solve.eigenvectors().real();	

	// sort and get the permutation indices
	float total_var = 0.0;
	PermutationIndices pi;
	for (int i = 0 ; i < joint_m; i++)
	{
		total_var += eigenvalues(i);
		pi.push_back(make_pair(eigenvalues(i), i));
	}
	sort(pi.begin(), pi.end());
	reverse(pi.begin(), pi.end());

	// For checking
	for (int i = 0; i < joint_m ; i++)
		cout << "eigen = " << pi[i].first << " pi = " << pi[i].second << " var = " << (pi[i].first / total_var) << endl;

	// consider the subspace corresponding to the top-k eigenvectors
	MatrixXf cmat = MatrixXf::Zero(l, joint_m);
	for(int i = 0; i < l; i++)
	{
		cout << "Top " << (i + 1) << " in index = " << pi[i].second << " eigenvector:\n" << eigenVectors.row(pi[i].second) << endl;
		cmat.row(i) = eigenVectors.row(pi[i].second);
	}
/*	unsigned int i = pi.size()-1;
	unsigned int howMany = i - l;
	for (; i > howMany; i--)
	{
		cout << i << "-eigenvector: " << eigenVectors.row(i) << endl;
	}*/

	// build trans matrix
	trans_matrix = MatrixXf::Zero((l + 6), (joint_m + 6));
	trans_matrix.topLeftCorner(l, joint_m) = cmat;
	trans_matrix.bottomRightCorner(6, 6) = MatrixXf::Identity(6, 6);
	cout << endl << setprecision(3) << "trans matrix: " << endl << trans_matrix << endl;

}
