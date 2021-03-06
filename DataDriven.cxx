#include <fstream>
#include <vector>
#include "DataDriven.hxx"
#include "MyTools.hxx"

const float threshold = 0.9f;
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
	cout << "choose all data to dataset..." << endl;
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
	cout << "DataDriven setup done" << endl;
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
	cout << "DataDriven setup done" << endl;
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
	cout << "DataDriven setup done" << endl;
}

void DataDriven::startPCA(bool show = false)
{
	int joint_m = m - 6;
	// assgin dataset into matrix
	data_matrix = MatrixXf::Zero(joint_m, n);
	for(int i = 0; i < n; i++)
	{
		VectorXf tmpvec = MyTools::SKtoEigenVector(dataset[i]);
		VectorXf tmpvec2 = tmpvec.segment(0, joint_m);
		data_matrix.col(i) = tmpvec2;
	}
	MatrixXf zero_mean_matrix = data_matrix;

	typedef pair<float, int> myPair;
	typedef vector<myPair> PermutationIndices;

	// Switch to zero-mean matrix and compute mean_vector
	mean_vector = VectorXf::Zero(m);
	for (int i = 0; i < data_matrix.rows(); i++)
	{
		float mean = (data_matrix.row(i).sum())/ n;		 //compute mean
		mean_vector[i] = mean;
		VectorXf tmp_vec  = VectorXf::Constant(n, mean); // create a vector with constant value = mean
		zero_mean_matrix.row(i) -= tmp_vec;
	}
	if(show)	cout << "mean vector: " << endl << mean_vector << endl;

	// Get the covariance matrix
	cov_matrix = MatrixXf::Zero(joint_m, joint_m);
	cov_matrix = (1 / (float) n) * zero_mean_matrix * zero_mean_matrix.transpose();

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

	// Get the suitable l
	float cumulation = 0.0;
	for (int i = 0; i < joint_m ; i++)
	{
		cumulation += (pi[i].first / total_var);
		if(cumulation > threshold)
		{
			l = i + 1;
			break;
		}
	}
	if(show)	cout << "suitable l = " << l << endl;
	
	// For checking
	if(show)
		for (int i = 0; i < joint_m ; i++)
			cout << "eigen = " << pi[i].first << " pi = " << pi[i].second << " var = " << (pi[i].first / total_var) << endl;

	// consider the subspace corresponding to the top-k eigenvectors
	MatrixXf cmat = MatrixXf::Zero(l, joint_m);
	for(int i = 0; i < l; i++)
	{
		if(show)	cout << "Top " << (i + 1) << " in index = " << pi[i].second << " eigenvector:\n" << eigenVectors.row(pi[i].second) << endl;
		cmat.row(i) = eigenVectors.row(pi[i].second);
	}

	trans_matrix = MatrixXf::Zero((l + 6), (joint_m + 6));
	trans_matrix.topLeftCorner(l, joint_m) = cmat;
	trans_matrix.bottomRightCorner(6, 6) = MatrixXf::Identity(6, 6);

	// build min & max vectors
	buildMinMax();
}

void DataDriven::startPCA(int pc_d, bool show = false)
{
	l = pc_d;
	int joint_m = m - 6;
	// assgin dataset into matrix
	data_matrix = MatrixXf::Zero(joint_m, n);
	for(int i = 0; i < n; i++)
	{
		VectorXf tmpvec = MyTools::SKtoEigenVector(dataset[i]);
		VectorXf tmpvec2 = tmpvec.segment(0, joint_m);
		data_matrix.col(i) = tmpvec2;
	}
	MatrixXf zero_mean_matrix = data_matrix;

	typedef pair<float, int> myPair;
	typedef vector<myPair> PermutationIndices;

	// Switch to zero-mean matrix and compute mean_vector
	mean_vector = VectorXf::Zero(m);
	for (int i = 0; i < data_matrix.rows(); i++)
	{
		float mean = (data_matrix.row(i).sum())/ n;		 //compute mean
		mean_vector[i] = mean;
		VectorXf tmp_vec  = VectorXf::Constant(n, mean); // create a vector with constant value = mean
		zero_mean_matrix.row(i) -= tmp_vec;
	}
//	cout << "mean vector: " << endl << mean_vector << endl;

	// Get the covariance matrix
	cov_matrix = MatrixXf::Zero(joint_m, joint_m);
	cov_matrix = (1 / (float) n) * zero_mean_matrix * zero_mean_matrix.transpose();

	// Compute the eigenvalue on the cov Matrix
	EigenSolver<MatrixXf> m_solve(cov_matrix);
	
	if(show)	cout << "PCA computation done." << endl;
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
	if(show)
		for (int i = 0; i < joint_m ; i++)
			cout << "eigen = " << pi[i].first << " pi = " << pi[i].second << " var = " << (pi[i].first / total_var) << endl;

	// consider the subspace corresponding to the top-k eigenvectors
	MatrixXf cmat = MatrixXf::Zero(l, joint_m);
	for(int i = 0; i < l; i++)
	{
		if(show)	cout << "Top " << (i + 1) << " in index = " << pi[i].second << " eigenvector:\n" << eigenVectors.row(pi[i].second) << endl;
		cmat.row(i) = eigenVectors.row(pi[i].second);
	}

	// build trans matrix
	trans_matrix = MatrixXf::Zero((l + 6), (joint_m + 6));
	trans_matrix.topLeftCorner(l, joint_m) = cmat;
	trans_matrix.bottomRightCorner(6, 6) = MatrixXf::Identity(6, 6);

	// build min & max vectors
	buildMinMax();
}

void DataDriven::buildMinMax()
{
	max_vector = VectorXf::Zero(l + 6);
	min_vector = VectorXf::Zero(l + 6);
	MatrixXf prin_matrix = trans_matrix.topLeftCorner(l, m - 6);
	MatrixXf pca_data_matrix = prin_matrix * data_matrix;
//	cout << "show pca_data_matrix size = (" << pca_data_matrix.rows() << ", " << pca_data_matrix.cols() << ")\n";
	for (int i = 0; i < pca_data_matrix.rows(); i++)
	{
		float max = pca_data_matrix.row(i).maxCoeff();
		max_vector[i] = max;
		float min = pca_data_matrix.row(i).minCoeff();
		min_vector[i] = min;
	}
}
