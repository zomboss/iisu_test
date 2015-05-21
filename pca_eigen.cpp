#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

int main(int, char *[])
{
	unsigned int dimSpace = 10; // dimension space
	unsigned int m = 26;   // dimension of each point
	unsigned int n = 100;  // number of points 
	
	// this is a toy example, set some randomness in the origanl point space
	MatrixXd DataPoints = MatrixXd::Random(m, n);  // matrix (m x n)

	double mean; VectorXd meanVector;

	typedef pair<double, int> myPair;
	typedef vector<myPair> PermutationIndices;	

	// for each point, center the poin with the mean among all the coordinates
	for (int i = 0; i < DataPoints.cols(); i++)
	{
		mean = (DataPoints.col(i).sum())/ m;		 //compute mean
		meanVector  = VectorXd::Constant(m, mean); // create a vector with constant value = mean
		DataPoints.col(i) -= meanVector;
		// cout << meanVector.transpose() << "\n" << DataPoints.col(i).transpose() << "\n\n";
	}

	// get the covariance matrix
	MatrixXd Covariance = MatrixXd::Zero(m, m);
	Covariance = (1 / (double) n) * DataPoints * DataPoints.transpose();
	//  cout << Covariance ;	

	// compute the eigenvalue on the Cov Matrix
	EigenSolver<MatrixXd> m_solve(Covariance);
	cout << "PCA computation done." << endl;
	VectorXd eigenvalues = VectorXd::Zero(m);
	eigenvalues = m_solve.eigenvalues().real();
  
	MatrixXd eigenVectors = MatrixXd::Zero(n, m);  // matrix (n x m) (points, dims)
	eigenVectors = m_solve.eigenvectors().real();	

	// sort and get the permutation indices
	PermutationIndices pi;
	for (unsigned int i = 0 ; i < m; i++)
		pi.push_back(make_pair(eigenvalues(i), i));

	sort(pi.begin(), pi.end());

	for (unsigned int i = 0; i < m ; i++)
		cout << "eigen = " << pi[i].first << " pi = " << pi[i].second << endl;

	// consider the subspace corresponding to the top-k eigenvectors
	unsigned int i = pi.size()-1;
	unsigned int howMany = i - dimSpace;
	for (; i > howMany; i--)
	{
		cout << i << "-eigenvector " << eigenVectors.row(i) << endl;
	}

	system("pause");

	return 0;
}

