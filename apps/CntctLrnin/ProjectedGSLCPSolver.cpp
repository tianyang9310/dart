#include "ProjectedGSLCPSolver.h"

bool ProjectedGS(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, Eigen::VectorXd& z,
				const Eigen::VectorXd& lo, const Eigen::VectorXd& hi) {
	int mMaxIter = static_cast<int>(PGS_MAX_ITER);
	double mPGS_Zero = PGS_ZERO;
	int numRows = A.rows();

	Eigen::VectorXd oldZ = z;

	assert(numRows == b.rows());
	assert(numRows == z.rows());
	assert(numRows == lo.rows());
	assert(numRows == hi.rows());

	for (int k = 0; k < mMaxIter; ++k)
	{
		for (int i = 0; i < numRows; ++i)
		{
			double delta = 0;
			for (int j = 0; j < i; ++j)
			{
				delta += A(i,j) * z[j];
			}
			for (int j = i+1; j < numRows; ++j)
			{
				delta += A(i,j) * z[j];
			}
			z[i] = (- b[i] - delta) / A(i,i);
			if (z[i] < lo[i]) {
				z[i] = lo[i];
			}
			if (z[i] > hi[i]) {
				z[i] = hi[i];
			}
		}
		if (((oldZ - z).array().abs() < mPGS_Zero).all()) {
			// std::cout << "Old z: " << oldZ.transpose() << std::endl;
			// std::cout << "New z: " << z.transpose() << std::endl;
			// std::cout << "Early stopping: " << k << std::endl;
			break;
		} else {
			oldZ = z;
		}
	}
	bool Validation = dart::lcpsolver::YT::validate(A, b, z);
	if (Validation) {
		return true;
	} else {
		z.setZero();
		return false;
	}
	
}