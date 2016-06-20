#include "utility.h"

namespace GPS_NSpace
{

VectorXd GaussianSampler(VectorXd mu, MatrixXd cov)
{
//  mu is mean
//  cov is the covariance

	EigenMultivariateNormal<double> normX_solver(mu, cov);
	return normX_solver.samples(1);
}

}
