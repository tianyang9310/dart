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

double GaussianEvaluator(VectorXd mean, MatrixXd covariance, VectorXd testX)
{
//  This function should be able to handle multivariate case
//  Here covariance is variance, not standard deviation!!!
    double probability = 0;
    return probability;
}

double GaussianEvaluator(double mean, double covariance, double testX)
{
//  This function is a specific one to compute probability for univariate case
//  Here covariance is variance, not standard deviation!!!
    double probability = 0;
    probability = 1/sqrt(2*M_PI*covariance)*exp(-pow((testX-mean),2)/(covariance*2));
    return probability;
}

}
