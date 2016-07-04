#ifndef UTILITY_H
#define UTILITY_H

#include "eigenmvn.h"
#include <math.h>

namespace GPS_NSpace{

using namespace Eigen;
using namespace std;

VectorXd GaussianSampler(VectorXd mu, MatrixXd conv);
double GaussianEvaluator(VectorXd mean, MatrixXd covariance, VectorXd testX);
double GaussianEvaluator(double mean, double covariance, double testX);

}

#endif
