#ifndef UTILITY_H
#define UTILITY_H

#include "eigenmvn.h"

namespace GPS_NSpace{

using namespace Eigen;

VectorXd GaussianSampler(VectorXd mu, MatrixXd conv);

}

#endif
