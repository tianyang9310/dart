#ifndef PROJECTEDGSLCPSOLVER_H
#define PROJECTEDGSLCPSOLVER_H

#include "dart/dart.h"
#include "parameter.h"
#include "apps/lemkeFix/myLemke.h"

/// Brief: Solve a LCP problem 
/// A*z + b = w
/// z >= 0
/// w >= 0
/// z^T * w == 0
bool ProjectedGS(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, Eigen::VectorXd& z,
				const Eigen::VectorXd& lo, const Eigen::VectorXd& hi);

#endif