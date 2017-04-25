#ifndef PROJECTEDGSLCPSOLVER_H
#define PROJECTEDGSLCPSOLVER_H

#include "apps/lemkeFix/myLemke.h"
#include "dart/dart.h"
#include "parameter.h"

/// Brief: Solve a LCP problem
/// A*z + b = w
/// z >= 0
/// w >= 0
/// z^T * w == 0
bool ProjectedGS(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                 Eigen::VectorXd& z, const Eigen::VectorXd& lo,
                 const Eigen::VectorXd& hi);

void permuteAandBforPGS(Eigen::MatrixXd& newA, Eigen::VectorXd& newb,
                        const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                        int idx0, int idx1);

#endif
