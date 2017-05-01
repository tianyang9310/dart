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
/// z is the inital guess, which is must be provided. If starts from no where,
/// please provide z as all zero vector
bool ProjectedGS(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                 Eigen::VectorXd& z, const Eigen::VectorXd& lo,
                 const Eigen::VectorXd& hi);

/// Brief: Permute NN-projected 0 variables to hind part
void permuteAandBforPGS(Eigen::MatrixXd& newA, Eigen::VectorXd& newb,
                        Eigen::MatrixXd& T, const Eigen::MatrixXd& A,
                        const Eigen::VectorXd& b, const Eigen::VectorXd& z0);

/// Brief: Wrapper for ProjectedGS and permuteAandBforPGS
void permuteProjectedGS(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                        Eigen::VectorXd& z, const Eigen::VectorXd& z0);

#endif
