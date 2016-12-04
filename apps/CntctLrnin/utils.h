#ifndef MYUTILS
#define MYUTILS

#include "dart/dart.h"

#ifndef DART_CONTACT_CONSTRAINT_EPSILON
#define DART_CONTACT_CONSTRAINT_EPSILON 1.0e-6
#endif

#ifndef MY_DART_ZERO
#define MY_DART_ZERO 1.0e-7
#endif

Eigen::MatrixXd getTangentBasisMatrixLemke(const Eigen::Vector3d& _n,
                                           int numBasis);

#endif  // MYUTILS
