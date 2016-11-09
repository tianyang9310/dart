#ifndef MYUTILS
#define MYUTILS

#include "dart/dart.h"

#ifndef DART_CONTACT_CONSTRAINT_EPSILON
#define DART_CONTACT_CONSTRAINT_EPSILON  1e-6
#endif

#ifndef DART_EPSILON
#define DART_EPSILON (1.0E-6)
#endif

Eigen::MatrixXd getTangentBasisMatrixLemke(const Eigen::Vector3d& _n, int numBasis);

#endif // MYUTILS
