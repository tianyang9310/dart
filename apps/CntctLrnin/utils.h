#ifndef MYUTILS
#define MYUTILS

#include "dart/dart.h"

#ifndef DART_CONTACT_CONSTRAINT_EPSILON
#define DART_CONTACT_CONSTRAINT_EPSILON 1.0e-6
#endif

#ifndef MY_DART_ZERO
#define MY_DART_ZERO 1.0e-7
#endif

// #define REGULARIZED_PRINT // print regularized info

Eigen::MatrixXd getTangentBasisMatrixLemke(const Eigen::Vector3d& _n,
                                           int numBasis);

template <typename T>
void clampZero(T& z) {
  // manually regularize Lemke solution right before apply impulse
  // after regularization, w might not satisfy LCP condition
  //
  // Zero regularization
  Eigen::VectorXd oldZ = z;
  Eigen::VectorXd myZero = Eigen::VectorXd::Zero(z.rows());
  z = (z.array() > -MY_DART_ZERO && z.array() < MY_DART_ZERO)
             .select(myZero, z);
  if (((oldZ).array() > -MY_DART_ZERO && (oldZ).array() < MY_DART_ZERO &&
       (oldZ).array() != 0)
          .any()) {
#ifdef REGULARIZED_PRINT
    dtwarn << "Before zero regularized: z is " << oldZ.transpose() << std::endl;
    dtwarn << "After zero regularized: z is " << z.transpose() << std::endl;
#endif
  }
}

template <typename T1, typename T2>
void clampNeg(T1& z, T2 Validation) {
  // Negative regularization
  Eigen::VectorXd oldZ = z;
  Eigen::VectorXd myZero = Eigen::VectorXd::Zero(z.rows());
  if (Validation) {
    oldZ = z;
    z = (z.array() < -MY_DART_ZERO).select(myZero, z);
    if (((oldZ).array() < -MY_DART_ZERO).any()) {
#ifdef REGULARIZED_PRINT
      dtwarn << "Before negative regularized: z is " << oldZ.transpose()
             << std::endl;
      dtwarn << "After negative regularized: z is" << z.transpose()
             << std::endl;
#endif
    }
  }
}

#endif  // MYUTILS
