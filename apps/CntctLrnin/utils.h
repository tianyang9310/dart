#ifndef MYUTILS
#define MYUTILS

#include "dart/dart.h"

#ifndef DART_CONTACT_CONSTRAINT_EPSILON
#define DART_CONTACT_CONSTRAINT_EPSILON 1e-6
#endif

#ifndef MY_DART_ZERO
#define MY_DART_ZERO 1e-6
#endif

#ifndef CLAMP_ZERO
#define CLAMP_ZERO 1e-12
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
  auto oldZ = z;
  auto myZero = z;
  myZero.setZero();
  z = (z.array() > -CLAMP_ZERO && z.array() < CLAMP_ZERO)
             .select(myZero, z);
  if (((oldZ).array() > -CLAMP_ZERO && (oldZ).array() < CLAMP_ZERO &&
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
  auto oldZ = z;
  auto myZero = z;
  myZero.setZero();
  if (Validation) {
    oldZ = z;
    z = (z.array() < -CLAMP_ZERO).select(myZero, z);
    if (((oldZ).array() < -CLAMP_ZERO).any()) {
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
