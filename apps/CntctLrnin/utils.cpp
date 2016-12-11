#include "utils.h"

Eigen::MatrixXd getTangentBasisMatrixLemke(const Eigen::Vector3d& _n,
                                           int numBasis) {
  // TODO(JS): Use mNumFrictionConeBases
  // Check if the number of bases is even number.
  //  bool isEvenNumBases = mNumFrictionConeBases % 2 ? true : false;

  Eigen::MatrixXd T(Eigen::MatrixXd::Zero(3, numBasis));

  // Pick an arbitrary vector to take the cross product of (in this case,
  // Z-axis)
  Eigen::Vector3d tangent = Eigen::Vector3d::UnitZ().cross(_n);

  // TODO(JS): Modify following lines once _updateFirstFrictionalDirection() is
  //           implemented.
  // If they're too close, pick another tangent (use X-axis as arbitrary vector)
  if (tangent.norm() < DART_CONTACT_CONSTRAINT_EPSILON)
    tangent = Eigen::Vector3d::UnitX().cross(_n);

  tangent.normalize();

  // Rotate the tangent around the normal to compute bases.
  // Note: a possible speedup is in place for mNumDir % 2 = 0
  // Each basis and its opposite belong in the matrix, so we iterate half as
  // many times
  T.col(0) = tangent;
  for (size_t idx_basis = 1; idx_basis < numBasis; idx_basis++) {
    T.col(idx_basis) =
        Eigen::Quaterniond(Eigen::AngleAxisd(DART_PI_HALF / 2, _n)) *
        T.col(idx_basis - 1);
    if (T.col(idx_basis).dot(_n) > MY_DART_ZERO) {
      dterr << "Error in constructing basis matrix" << std::endl;
    }
  }
  // Here it should also be unnecessary to clamp the data
  // But in order to eliminate the error introduced here, clamp it
  clampZero(T);
  return T;
}


