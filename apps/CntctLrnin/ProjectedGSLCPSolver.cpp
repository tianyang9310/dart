#include "ProjectedGSLCPSolver.h"
#include <glog/logging.h>

bool ProjectedGS(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                 Eigen::VectorXd& z, const Eigen::VectorXd& lo,
                 const Eigen::VectorXd& hi) {
  int mMaxIter = static_cast<int>(PGS_MAX_ITER);
  double mPGS_Zero = PGS_ZERO;
  int numRows = A.rows();

  assert(numRows == b.rows());
  assert(numRows == z.rows());
  assert(numRows == lo.rows());
  assert(numRows == hi.rows());

  // Pivoting s.t. diagonal terms are all non-zero
  std::vector<int> nonbas;
  for (int i = 0; i < numRows; ++i) {
    if (std::abs(A(i, i)) <= mPGS_Zero) {
      nonbas.push_back(i);
    }
  }

  Eigen::MatrixXd T = Eigen::MatrixXd::Identity(numRows, numRows);
  Eigen::MatrixXd TInv = Eigen::MatrixXd::Identity(numRows, numRows);

  for (int i = 0; i < nonbas.size(); ++i) {
    Eigen::ArrayXd dCol = A.col(nonbas[i]);
    int nIdx;
    double dColMax = dCol.abs().maxCoeff(&nIdx);
    if (dColMax > mPGS_Zero) {
      if (std::abs(A(nIdx, nIdx) - dCol(nIdx)) > mPGS_Zero) {
        T(nonbas[i], nIdx) = 1;
        TInv(nonbas[i], nIdx) = -1;
      } else if (std::abs(A(nIdx, nIdx) - 0.5 * dCol(nIdx)) > mPGS_Zero) {
        T(nonbas[i], nIdx) = 0.5;
        TInv(nonbas[i], nIdx) = -0.5;
      } else {
        LOG(FATAL) << "Pivoting Fail...." << std::endl;
      }
    } else {
      LOG(FATAL) << "All zero in one column of A" << std::endl;
    }
  }

  CHECK_EQ(T * TInv, Eigen::MatrixXd::Identity(numRows, numRows))
      << "T: " << std::endl
      << T << std::endl
      << "TInv:" << std::endl
      << TInv << std::endl;

  Eigen::MatrixXd newA = T * A * TInv;
  Eigen::VectorXd newb = T * b;
  Eigen::VectorXd newZ = T * z;
  Eigen::VectorXd newlo = T * lo;
  Eigen::VectorXd newhi = T * hi;

  Eigen::VectorXd oldZ = newZ;

  for (int k = 0; k < mMaxIter; ++k) {
    for (int i = 0; i < numRows; ++i) {
      double delta = 0;
      for (int j = 0; j < i; ++j) {
        delta += newA(i, j) * newZ[j];
      }
      for (int j = i + 1; j < numRows; ++j) {
        delta += newA(i, j) * newZ[j];
      }
      newZ[i] = (-newb[i] - delta) / newA(i, i);
      if (newZ[i] < newlo[i]) {
        newZ[i] = newlo[i];
      }
      if (newZ[i] > newhi[i]) {
        newZ[i] = newhi[i];
      }
    }
    if (((oldZ - newZ).array().abs() < mPGS_Zero).all()) {
      // std::cout << "Old z: " << oldZ.transpose() << std::endl;
      // std::cout << "New z: " << z.transpose() << std::endl;
      // std::cout << "Early stopping: " << k << std::endl;
      break;
    } else {
      oldZ = newZ;
    }
    std::cout << "Iter " << k << ": " << newZ.transpose() << std::endl;
  }

  // ------------------------------------------------------------------------
  std::cout << std::boolalpha << dart::lcpsolver::YT::validate(newA, newb, newZ)
            << std::endl;
  std::cout << "newA: " << std::endl << newA << std::endl;
  std::cout << "newb: " << std::endl << newb.transpose() << std::endl;
  std::cout << "A: " << std::endl << A << std::endl;
  std::cout << "b: " << std::endl << b.transpose() << std::endl;
  std::cout << "T: " << std::endl << T << std::endl;
  // ------------------------------------------------------------------------

  z = TInv * newZ;

  bool Validation = dart::lcpsolver::YT::validate(A, b, z);
  if (Validation) {
    return true;
  } else {
    z.setZero();
    return false;
  }
}

void permuteAandBforPGS(Eigen::MatrixXd& newA, Eigen::VectorXd& newb,
                        Eigen::MatrixXd& T, const Eigen::MatrixXd& A,
                        const Eigen::VectorXd& b, const Eigen::VectorXd& z0) {
  int numRows = A.rows();

  assert(numRows == A.cols());
  assert(numRows == b.rows());
  assert(numRows == z0.rows());

  // Here bas are index of non-zero item
  std::vector<int> bas;
  std::vector<int> nonbas;

  for (int i = 0; i < numRows; ++i) {
    if (z0[i] > 0) {
      bas.push_back(i);
    } else {
      nonbas.push_back(i);
    }
  }

  assert(bas.size() + nonbas.size() == numRows);

  T.resize(numRows, numRows);
  T.setZero();
  for (int i = 0; i < bas.size(); ++i) {
    T(i, bas[i]) = 1;
  }
  for (int i = 0; i < nonbas.size(); ++i) {
    T(bas.size() + i, nonbas[i]) = 1;
  }

  assert(T * T.transpose() == Eigen::MatrixXd::Identity(numRows, numRows));
  newA = T * A * T.transpose();
  newb = T * b;
}

void permuteProjectedGS(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                        Eigen::VectorXd& z, const Eigen::VectorXd& z0) {
  int numRows = A.rows();

  assert(numRows == A.cols());
  assert(numRows == b.rows());
  assert(numRows == z0.rows());

  Eigen::MatrixXd newA;
  Eigen::VectorXd newb;
  Eigen::MatrixXd T;
  permuteAandBforPGS(newA, newb, T, A, b, z0);

  Eigen::VectorXd newZ = z0;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(numRows);
  Eigen::VectorXd hi = Eigen::VectorXd::Constant(numRows, 1e9);

  bool succ = ProjectedGS(newA, newb, newZ, lo, hi);
  if (succ) {
    z = T.transpose() * newZ;
  } else {
    z = Eigen::VectorXd::Zero(numRows);
  }
}
