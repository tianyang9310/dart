#include "My2DantzigLCPSolver.h"

//==============================================================================
My2DantzigLCPSolver::My2DantzigLCPSolver(double _timestep)
    : DantzigLCPSolver(_timestep) {
  numBasis = 8;
}

//==============================================================================
My2DantzigLCPSolver::~My2DantzigLCPSolver() {
  // pass
}

//==============================================================================
void My2DantzigLCPSolver::solve(ConstrainedGroup* _group) {
  // If there is no constraint, then just return true.
  size_t numConstraints = _group->getNumConstraints();
  numContactsCallBack = numConstraints;
  if (numConstraints == 0) return;

  // Build LCP terms by aggregating them from constraints
  size_t n = _group->getTotalDimension();
  // int nSkip = dPAD(n); // YT: Don't use nSkip
  double* A = new double[n * n];
  double* x = new double[n];
  double* b = new double[n];
  double* w = new double[n];
  double* lo = new double[n];
  double* hi = new double[n];
  int* findex = new int[n];

  // Set w to 0 and findex to -1
  std::memset(A, 0.0, n * n * sizeof(double));
  std::memset(w, 0.0, n * sizeof(double));
  std::memset(b, 0.0, n * sizeof(double));
  std::memset(findex, -1, n * sizeof(int));

  // Compute offset indices
  size_t* offset = new size_t[n];
  offset[0] = 0;
  //  std::cout << "offset[" << 0 << "]: " << offset[0] << std::endl;
  for (size_t i = 1; i < numConstraints; ++i) {
    ConstraintBase* constraint = _group->getConstraint(i - 1);
    assert(constraint->getDimension() > 0);
    offset[i] = offset[i - 1] + constraint->getDimension();
    //    std::cout << "offset[" << i << "]: " << offset[i] << std::endl;
  }

  // For each constraint
  ConstraintInfo constInfo;
  constInfo.invTimeStep = 1.0 / mTimeStep;
  ConstraintBase* constraint;

  Eigen::MatrixXd mu(numConstraints, numConstraints);
  Eigen::MatrixXd E(numConstraints * numBasis, numConstraints);
  mu.setIdentity();
  E.setZero();
  for (size_t i = 0; i < numConstraints; ++i) {
    constraint = _group->getConstraint(i);

    constInfo.x = x + offset[i];
    constInfo.lo = lo + offset[i];
    constInfo.hi = hi + offset[i];
    constInfo.b = b + offset[i];
    constInfo.findex = findex + offset[i];
    constInfo.w = w + offset[i];

    // -------------------------------------------------------------------------
    // Fill vectors: lo, hi, b, w, mu, E
    constraint->getInformation(&constInfo);
    mu(i, i) = dynamic_cast<My2ContactConstraint*>(constraint)->mFrictionCoeff;
    E.block(i * numBasis, i, numBasis, 1) = Eigen::VectorXd::Ones(numBasis);

    // -------------------------------------------------------------------------
    // Fill a matrix by impulse tests: A
    // Matrix A is row major
    constraint->excite();
    for (size_t j = 0; j < constraint->getDimension(); ++j) {
      /*
       * // comment out because findex only use in ODE solver
       * // Adjust findex for global index
       * if (findex[offset[i] + j] >= 0) findex[offset[i] + j] += offset[i];
       */

      // Apply impulse for mipulse test
      constraint->applyUnitImpulse(j);

      // Fill upper triangle blocks of A matrix
      int index = n * (offset[i] + j) + offset[i];
      constraint->getVelocityChange(A + index, true);
      for (size_t k = i + 1; k < numConstraints; ++k) {
        index = n * (offset[i] + j) + offset[k];
        _group->getConstraint(k)->getVelocityChange(A + index, false);
      }

      // Filling symmetric part of A matrix
      for (size_t k = 0; k < i; ++k) {
        for (size_t l = 0; l < _group->getConstraint(k)->getDimension(); ++l) {
          int index1 = n * (offset[i] + j) + offset[k] + l;
          int index2 = n * (offset[k] + l) + offset[i] + j;

          A[index1] = A[index2];
        }
      }
    }

    assert(isSymmetric(n, A, offset[i],
                       offset[i] + constraint->getDimension() - 1));

    constraint->unexcite();
    // -------------------------------------------------------------------------
  }
  // ---------------------------------------------------------------------------
  // Assertion
  assert(n == numConstraints * (1 + numBasis));

  // ---------------------------------------------------------------------------
  // Establish Lemke b
  Eigen::VectorXd Pre_Lemke_b = Eigen::Map<Eigen::VectorXd>(b, n);

  Eigen::VectorXd Lemke_b(numConstraints * (2 + numBasis));
  Eigen::MatrixXd T(numConstraints * (1 + numBasis),
                    numConstraints * (1 + numBasis));
  Lemke_b.setZero();
  T.setZero();

  PermuteNegAug_b(b, Lemke_b, Pre_Lemke_b, T);

  /*
   * // debug b
   * std::cout << "Print out b" << std::endl;
   * for (size_t tt=0; tt < n; tt++){
   *   std::cout << *(b+tt) << std::endl;
   * }
   * std::cout << "========================="<<std::endl << Pre_Lemke_b <<
   * std::endl;
   * std::cout << "========================="<<std::endl << Lemke_b <<
   * std::endl;
   * std::cin.get();
   */
  // ---------------------------------------------------------------------------
  // Establish Lemke A
  Eigen::MatrixXd Pre_Lemke_A = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      A, n, n);

  Eigen::MatrixXd Lemke_A(numConstraints * (2 + numBasis),
                          numConstraints * (2 + numBasis));
  Lemke_A.setZero();

  // Permute A
  PermuteAug_A(Pre_Lemke_A, Lemke_A, T, mu, E);

  /*
   *   // debug A
   *   std::cout << "A: " << std::endl;
   *   for (size_t i = 0; i < n; ++i) {
   *     for (size_t j = 0; j < n; ++j) {
   *       std::cout << std::setprecision(10) << A[i * n + j] << " ";
   *     }
   *     std::cout << std::endl;
   *   }
   *
   *   std::cout << "========================="<<std::endl << Pre_Lemke_A <<
   * std::endl;
   *   std::cout << "========================="<<std::endl << Lemke_A <<
   * std::endl;
   *   std::cin.get();
   */
  // ---------------------------------------------------------------------------

  assert(isSymmetric(n, A));

  // Print LCP formulation
  //  dtdbg << "Before solve:" << std::endl;
  //  print(n, A, x, lo, hi, b, w, findex);
  //  std::cout << std::endl;

  // Solve LCP using ODE's Dantzig algorithm
  dSolveLCP(n, A, x, b, w, 0, lo, hi, findex);

  // Print LCP formulation
  //  dtdbg << "After solve:" << std::endl;
  //  print(n, A, x, lo, hi, b, w, findex);
  //  std::cout << std::endl;

  // Apply constraint impulses
  for (size_t i = 0; i < numConstraints; ++i) {
    constraint = _group->getConstraint(i);
    constraint->applyImpulse(x + offset[i]);
    constraint->excite();
  }

  delete[] offset;

  delete[] A;
  delete[] x;
  delete[] b;
  delete[] w;
  delete[] lo;
  delete[] hi;
  delete[] findex;
}

//==============================================================================
void My2DantzigLCPSolver::PermuteNegAug_b(double* b, Eigen::VectorXd& Lemke_b,
                                          const Eigen::VectorXd& Pre_Lemke_b,
                                          Eigen::MatrixXd& T) {
  size_t mDim = 1 + numBasis;
  // Obtain transformation matrix
  int Trows = T.rows();
  int Tcols = T.cols();
  assert(Trows == mDim * numContactsCallBack);
  assert(Tcols == mDim * numContactsCallBack);
  Eigen::MatrixXd Tcache(Trows, Tcols);
  Tcache.setIdentity();

  // Permute b
  for (size_t mIdxConstraint = 0; mIdxConstraint < numContactsCallBack;
       mIdxConstraint++) {
    Lemke_b[mIdxConstraint] = *(b + mIdxConstraint * mDim);
    T.row(mIdxConstraint) = Tcache.row(mIdxConstraint * mDim);

    Lemke_b.segment(numContactsCallBack + mIdxConstraint * numBasis, numBasis) =
        Eigen::Map<Eigen::VectorXd>((b + mIdxConstraint * mDim + 1), numBasis);
    T.block(numContactsCallBack + mIdxConstraint * numBasis, 0, numBasis,
            Tcols) =
        Tcache.block(mIdxConstraint * mDim + 1, 0, numBasis, Tcols);
  }

  // // Assert transformation
  // std::cout << "T*b" << std::endl << T*Pre_Lemke_b << std::endl;
  // std::cout << "b' " << std::endl << Lemke_b.head(Pre_Lemke_b.rows()) <<
  // std::endl;
  assert(T * Pre_Lemke_b == Lemke_b.head(Pre_Lemke_b.rows()));

  // Negate b
  Lemke_b = -Lemke_b;

  // Augment b
  Lemke_b.tail(numContactsCallBack).setZero();
}

//==============================================================================
void My2DantzigLCPSolver::PermuteAug_A(const Eigen::MatrixXd& Pre_Lemke_A,
                                       Eigen::MatrixXd& Lemke_A,
                                       const Eigen::MatrixXd& T,
                                       const Eigen::MatrixXd& mu,
                                       const Eigen::MatrixXd& E) {
  size_t mDim = 1 + numBasis;
  // Permute A
  Lemke_A.block(0, 0, Pre_Lemke_A.rows(), Pre_Lemke_A.cols()) =
      T * Pre_Lemke_A * T.inverse();

  // Augment A
  // _TimeStep =  1.0 means using calculating impulse in Lemke
  // otherwise _TimeStep  = mTimeStep means calculating force in Lemke
  double _TimeStep = 1.0;  // mTimeStep;
  Lemke_A.block(numContactsCallBack * mDim, 0, numContactsCallBack,
                numContactsCallBack) =
      mu / (_TimeStep == 1.0 ? mTimeStep : 1.0);
  Lemke_A.block(numContactsCallBack * mDim, numContactsCallBack,
                numContactsCallBack, numContactsCallBack * numBasis) =
      -E.transpose() / (_TimeStep == 1.0 ? mTimeStep : 1.0);
  Lemke_A.block(numContactsCallBack, numContactsCallBack * mDim,
                numContactsCallBack * numBasis, numContactsCallBack) =
      E / (_TimeStep == 1.0 ? mTimeStep : 1.0);
}

//==============================================================================
void My2DantzigLCPSolver::print(size_t _n, double* _A, double* _x, double* lo,
                                double* hi, double* b, double* w, int* findex) {
  size_t nSkip = _n;
  std::cout << "A: " << std::endl;
  for (size_t i = 0; i < _n; ++i) {
    for (size_t j = 0; j < nSkip; ++j) {
      std::cout << std::setprecision(10) << _A[i * nSkip + j] << " ";
    }
    std::cout << std::endl;
  }

  std::cout << "b: ";
  for (size_t i = 0; i < _n; ++i) {
    std::cout << std::setprecision(10) << b[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "w: ";
  for (size_t i = 0; i < _n; ++i) {
    std::cout << w[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "x: ";
  for (size_t i = 0; i < _n; ++i) {
    std::cout << _x[i] << " ";
  }
  std::cout << std::endl;

  //  std::cout << "lb: ";
  //  for (int i = 0; i < dim; ++i)
  //  {
  //    std::cout << lb[i] << " ";
  //  }
  //  std::cout << std::endl;

  //  std::cout << "ub: ";
  //  for (int i = 0; i < dim; ++i)
  //  {
  //    std::cout << ub[i] << " ";
  //  }
  //  std::cout << std::endl;

  // std::cout << "frictionIndex: ";
  // for (size_t i = 0; i < _n; ++i)
  // {
  //     std::cout << findex[i] << " ";
  // }
  // std::cout << std::endl;

  double* Ax = new double[_n];

  for (size_t i = 0; i < _n; ++i) {
    Ax[i] = 0.0;
  }

  for (size_t i = 0; i < _n; ++i) {
    for (size_t j = 0; j < _n; ++j) {
      Ax[i] += _A[i * nSkip + j] * _x[j];
    }
  }

  std::cout << "Ax   : ";
  for (size_t i = 0; i < _n; ++i) {
    std::cout << Ax[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "b + w: ";
  for (size_t i = 0; i < _n; ++i) {
    std::cout << b[i] + w[i] << " ";
  }
  std::cout << std::endl;

  delete[] Ax;
}

//==============================================================================
bool My2DantzigLCPSolver::isSymmetric(size_t _n, double* _A) {
  size_t nSkip = _n;
  for (size_t i = 0; i < _n; ++i) {
    for (size_t j = 0; j < _n; ++j) {
      if (std::abs(_A[nSkip * i + j] - _A[nSkip * j + i]) > 1e-6) {
        std::cout << "A: " << std::endl;
        for (size_t k = 0; k < _n; ++k) {
          for (size_t l = 0; l < nSkip; ++l) {
            std::cout << std::setprecision(10) << _A[k * nSkip + l] << " ";
          }
          std::cout << std::endl;
        }

        std::cout << "A(" << i << ", " << j << "): " << _A[nSkip * i + j]
                  << std::endl;
        std::cout << "A(" << j << ", " << i << "): " << _A[nSkip * j + i]
                  << std::endl;
        return false;
      }
    }
  }

  return true;
}

//==============================================================================
bool My2DantzigLCPSolver::isSymmetric(size_t _n, double* _A, size_t _begin,
                                      size_t _end) {
  size_t nSkip = _n;
  for (size_t i = _begin; i <= _end; ++i) {
    for (size_t j = _begin; j <= _end; ++j) {
      if (std::abs(_A[nSkip * i + j] - _A[nSkip * j + i]) > 1e-6) {
        std::cout << "A: " << std::endl;
        for (size_t k = 0; k < _n; ++k) {
          for (size_t l = 0; l < nSkip; ++l) {
            std::cout << std::setprecision(10) << _A[k * nSkip + l] << " ";
          }
          std::cout << std::endl;
        }

        std::cout << "A(" << i << ", " << j << "): " << _A[nSkip * i + j]
                  << std::endl;
        std::cout << "A(" << j << ", " << i << "): " << _A[nSkip * j + i]
                  << std::endl;
        return false;
      }
    }
  }

  return true;
}
