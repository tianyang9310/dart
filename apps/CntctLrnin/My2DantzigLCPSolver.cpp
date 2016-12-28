#include "My2DantzigLCPSolver.h"

//==============================================================================
My2DantzigLCPSolver::My2DantzigLCPSolver(double _timestep)
    : DantzigLCPSolver(_timestep) {
  numBasis = NUMBASIS;
}

//==============================================================================
My2DantzigLCPSolver::~My2DantzigLCPSolver() {
  // pass
}

//==============================================================================
void My2DantzigLCPSolver::solve(ConstrainedGroup* _group) {
  std::cout << std::setprecision(10);
  // If there is no constraint, then just return true.
  size_t numConstraints = _group->getNumConstraints();
  numContactsCallBack = numConstraints;
  if (numConstraints == 0) return;

  // Build LCP terms by aggregating them from constraints
  size_t n = _group->getTotalDimension();
  int nSkip = dPAD(n);  // YT: Don't use nSkip
  double* A;
  if (numBasis != 2) {
    A = new double[n * n];
  } else {
    A = new double[n * nSkip];
  }
  double* x = new double[n];
  double* b = new double[n];
  double* w = new double[n];
  double* lo = new double[n];
  double* hi = new double[n];
  int* findex = new int[n];

  // Set w to 0 and findex to -1
  if (numBasis != 2) {
    std::memset(A, 0.0, n * n * sizeof(double));
  } else {
    std::memset(A, 0.0, n * nSkip * sizeof(double));
  }
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
      int index;
      if (numBasis != 2) {
        index = n * (offset[i] + j) + offset[i];
      } else {
        index = nSkip * (offset[i] + j) + offset[i];
      }
      constraint->getVelocityChange(A + index, true);
      for (size_t k = i + 1; k < numConstraints; ++k) {
        if (numBasis != 2) {
          index = n * (offset[i] + j) + offset[k];
        } else {
          index = nSkip * (offset[i] + j) + offset[k];
        }
        _group->getConstraint(k)->getVelocityChange(A + index, false);
      }

      // Filling symmetric part of A matrix
      for (size_t k = 0; k < i; ++k) {
        for (size_t l = 0; l < _group->getConstraint(k)->getDimension(); ++l) {
          int index1;
          int index2;
          if (numBasis != 2) {
            index1 = n * (offset[i] + j) + offset[k] + l;
            index2 = n * (offset[k] + l) + offset[i] + j;
          } else {
            index1 = nSkip * (offset[i] + j) + offset[k] + l;
            index2 = nSkip * (offset[k] + l) + offset[i] + j;
          }

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
  Eigen::MatrixXd Pre_Lemke_A;
  if (numBasis != 2) {
    Pre_Lemke_A = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        A, n, n);
  } else {
    Eigen::MatrixXd Pre_Lemke_A_nSkip = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        A, n, nSkip);
    Pre_Lemke_A = Pre_Lemke_A_nSkip.block(0, 0, n, n);
  }

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
  // Using Lemke to solve
  std::cout << "# ct points: " << numContactsCallBack << std::endl;
  if (numBasis != 2) {
    Eigen::VectorXd* z = new Eigen::VectorXd(numConstraints * (2 + numBasis));
    int err = dart::lcpsolver::YT::Lemke(Lemke_A, Lemke_b, z);

    double err_dist = 0.0;
    bool Validation =
        dart::lcpsolver::YT::validate(Lemke_A, (*z), Lemke_b, err_dist);

    // Debug Lemke
    std::cout << std::endl
              << "====================================" << std::endl;
    std::cout << "Matrix A " << std::endl << Lemke_A << std::endl;
    std::cout << "Vector b " << std::endl << Lemke_b.transpose() << std::endl;
    std::cout << "Vector z " << std::endl << (*z).transpose() << std::endl;

    for (size_t i = 0; i < numConstraints; i++) {
      ContactConstraint* cntctconstraint =
          dynamic_cast<ContactConstraint*>(_group->getConstraint(i));
      std::cout << "BodyNode1 old constraint impulse "
                << cntctconstraint->mBodyNode1->mConstraintImpulse.transpose()
                << std::endl;
      std::cout << "BodyNode2 old constraint impulse "
                << cntctconstraint->mBodyNode2->mConstraintImpulse.transpose()
                << std::endl;
    }

    if (Validation) {
      //  ---------------------------------------
      // justify the (*z)
      // assert(!(Eigen::isnan((*z).array()).any()));

      My2ContactConstraint* Mycntctconstraint;
      // (*z); N; B
      Eigen::VectorXd fn((*z).head(numConstraints));
      Eigen::VectorXd fd(
          (*z).segment(numConstraints, numConstraints * numBasis));
      // Eigen::VectorXd lambda((*z).tail(numConstraints));

      // Using Lemke to simulate
      for (size_t idx_cnstrnt = 0; idx_cnstrnt < numConstraints;
           ++idx_cnstrnt) {
        double fn_each = fn(idx_cnstrnt);
        Eigen::VectorXd fd_each = fd.segment(idx_cnstrnt * numBasis, numBasis);

        Mycntctconstraint = dynamic_cast<My2ContactConstraint*>(
            _group->getConstraint(idx_cnstrnt));
        Mycntctconstraint->MyapplyImpulse(fn_each, fd_each, true);

        Mycntctconstraint->excite();
      }
    } else {
      std::cout << "Lemke fails!!!" << std::endl;
      std::cin.get();
    }

    for (size_t i = 0; i < numConstraints; i++) {
      ContactConstraint* cntctconstraint =
          dynamic_cast<ContactConstraint*>(_group->getConstraint(i));
      std::cout << "BodyNode1 new constraint impulse "
                << cntctconstraint->mBodyNode1->mConstraintImpulse.transpose()
                << std::endl;
      std::cout << "BodyNode2 new constraint impulse "
                << cntctconstraint->mBodyNode2->mConstraintImpulse.transpose()
                << std::endl;
    }
  } else {
    // ---------------------------------------------------------------------------
    assert(isSymmetric(n, A));

    double* old_A = new double[n * nSkip];
    double* old_b = new double[n];
    for (int i = 0; i < n * nSkip; i++) old_A[i] = A[i];
    for (int i = 0; i < n; i++) old_b[i] = b[i];

    // Print LCP formulation
    std::cout << "Before solve:" << std::endl;
    print(n, A, x, lo, hi, b, w, findex);
    std::cout << std::endl;

    // Solve LCP using ODE's Dantzig algorithm
    dSolveLCP(n, A, x, b, w, 0, lo, hi, findex);

    //``````````````````````````````````````````````````````````````````````````
    // Solve LCP using Lemke
    Pre_Lemke_b = -Pre_Lemke_b;
    Eigen::VectorXd* z = new Eigen::VectorXd(numConstraints * (1 + numBasis));
    int err = dart::lcpsolver::YT::Lemke(Pre_Lemke_A, Pre_Lemke_b, z);

    double err_dist = 0.0;
    bool Validation =
        dart::lcpsolver::YT::validate(Pre_Lemke_A, (*z), Pre_Lemke_b, err_dist);

    std::cout << std::endl
              << "```````````````````````````````````````````````" << std::endl;
    std::cout << "Matrix A " << std::endl << Pre_Lemke_A << std::endl;
    std::cout << "Vector b " << std::endl
              << Pre_Lemke_b.transpose() << std::endl;
    std::cout << "Vector z " << (*z).transpose() << std::endl;
    std::cout << "Vector w " << (Pre_Lemke_A *(*z)+Pre_Lemke_b).transpose() << std::endl;
    std::cout << "[w].*[z] " 
              << ((*z).array()*(Pre_Lemke_A *(*z)+Pre_Lemke_b).array()).transpose() << std::endl;
    std::cout << "Validation: " << std::boolalpha << Validation << std::endl;
    std::cout << std::endl
              << "```````````````````````````````````````````````" << std::endl;
    std::cout << std::endl;

    // std::cin.get();
    //``````````````````````````````````````````````````````````````````````````

    // Print LCP formulation
    std::cout << "After solve:" << std::endl;
    print(n, old_A, x, lo, hi, old_b, w, findex);
    std::cout << std::endl;

    for (size_t i = 0; i < numConstraints; i++) {
      ContactConstraint* cntctconstraint =
          dynamic_cast<ContactConstraint*>(_group->getConstraint(i));
      std::cout << "BodyNode1 old constraint impulse "
                << cntctconstraint->mBodyNode1->mConstraintImpulse.transpose()
                << std::endl;
      std::cout << "BodyNode2 old constraint impulse "
                << cntctconstraint->mBodyNode2->mConstraintImpulse.transpose()
                << std::endl;
    }

    // Apply constraint impulses
    for (size_t i = 0; i < numConstraints; ++i) {
      constraint = _group->getConstraint(i);
      constraint->applyImpulse(x + offset[i]);

      /*
       * My2ContactConstraint* Mycntctconstraint;
       * Mycntctconstraint = dynamic_cast<My2ContactConstraint*>(
       *     _group->getConstraint(i));
       * double fn_each = (*z)(i*(1+numBasis));
       * Eigen::VectorXd fd_each = (*z).segment(i*(1+numBasis)+1,numBasis);
       * Mycntctconstraint->MyapplyImpulse(fn_each, fd_each, true);
       */

      constraint->excite();
    }

    for (size_t i = 0; i < numConstraints; i++) {
      ContactConstraint* cntctconstraint =
          dynamic_cast<ContactConstraint*>(_group->getConstraint(i));
      std::cout << "BodyNode1 new constraint impulse "
                << cntctconstraint->mBodyNode1->mConstraintImpulse.transpose()
                << std::endl;
      std::cout << "BodyNode2 new constraint impulse "
                << cntctconstraint->mBodyNode2->mConstraintImpulse.transpose()
                << std::endl;
    }
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
  size_t nSkip;
  if (numBasis != 2) {
    nSkip = _n;
  } else {
    nSkip = dPAD(_n);
  }
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

   std::cout << "lb: ";
   for (int i = 0; i < _n; ++i)
   {
     std::cout << lo[i] << " ";
   }
   std::cout << std::endl;

   std::cout << "ub: ";
   for (int i = 0; i < _n; ++i)
   {
     std::cout << hi[i] << " ";
   }
   std::cout << std::endl;

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

  std::cout << "x^T * w: ";
  for (size_t i = 0; i < _n; ++i) {
    std::cout << _x[i]*w[i] << " ";
  }
  std::cout << std::endl;

  delete[] Ax;
}

//==============================================================================
bool My2DantzigLCPSolver::isSymmetric(size_t _n, double* _A) {
  size_t nSkip;
  if (numBasis != 2) {
    nSkip = _n;
  } else {
    nSkip = dPAD(_n);
  }
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
  size_t nSkip;
  if (numBasis != 2) {
    nSkip = _n;
  } else {
    nSkip = dPAD(_n);
  }
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
