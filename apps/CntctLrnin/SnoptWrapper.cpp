#include "SnoptWrapper.h"

SnoptWrapper::SnoptWrapper(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
    : mA(A), mb(b) {
  dim_var = mA.cols();
  dim_cnst = mA.rows();
  assert(dim_var == dim_cnst);
}

void SnoptWrapper::solveLCP(Eigen::VectorXd& z) {
  // negate b because ```ret = ((mA * x - mB).array() * x.array()).matrix();```
  for (size_t i = 0; i < dim_var; i++) {
    mb(i) = -mb(i);
  }
  z.resize(dim_var);
  z.setZero();

  // Use SnoptLP to solve a LP problem, where A*z+b=w, z>=0, w>=0, z^T*w=0
  QPCCProblem problem(dim_var, dim_cnst, mA, mb);
  SnoptSolver solver(&problem);
  solver.solve();

  for (size_t i = 0; i < dim_var; i++) {
    z(i) = problem.vars()[i]->mVal;
  }
  // std::cout << z.transpose() << std::endl;
}

void SnoptWrapper::solveLP(Eigen::VectorXd& z, bool bInitGuess) {
  // No need to negate because ```return mA * x - mB;```
  if (!bInitGuess) {
    z.resize(dim_var);
    z.setZero();
  }  // else (provide init guess)

  // Use SnoptLP to solve a LP problem, where A*x = b, x>=0
  std::shared_ptr<SnoptLPproblem> problem;

  if (!bInitGuess) {
    problem = std::make_shared<SnoptLPproblem>(dim_var, dim_cnst, mA, mb);
  } else {
    problem = std::make_shared<SnoptLPproblem>(dim_var, dim_cnst, mA, mb, &z);
  }

  SnoptSolver solver(problem.get());

  solver.solve();

  for (size_t i = 0; i < dim_var; i++) {
    z(i) = problem->vars()[i]->mVal;
  }
  // std::cout << z.transpose() << std::endl;
}

void SnoptWrapper::solveLPFullPiv(Eigen::VectorXd& z) {
  // Using Eigen Library's built-in Full Pivoting method to solve linear
  // equations
  z.resize(dim_var);
  z.setZero();

  // --------------------------------------------------------------------------
  // A*x = b
  z = mA.fullPivLu().solve(mb);
  // if (((mA*z - mb).array().abs() > - 1e-12).all()) {
  //   // std::cout << "dim_var: " << dim_var << std::endl;
  //   // std::cout << "mA: " << mA << std::endl;
  //   // std::cout << "mb: " << mb.transpose() << std::endl;
  //   // std::cout << "z: " << z.transpose() << std::endl;
  //   // std::cout << "err: " << (mA*z-mb).transpose() << std::endl;

  //   // std::cin.get();
  // } else {
  //   // std::cout << "mA: " << mA << std::endl;
  //   // std::cout << "mb: " << mb.transpose() << std::endl;
  //   // std::cout << "z: " << z.transpose() << std::endl;
  //   // std::cout << "err: " << (mA*z-mb).transpose() << std::endl;

  //   // std::cin.get();
  // }

  // --------------------------------------------------------------------------
  // Use null space basis to get solution of x>=0
  double LPFULLPIV_ZERO = 1e-6;
  assert(LPFULLPIV_ZERO == LCPLS_ZERO);

  if (z.minCoeff() < -LPFULLPIV_ZERO) {
    Eigen::FullPivLU<Eigen::MatrixXd> lu(mA);
    Eigen::MatrixXd ANullSpace = lu.kernel();
    std::cout << "LP mA: " << mA << std::endl;
    std::cout << "LP mb: " << mb.transpose() << std::endl;
    std::cout << "LP z: " << z.transpose() << std::endl;
    std::cout << "LP err: " << (mA * z - mb).transpose() << std::endl;
    // std::cout << "LU threshold: " << lu.threshold() << std::endl;
    // std::cout << "kernel: " << std::endl << ANullSpace << std::endl;
    // std::cout << "A*kenrel: " << std::endl << mA * ANullSpace << std::endl;

    // std::cin.get();

    // Gram-Schmidt orthogonalization
    Eigen::MatrixXd ANullSpaceOrthogonal = ANullSpace;
    for (int i = 1; i < ANullSpace.cols(); ++i) {
      Eigen::VectorXd ANullSpaceOrthogonalCol = ANullSpaceOrthogonal.col(i);
      for (int j = 0; j < i; ++j) {
        ANullSpaceOrthogonalCol =
            ANullSpaceOrthogonalCol -
            (ANullSpaceOrthogonal.col(j).dot(ANullSpace.col(i))) /
                (ANullSpaceOrthogonal.col(j).dot(ANullSpaceOrthogonal.col(j))) *
                ANullSpaceOrthogonal.col(j);
      }
      ANullSpaceOrthogonal.col(i) = ANullSpaceOrthogonalCol;
    }
    ANullSpaceOrthogonal.colwise().normalize();
    std::cout << "kernel orthogonalization and normalization: " << std::endl
              << ANullSpaceOrthogonal << std::endl;

    Eigen::VectorXd kelCoeff =
        -(z.transpose() * ANullSpaceOrthogonal).transpose();

    std::cout << "kelCoeff: " << std::endl << kelCoeff.transpose() << std::endl;
    z = z + ANullSpaceOrthogonal * kelCoeff;
    std::cout << "After offset: " << std::endl << z.transpose() << std::endl;

    std::vector<Eigen::VectorXd> CPos;
    std::vector<Eigen::VectorXd> CNeg;
    std::vector<Eigen::VectorXd> CPosNeg;
    double BASIS_ZERO = 1e-12;
    for (int i = 0; i < ANullSpaceOrthogonal.cols(); ++i) {
      if (ANullSpaceOrthogonal.col(i).minCoeff() >= -BASIS_ZERO) {
        CPos.push_back(ANullSpaceOrthogonal.col(i));
      } else if (ANullSpaceOrthogonal.col(i).maxCoeff() <= BASIS_ZERO) {
        CNeg.push_back(ANullSpaceOrthogonal.col(i));
      } else {
        CPosNeg.push_back(ANullSpaceOrthogonal.col(i));
      }
    }

    std::cout << "CPos size: " << CPos.size() << std::endl;
    std::cout << "CNeg size: " << CNeg.size() << std::endl;
    std::cout << "CPosNeg size: " << CPosNeg.size() << std::endl;
    for (auto CPosNegEach : CPosNeg) {
      std::cout << CPosNegEach.transpose() << std::endl;
    }

    for (int i = 0; i < dim_var; ++i) {
      if (z(i) > -LPFULLPIV_ZERO) {
        continue;
      }

      // using CPos to offset
      for (auto CPosEach : CPos) {
        if (CPosEach(i) > 0) {
          z = z + CPosEach * (-z(i) / CPosEach(i));
        }
      }

      // using CNeg to offset
      for (auto CNegEach : CNeg) {
        if (CNegEach(i) < 0) {
          z = z + CNegEach * (-z(i) / CNegEach(i));
        }
      }

      // using CPosNeg to offset
    }

    std::cout << "After offset2: " << std::endl << z.transpose() << std::endl;

    if (z.minCoeff() < -LPFULLPIV_ZERO) {
      std::cin.get();
    }

    // if (z.minCoeff() < -LPFULLPIV_ZERO) {
    //   for (int i = 0; i < dim_var; ++i)
    //   {
    //     if (z(i) > -LPFULLPIV_ZERO) {
    //       continue;
    //     }

    //     int nIdx = 0;
    //     double offset_abs = ANullSpace.row(i).array().abs().maxCoeff(&nIdx);
    //     if (offset_abs < LPFULLPIV_ZERO) {
    //       break;
    //     }

    //     double offset = ANullSpace(i, nIdx);

    //     Eigen::VectorXd tmpZ = z + ANullSpace.col(nIdx) * (-z(i)/offset);
    //     if (tmpZ.minCoeff() > -LPFULLPIV_ZERO) {
    //       z = tmpZ;
    //     }
    //   }
    // }
  }
  // if (z.minCoeff() < - LPFULLPIV_ZERO) {
  //   Eigen::FullPivLU<Eigen::MatrixXd> lu(mA);
  //   Eigen::MatrixXd ANullSpace = lu.kernel();
  //   std::cout << "LP mA: " << mA << std::endl;
  //   std::cout << "LP mb: " << mb.transpose() << std::endl;
  //   std::cout << "LP z: " << z.transpose() << std::endl;
  //   std::cout << "LP err: " << (mA*z-mb).transpose() << std::endl;
  //   std::cout << "LU threshold: " << lu.threshold() << std::endl;
  //   std::cout << "kernel: " << std::endl << ANullSpace << std::endl;
  //   std::cout << "A*kenrel: " << std::endl << mA * ANullSpace << std::endl;
  //   std::cin.get();
  // }
}

void SnoptWrapper::solveQP(const Eigen::VectorXd& z0, Eigen::VectorXd& z) {
  z.resize(dim_var);
  z.setZero();

  // Use SnoptQP to solve a QP problem, where
  // min 0.5*z'*2A*z + z'*b
  // st. z>=0, A*z - (-b) >=0
  SnoptQPproblem problem(dim_var, dim_cnst, mA, mb, z0);
  SnoptSolver solver(&problem);
  solver.solve();

  for (size_t i = 0; i < dim_var; i++) {
    z(i) = problem.vars()[i]->mVal;
  }
}

inline int findPivotCol(const Eigen::MatrixXd& tableau) {
  double piv_tol = 1e-10;
  int nSize = tableau.rows() - 1;

  Eigen::VectorXd objRow = tableau.row(0).transpose().segment(1, 2 * nSize);
  int pivotCol;
  double highest = objRow.maxCoeff(&pivotCol);
  pivotCol = pivotCol + 1;
  if (highest <= piv_tol) {
    pivotCol = -1;
  }
  // std::cout << "objRow: " << objRow.transpose() << std::endl;
  // std::cout << "highest: " << highest << " objRow[highest]: " <<
  // objRow(pivotCol-1) << " pivot Col: " << pivotCol << " ";
  return pivotCol;
}

inline int findPivotRow(const Eigen::MatrixXd& tableau, int pivotCol) {
  double zer_tol = 1e-6;
  double piv_tol = 1e-10;

  int nSize = tableau.rows() - 1;

  int pivotRow = -1;
  double minRatio = 1e20;
  for (int i = 1; i < tableau.rows(); ++i) {
    if (tableau(i, pivotCol) > 0) {
      double ratio =
          (tableau(i, 2 * nSize + 1) + zer_tol) / tableau(i, pivotCol);
      if (ratio < minRatio) {
        minRatio = ratio;
        pivotRow = i;
      }
    }
  }
  // std::cout << "minRatio: " << minRatio << " pivot Row: " << pivotRow << " ";
  return pivotRow;
}

inline void pivotOn(Eigen::MatrixXd& tableau, int pivotCol, int pivotRow) {
  // std::cout << "pivotRow: " << pivotRow << " pivotCol: " << pivotCol <<
  // std::endl; std::cout << "Before pivoting..." << std::endl << tableau <<
  // std::endl;
  tableau.row(pivotRow) = tableau.row(pivotRow) / tableau(pivotRow, pivotCol);

  for (int i = 0; i < tableau.rows(); ++i) {
    if (i != pivotRow) {
      tableau.row(i) =
          tableau.row(i) - tableau.row(pivotRow) * tableau(i, pivotCol);
    }
  }
  // std::cout << "After pivoting..." << std::endl << tableau << std::endl;
  // std::cin.get();
}

void SnoptWrapper::solveLPBFS(const Eigen::MatrixXd& A,
                              const Eigen::VectorXd& b, Eigen::VectorXd& x) {
  Eigen::MatrixXd LPBFS_A = A;
  Eigen::VectorXd LPBFS_b = b;
  assert(A.rows() == b.rows());
  int nSize = A.rows();

  // negate all negative rhs constraints
  for (int i = 0; i < nSize; ++i) {
    if (LPBFS_b(i) < 0) {
      LPBFS_A.row(i) = -LPBFS_A.row(i);
      LPBFS_b(i) = -LPBFS_b(i);
    }
  }

  Eigen::MatrixXd tableau(nSize + 1, 2 * (nSize + 1));
  tableau.setZero();

  Eigen::VectorXd objRow(2 * (nSize + 1));
  objRow.setZero();
  objRow(0) = 1;
  objRow.segment(1, nSize) = LPBFS_A.colwise().sum();
  objRow(2 * nSize + 1) = LPBFS_b.sum();
  tableau.row(0) = objRow.transpose();

  tableau.block(1, 1, nSize, nSize) = LPBFS_A;
  tableau.block(1, nSize + 1, nSize, nSize) =
      Eigen::MatrixXd::Identity(nSize, nSize);
  tableau.block(1, 2 * nSize + 1, nSize, 1) = LPBFS_b;

  int nMaxIter = static_cast<int>(1e3);

  x.resize(nSize);
  x.setZero();
  Eigen::VectorXi xIdx = Eigen::VectorXi::Constant(nSize, -1);
  Eigen::VectorXd y = Eigen::VectorXd::Zero(nSize);
  Eigen::VectorXi yIdx = Eigen::VectorXi::Zero(nSize);

  for (int i = 0; i < nSize; ++i) {
    yIdx(i) = i + 1;
  }

  for (int idx = 0; idx < nMaxIter; ++idx) {
    int pivotCol, pivotRow;
    pivotCol = findPivotCol(tableau);
    if (pivotCol < 0) {
      // std::cout << "Simplex converge..." << std::endl;
      break;
    }

    pivotRow = findPivotRow(tableau, pivotCol);
    if (pivotRow < 0) {
      // std::cout << "Unbounded constraint limit..." << std::endl;
      break;
    }

    pivotOn(tableau, pivotCol, pivotRow);
    // std::cout << "Pivot row: " << pivotRow << " Pivot Col: " << pivotCol <<
    // std::endl; std::cout << "tableau: " << std::endl << tableau << std::endl;
    if (pivotCol < nSize + 1) {
      for (int idx_XYidx = 0; idx_XYidx < nSize; ++idx_XYidx) {
        if (xIdx(idx_XYidx) == pivotRow) {
          xIdx(idx_XYidx) = -1;
          break;
        }
        if (yIdx(idx_XYidx) == pivotRow) {
          yIdx(idx_XYidx) = -1;
          break;
        }
      }
      xIdx(pivotCol - 1) = pivotRow;
    } else {
      for (int idx_XYidx = 0; idx_XYidx < nSize; ++idx_XYidx) {
        if (xIdx(idx_XYidx) == pivotRow) {
          xIdx(idx_XYidx) = -1;
          break;
        }
        if (yIdx(idx_XYidx) == pivotRow) {
          yIdx(idx_XYidx) = -1;
          break;
        }
      }
      yIdx(pivotCol - nSize - 1) = pivotRow;
    }
  }

  for (int i = 0; i < nSize; ++i) {
    if (xIdx(i) != -1) {
      x(i) = tableau(xIdx(i), 2 * nSize + 1);
    }
    if (yIdx(i) != -1) {
      y(i) = tableau(yIdx(i), 2 * nSize + 1);
    }
  }

  // std::cout << "tableau: " << std::endl << tableau << std::endl;
  // std::cout << "A: " << std::endl << A << std::endl;
  // std::cout << "LPBFS_A: " << std::endl << LPBFS_A << std::endl;
  // std::cout << "b: " << std::endl << b.transpose() << std::endl;
  // std::cout << "LPBFS_b: " << std::endl << LPBFS_b.transpose() << std::endl;
  // std::cout << "xIdx: " << std::endl << xIdx.transpose() << std::endl;
  // std::cout << "x: " << std::endl << x.transpose() << std::endl;
  // std::cout << "yIdx: " << std::endl << yIdx.transpose() << std::endl;
  // std::cout << "y: " << std::endl << y.transpose() << std::endl;
  // std::cout << "err: " << (A*x-b).transpose() << std::endl;

  // std::cin.get();
}
