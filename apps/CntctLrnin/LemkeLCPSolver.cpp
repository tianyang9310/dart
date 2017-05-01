#include "LemkeLCPSolver.h"

namespace CntctLrnin {
//==============================================================================
LemkeLCPSolver::LemkeLCPSolver(double _timestep, dart::gui::SimWindow* _mWindow)
    : DantzigLCPSolver(_timestep) {
  numBasis = NUMBASIS;
  mPrecision = PRECISION;
  mWindow = _mWindow;
  numLCPFail = 0;
  outputFileOpen();

#ifdef CAFFE_LCP
  // load CaffeLP solver which will load Caffe Net from ct1 to ct9
  mMaxCaffeContact = 9;
  mCaffeLPSovler = std::make_shared<CaffeLPSolver>(mMaxCaffeContact);
  CaffeSuccRate.resize(mMaxCaffeContact);
  CaffeSuccRate.setZero();
  for (int i = 0; i < mMaxCaffeContact; ++i) {
    numCaffeSucc.push_back(0);
    numTotalLCP.push_back(0);
  }
#endif
}

//==============================================================================
LemkeLCPSolver::~LemkeLCPSolver() { outputFileClose(); }

//==============================================================================
void LemkeLCPSolver::outputFileOpen() {
#ifdef OUTPUT2FILE
  dataSize = 10000;
  // Here magic number of 4 is what we suppose to get from cube testing
  numDesiredFiles = 4 * NUMBODYNODES;

  for (int numContactsToLearn = 1; numContactsToLearn < 1 + numDesiredFiles;
       numContactsToLearn++) {
    std::string dataFile = "/tmp/CntctLrnin/lcp_data" +
                           std::to_string(numContactsToLearn) + ".csv";
    std::shared_ptr<std::fstream> outputFile =
        std::make_shared<std::fstream>(dataFile, std::fstream::out);

    outputFile->precision(mPrecision);
    outputFiles.push_back(outputFile);
    counters.push_back(0);
  }
#endif

#ifdef LEMKE_PRINT
  lemkeFile = std::make_shared<std::fstream>("/tmp/CntctLrnin/Lemke.out",
                                             std::fstream::out);
  lemkeFile->precision(mPrecision);
#endif

#ifdef ODE_PRINT
  odeFile = std::make_shared<std::fstream>("/tmp/CntctLrnin/Ode.out",
                                           std::fstream::out);
  odeFile->precision(mPrecision);
#endif
}

//==============================================================================
void LemkeLCPSolver::outputFileClose() {
#ifdef OUTPUT2FILE
  for (int numContactsToLearn = 1; numContactsToLearn < 1 + numDesiredFiles;
       numContactsToLearn++) {
    outputFiles[numContactsToLearn - 1]->close();
  }
#endif

#ifdef LEMKE_PRINT
  lemkeFile->close();
#endif

#ifdef ODE_PRINT
  odeFile->close();
#endif
}

//==============================================================================
void LemkeLCPSolver::solve(ConstrainedGroup* _group) {
  if (numBasis != 2) {
    solveLemke(_group);
  } else {
    solveODE(_group);
  }
}

//==============================================================================
void LemkeLCPSolver::solveLemke(ConstrainedGroup* _group) {
  std::cout << std::setprecision(mPrecision);
  // If there is no constraint, then just return true.
  size_t numConstraints = _group->getNumConstraints();
  numContactsCallBack = numConstraints;
  if (numConstraints == 0) return;

  // Build LCP terms by aggregating them from constraints
  size_t n = _group->getTotalDimension();
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
  mu.setZero();
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
    mu(i, i) = dynamic_cast<ContactConstraint*>(constraint)->mFrictionCoeff;
    E.block(i * numBasis, i, numBasis, 1) = Eigen::VectorXd::Ones(numBasis);

    // -------------------------------------------------------------------------
    // Fill a matrix by impulse tests: A
    // Matrix A is row major
    constraint->excite();
    for (size_t j = 0; j < constraint->getDimension(); ++j) {
      // Apply impulse for mipulse test
      constraint->applyUnitImpulse(j);

      // Fill upper triangle blocks of A matrix
      int index;
      index = n * (offset[i] + j) + offset[i];

      // Don't add mCfm because it will make the solver far less accurate
      // constraint->getVelocityChange(A + index, true);
      constraint->getVelocityChange(A + index, false);
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
  }
  assert(isSymmetric(n, A));
  assert(n == numConstraints * (1 + numBasis));

  // ---------------------------------------------------------------------------
  // Establish Lemke b
  Eigen::VectorXd preLemkeB = Eigen::Map<Eigen::VectorXd>(b, n);

  Eigen::VectorXd lemkeB(numConstraints * (2 + numBasis));
  Eigen::MatrixXd T(n, n);

  permuteNegateAugumentB(b, preLemkeB, lemkeB, T);

  /*
   * // debug b
   * std::cout << "Print out b" << std::endl;
   * for (size_t tt=0; tt < n; tt++){
   *   std::cout << *(b+tt) << std::endl;
   * }
   * std::cout << "========================="<<std::endl << preLemkeB <<
   * std::endl;
   * std::cout << "========================="<<std::endl << lemkeB <<
   * std::endl;
   * std::cin.get();
   */

  // ---------------------------------------------------------------------------
  // Establish Lemke A
  Eigen::MatrixXd preLemkeA;
  preLemkeA = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      A, n, n);

  Eigen::MatrixXd lemkeA(numConstraints * (2 + numBasis),
                         numConstraints * (2 + numBasis));

  // Permute A
  permuteAugumentA(preLemkeA, lemkeA, T, mu, E);

  // scale A
  // std::cout << "Before scale: " << std::endl << lemkeA << std::endl;
  scale(lemkeA);
  // std::cout << "After scale: " << std::endl << tmpA << std::endl;

  /*
   *   // debug A
   *   std::cout << "A: " << std::endl;
   *   for (size_t i = 0; i < n; ++i) {
   *     for (size_t j = 0; j < n; ++j) {
   *       std::cout << std::setprecision(mPrecision) << A[i * n + j] << " ";
   *     }
   *     std::cout << std::endl;
   *   }
   *
   *   std::cout << "========================="<<std::endl << preLemkeA <<
   * std::endl;
   *   std::cout << "========================="<<std::endl << lemkeA <<
   * std::endl;
   *   std::cin.get();
   */

  int err = 0;
  bool Validation = false;
  Eigen::VectorXd* z = new Eigen::VectorXd(numConstraints * (2 + numBasis));

#ifdef CAFFE_LCP
  if (numConstraints <= mMaxCaffeContact /* && numConstraints >0 */) {
    numTotalLCP[numConstraints - 1] = numTotalLCP[numConstraints - 1] + 1;
    mCaffeLPSovler->solve(numConstraints, lemkeA, lemkeB, (*z));
    Validation = dart::lcpsolver::YT::validate(lemkeA, lemkeB, (*z));
    if (Validation) {
      numCaffeSucc[numConstraints - 1] = numCaffeSucc[numConstraints - 1] + 1;
    }
    CaffeSuccRate[numConstraints - 1] =
        static_cast<double>(numCaffeSucc[numConstraints - 1]) /
        numTotalLCP[numConstraints - 1];
    LOG(ERROR) << "Caffe success rate: " << CaffeSuccRate.transpose()
               << std::endl;
  }
#endif
  
  // ---------------------------------------------------------------------------
  // Using Lemke to solve
  // dtmsg << "# ct points: " << numContactsCallBack << std::endl;

  if (!Validation) {
    err = dart::lcpsolver::YT::Lemke(lemkeA, lemkeB, z);
    Validation = dart::lcpsolver::YT::validate(lemkeA, lemkeB, (*z));
  }

  // -------------------------------------------------------------------------
  // Lemke failure remedy and sanity check
  if (!Validation) {
    recallLemke(Validation, lemkeA, lemkeB, z);
  }

  if (!Validation) {
    useSnoptLCPSolver(Validation, lemkeA, lemkeB, z);
  }

  if (!Validation) {
    bruteForce(Validation, lemkeA, lemkeB, z);
  }

  // If fail anyway, set z as 0 to make it free from breaking
  if (!Validation) {
    numLCPFail++;
    print(lemkeA, lemkeB, (*z), Validation, err);
    z->setZero();
    // std::cin.get();
  }

  sanityCheck(lemkeA, lemkeB, (*z));

  // -------------------------------------------------------------------------
  print(lemkeA, lemkeB, (*z), Validation, err);

  if (Validation) {
    //  ---------------------------------------
    // justify the (*z)
    assert(!(Eigen::isnan((*z).array()).any()));

    MyContactConstraint* Mycntctconstraint;

    Eigen::VectorXd fn((*z).head(numConstraints));
    Eigen::VectorXd fd((*z).segment(numConstraints, numConstraints * numBasis));
    // Eigen::VectorXd lambda((*z).tail(numConstraints));

    // Using Lemke to simulate
    for (size_t mIdxCnstrnt = 0; mIdxCnstrnt < numConstraints; ++mIdxCnstrnt) {
      double eachFn = fn(mIdxCnstrnt);
      Eigen::VectorXd eachFd = fd.segment(mIdxCnstrnt * numBasis, numBasis);

      Mycntctconstraint = dynamic_cast<MyContactConstraint*>(
          _group->getConstraint(mIdxCnstrnt));
      Mycntctconstraint->MyapplyImpulse(eachFn, eachFd, true);

      Mycntctconstraint->excite();
    }
  } else {
    dterr << "Lemke fails!!!" << std::endl;
    // std::cin.get();
  }

#ifdef OUTPUT2FILE
  // output to file after all necessary computation and valid Lemke results
  if (Validation && (counters[numConstraints - 1] < dataSize)) {
    recordLCPSolve(lemkeA, lemkeB, (*z));
  }

  // early stopping
  bool earlyStopping = true;
  for (int mIdxFile = 0; mIdxFile < numDesiredFiles; mIdxFile++) {
    if (counters[mIdxFile] < dataSize) {
      earlyStopping = false;
      break;
    }
  }

  if (earlyStopping) {
    exit(0);
  }
#endif

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
void LemkeLCPSolver::recallLemke(bool& Validation,
                                 const Eigen::MatrixXd& lemkeA,
                                 const Eigen::VectorXd& lemkeB,
                                 Eigen::VectorXd* z) {
#ifdef RECALLSOLVE
  // recalling Lemke to solve (sometimes effective due to randomness in
  // Lemke implementation)
  dtmsg << "# Trying to recall Lemke to solve..." << std::endl;
  int Lemke_try = 30;
  while (!Validation && Lemke_try > 0) {
    int err = dart::lcpsolver::YT::Lemke(lemkeA, lemkeB, z);
    Validation = dart::lcpsolver::YT::validate(lemkeA, lemkeB, (*z));
    Lemke_try--;
    if (Validation) {
      dtmsg << "$ Finding a solution after " << 30 - Lemke_try
            << " times retry..." << std::endl;
      break;
    }
  }
  if (Lemke_try == 0 && !Validation) {
    dtmsg << "@ Recalling Lemke still cannot find a solution..." << std::endl;
  }
#endif
}

//==============================================================================
void LemkeLCPSolver::useSnoptLCPSolver(bool& Validation,
                                       const Eigen::MatrixXd& lemkeA,
                                       const Eigen::VectorXd& lemkeB,
                                       Eigen::VectorXd* z) {
#ifdef SNOPTSOLVE
  // Using snopt LCP to solve
  dtmsg << "# Trying to use Snopt LCP solver to solve..." << std::endl;
  SnoptWrapper mSnoptLCPSolver(lemkeA, lemkeB);
  mSnoptLCPSolver.solveLCP((*z));
  Validation = dart::lcpsolver::YT::validate(lemkeA, lemkeB, (*z));
  if (Validation) {
    dtmsg << "$ Finding a solution via Snopt LCP solver..." << std::endl;
    // print(lemkeA, lemkeB, (*z), Validation, err);
    // std::cin.get();
  } else {
    dtmsg << "@ Snopt stil fails to find a solution either!" << std::endl;
  }
#endif
}

//==============================================================================
void LemkeLCPSolver::bruteForce(bool& Validation, const Eigen::MatrixXd& lemkeA,
                                const Eigen::VectorXd& lemkeB,
                                Eigen::VectorXd* z) {
#ifdef BRUTESOLVE
  // Using brute force to solve
  dtmsg << "# Trying to use brute force to solve..." << std::endl;
  int dim_var = lemkeB.size();
  Eigen::VectorXd zPattern(dim_var);
  zPattern.setZero();
  std::vector<Eigen::VectorXd> ret_list;
  ret_list.clear();
  DFS(zPattern, 0, lemkeA, lemkeB, ret_list);
  if (!ret_list.empty()) {
    dtmsg << "$ Finding a solution via brute force..." << std::endl;
    Validation = true;
    (*z) = ret_list[0];
    // std::cout << "Solution is " << (*z).transpose() << std::endl;
    // print(lemkeA, lemkeB, (*z), Validation, err);
  } else {
    dtmsg << "@ Brute force still fails to find a solution..." << std::endl;
  }
#endif
}

//==============================================================================
void LemkeLCPSolver::sanityCheck(const Eigen::MatrixXd& lemkeA,
                                 const Eigen::VectorXd& lemkeB,
                                 const Eigen::VectorXd& z) {
#ifdef SANITY_CHECK
  double SanityCheckZero = 1e-6;

  std::vector<Eigen::VectorXd> zGroups;
  decompose(z, zGroups);

  for (int i = 0; i < numContactsCallBack; i++) {
    Eigen::VectorXd eachZ = zGroups[i];
    // -------------------------------------------------------------------------
    // Corner case where fn==0, fd>0 and lambda>0
    if (std::abs(eachZ[0]) < SanityCheckZero &&
        eachZ.segment(1, numBasis).maxCoeff() > SanityCheckZero &&
        eachZ[numBasis + 1] > SanityCheckZero) {
      dterr << std::endl
            << "ERROR: fn==0, fd>0 and lambda>0" << std::endl
            << "Lemke A is " << std::endl
            << lemkeA << std::endl
            << "Lemke b is " << lemkeB.transpose() << std::endl
            << "[z]" << z.transpose() << std::endl
            << "[w]" << (lemkeA * z + lemkeB).transpose() << std::endl
            << "[z].*[w]"
            << (z.array() * (lemkeA * z + lemkeB).array()).transpose()
            << std::endl
            << "Error z: " << eachZ << std::endl;
      // std::cin.get();
    }
    // -------------------------------------------------------------------------
    // Corner case where fn==0, fd>0 and lambda=0
    if (std::abs(eachZ[0]) < SanityCheckZero &&
        eachZ.segment(1, numBasis).maxCoeff() > SanityCheckZero &&
        std::abs(eachZ[numBasis + 1]) < SanityCheckZero) {
      dterr << std::endl
            << "ERROR: fn==0, fd>0 and lambda==0" << std::endl
            << "Lemke A is " << std::endl
            << lemkeA << std::endl
            << "Lemke b is " << lemkeB.transpose() << std::endl
            << "[z]" << z.transpose() << std::endl
            << "[w]" << (lemkeA * z + lemkeB).transpose() << std::endl
            << "[z].*[w]"
            << (z.array() * (lemkeA * z + lemkeB).array()).transpose()
            << std::endl
            << "Error z: " << eachZ << std::endl;
      // std::cin.get();
    }
    // -------------------------------------------------------------------------
    // Corner case where fn>0, fd=0 and lambda>0
    if (eachZ[0] > SanityCheckZero &&
        (eachZ.segment(1, numBasis).array() > -SanityCheckZero &&
         eachZ.segment(1, numBasis).array() < SanityCheckZero)
            .all() &&
        eachZ[numBasis + 1] > SanityCheckZero) {
      dterr << std::endl
            << "ERROR: fn>0, fd==0 and lambda>0" << std::endl
            << "Lemke A is " << std::endl
            << lemkeA << std::endl
            << "Lemke b is " << lemkeB.transpose() << std::endl
            << "[z]" << z.transpose() << std::endl
            << "[w]" << (lemkeA * z + lemkeB).transpose() << std::endl
            << "[z].*[w]"
            << (z.array() * (lemkeA * z + lemkeB).array()).transpose()
            << std::endl
            << "Error z: " << eachZ << std::endl;
      // std::cin.get();
    }
// -------------------------------------------------------------------------
// Count how many non-zero does fd have
#ifdef COUNT_NONZERO
    double fdNz = 0;
    fdNz = (eachZ.segment(1, numBasis).array() > SanityCheckZero)
               .matrix()
               .cast<double>()
               .sum();
    dtmsg << i << "th contact has " << fdNz << " nonzero in fd. ";
    if (eachZ(1 + numBasis) > SanityCheckZero) {
      std::cout << "lambda"
                << " > 0 " << std::endl;
    } else {
      std::cout << "lambda"
                << " = 0 " << std::endl;
    }
#endif
  }
#endif
}

//==============================================================================
void LemkeLCPSolver::solveODE(ConstrainedGroup* _group) {
  std::cout << std::setprecision(mPrecision);
  // If there is no constraint, then just return true.
  size_t numConstraints = _group->getNumConstraints();
  numContactsCallBack = numConstraints;
  if (numConstraints == 0) return;

  // Build LCP terms by aggregating them from constraints
  size_t n = _group->getTotalDimension();
  int nSkip = dPAD(n);
  double* A = new double[n * nSkip];
  double* x = new double[n];
  double* b = new double[n];
  double* w = new double[n];
  double* lo = new double[n];
  double* hi = new double[n];
  int* findex = new int[n];

// Set w to 0 and findex to -1
#ifndef NDEBUG
  std::memset(A, 0.0, n * nSkip * sizeof(double));
#endif
  std::memset(w, 0.0, n * sizeof(double));
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
  for (size_t i = 0; i < numConstraints; ++i) {
    constraint = _group->getConstraint(i);

    constInfo.x = x + offset[i];
    constInfo.lo = lo + offset[i];
    constInfo.hi = hi + offset[i];
    constInfo.b = b + offset[i];
    constInfo.findex = findex + offset[i];
    constInfo.w = w + offset[i];

    // Fill vectors: lo, hi, b, w
    constraint->getInformation(&constInfo);

    // Fill a matrix by impulse tests: A
    constraint->excite();
    for (size_t j = 0; j < constraint->getDimension(); ++j) {
      // Adjust findex for global index
      if (findex[offset[i] + j] >= 0) findex[offset[i] + j] += offset[i];

      // Apply impulse for mipulse test
      constraint->applyUnitImpulse(j);

      // Fill upper triangle blocks of A matrix
      int index = nSkip * (offset[i] + j) + offset[i];
      constraint->getVelocityChange(A + index, true);
      for (size_t k = i + 1; k < numConstraints; ++k) {
        index = nSkip * (offset[i] + j) + offset[k];
        _group->getConstraint(k)->getVelocityChange(A + index, false);
      }

      // Filling symmetric part of A matrix
      for (size_t k = 0; k < i; ++k) {
        for (size_t l = 0; l < _group->getConstraint(k)->getDimension(); ++l) {
          int index1 = nSkip * (offset[i] + j) + offset[k] + l;
          int index2 = nSkip * (offset[k] + l) + offset[i] + j;

          A[index1] = A[index2];
        }
      }
    }

    assert(isSymmetric(n, A, offset[i],
                       offset[i] + constraint->getDimension() - 1));

    constraint->unexcite();
  }

  assert(isSymmetric(n, A));

  double* oldA = new double[n * nSkip];
  double* oldB = new double[n];
  for (int i = 0; i < n * nSkip; i++) oldA[i] = A[i];
  for (int i = 0; i < n; i++) oldB[i] = b[i];

  // Print LCP formulation
  dtdbg << "Before solve:" << std::endl;
  print(n, A, x, lo, hi, b, w, findex);
  std::cout << std::endl;

  // Solve LCP using ODE's Dantzig algorithm
  dSolveLCP(n, A, x, b, w, 0, lo, hi, findex);

  // Print LCP formulation
  dtdbg << "After solve:" << std::endl;
  print(n, oldA, x, lo, hi, oldB, w, findex);
  std::cout << std::endl;

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
void LemkeLCPSolver::permuteNegateAugumentB(double* b,
                                            const Eigen::VectorXd& preLemkeB,
                                            Eigen::VectorXd& lemkeB,
                                            Eigen::MatrixXd& T) {
  lemkeB.setZero();
  T.setZero();
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
    lemkeB[mIdxConstraint] = *(b + mIdxConstraint * mDim);
    T.row(mIdxConstraint) = Tcache.row(mIdxConstraint * mDim);

    lemkeB.segment(numContactsCallBack + mIdxConstraint * numBasis, numBasis) =
        Eigen::Map<Eigen::VectorXd>((b + mIdxConstraint * mDim + 1), numBasis);
    T.block(numContactsCallBack + mIdxConstraint * numBasis, 0, numBasis,
            Tcols) =
        Tcache.block(mIdxConstraint * mDim + 1, 0, numBasis, Tcols);
  }

  assert(T * preLemkeB == lemkeB.head(preLemkeB.rows()));
  assert(std::abs(T.determinant()) == 1.0);

  // Negate b
  lemkeB = -lemkeB;

  // Augment b
  lemkeB.tail(numContactsCallBack) = Eigen::VectorXd::Zero(numContactsCallBack);
}

//==============================================================================
void LemkeLCPSolver::permuteAugumentA(const Eigen::MatrixXd& preLemkeA,
                                      Eigen::MatrixXd& lemkeA,
                                      const Eigen::MatrixXd& T,
                                      const Eigen::MatrixXd& mu,
                                      const Eigen::MatrixXd& E) {
  lemkeA.setZero();
  size_t mDim = 1 + numBasis;
  assert(mDim * numContactsCallBack == preLemkeA.rows());
  assert(mDim * numContactsCallBack == preLemkeA.cols());

  assert(T * T.transpose() == Eigen::MatrixXd::Identity(preLemkeA.rows(), preLemkeA.cols()));

  // Permute A
  lemkeA.block(0, 0, preLemkeA.rows(), preLemkeA.cols()) =
      T * preLemkeA * T.transpose();

  // Augment A
  // _TimeStep =  1.0 means using calculating impulse in Lemke
  // otherwise _TimeStep  = mTimeStep means calculating force in Lemke
  double _TimeStep = 1.0;  // mTimeStep;
  lemkeA.block(numContactsCallBack * mDim, 0, numContactsCallBack,
               numContactsCallBack) = mu / (_TimeStep == 1.0 ? mTimeStep : 1.0);
  lemkeA.block(numContactsCallBack * mDim, numContactsCallBack,
               numContactsCallBack, numContactsCallBack * numBasis) =
      -E.transpose() / (_TimeStep == 1.0 ? mTimeStep : 1.0);
  lemkeA.block(numContactsCallBack, numContactsCallBack * mDim,
               numContactsCallBack * numBasis, numContactsCallBack) =
      E / (_TimeStep == 1.0 ? mTimeStep : 1.0);
}

//==============================================================================
void LemkeLCPSolver::print(size_t _n, double* _A, double* _x, double* lo,
                           double* hi, double* b, double* w, int* findex) {
#ifdef ODE_PRINT
  (*odeFile) << std::setprecision(mPrecision);
  size_t nSkip;
  if (numBasis != 2) {
    nSkip = _n;
  } else {
    nSkip = dPAD(_n);
  }
  (*odeFile) << "A: " << std::endl;
  for (size_t i = 0; i < _n; ++i) {
    for (size_t j = 0; j < nSkip; ++j) {
      (*odeFile) << _A[i * nSkip + j] << " ";
    }
    (*odeFile) << std::endl;
  }

  (*odeFile) << "b: ";
  for (size_t i = 0; i < _n; ++i) {
    (*odeFile) << b[i] << " ";
  }
  (*odeFile) << std::endl;

  (*odeFile) << "w: ";
  for (size_t i = 0; i < _n; ++i) {
    (*odeFile) << w[i] << " ";
  }
  (*odeFile) << std::endl;

  (*odeFile) << "x: ";
  for (size_t i = 0; i < _n; ++i) {
    (*odeFile) << _x[i] << " ";
  }
  (*odeFile) << std::endl;

  (*odeFile) << "lb: ";
  for (int i = 0; i < _n; ++i) {
    (*odeFile) << lo[i] << " ";
  }
  (*odeFile) << std::endl;

  (*odeFile) << "ub: ";
  for (int i = 0; i < _n; ++i) {
    (*odeFile) << hi[i] << " ";
  }
  (*odeFile) << std::endl;

  // (*odeFile) << "frictionIndex: ";
  // for (size_t i = 0; i < _n; ++i)
  // {
  //     (*odeFile) << findex[i] << " ";
  // }
  // (*odeFile) << std::endl;

  double* Ax = new double[_n];

  for (size_t i = 0; i < _n; ++i) {
    Ax[i] = 0.0;
  }

  for (size_t i = 0; i < _n; ++i) {
    for (size_t j = 0; j < _n; ++j) {
      Ax[i] += _A[i * nSkip + j] * _x[j];
    }
  }

  (*odeFile) << "Ax   : ";
  for (size_t i = 0; i < _n; ++i) {
    (*odeFile) << Ax[i] << " ";
  }
  (*odeFile) << std::endl;

  (*odeFile) << "b + w: ";
  for (size_t i = 0; i < _n; ++i) {
    (*odeFile) << b[i] + w[i] << " ";
  }
  (*odeFile) << std::endl;

  (*odeFile) << "x^T * w: ";
  for (size_t i = 0; i < _n; ++i) {
    (*odeFile) << _x[i] * w[i] << " ";
  }
  (*odeFile) << std::endl;

  delete[] Ax;
#endif
}

//==============================================================================
void LemkeLCPSolver::print(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                           const Eigen::VectorXd& z, const bool Validation,
                           const int err) {
#ifdef LEMKE_PRINT
  Eigen::IOFormat CSVFmt(Eigen::FullPrecision, Eigen::DontAlignCols, ",\t");

  (*lemkeFile) << std::endl
               << "```````````````````````````````````````````````"
               << std::endl;

  (*lemkeFile) << "Matrix A " << std::endl
               << A.format(CSVFmt) << std::endl
               << std::endl;
  (*lemkeFile) << "Vector b " << std::endl
               << b.transpose().format(CSVFmt) << std::endl
               << std::endl;

  (*lemkeFile) << "Vector z " << std::endl
               << z.transpose().head(numContactsCallBack) << std::endl;
  for (int i = 0; i < numContactsCallBack; i++) {
    (*lemkeFile) << z.transpose().segment(numContactsCallBack + i * numBasis,
                                          numBasis)
                 << std::endl;
  }
  (*lemkeFile) << z.transpose().tail(numContactsCallBack) << std::endl
               << std::endl;

  Eigen::VectorXd w = (A * z + b).eval();
  (*lemkeFile) << "Vector w " << std::endl
               << w.transpose().head(numContactsCallBack) << std::endl;
  for (int i = 0; i < numContactsCallBack; i++) {
    (*lemkeFile) << w.transpose().segment(numContactsCallBack + i * numBasis,
                                          numBasis)
                 << std::endl;
  }
  (*lemkeFile) << w.transpose().tail(numContactsCallBack) << std::endl
               << std::endl;

  Eigen::VectorXd wz = (z.array() * w.array()).eval();
  (*lemkeFile) << "[w].*[z]" << std::endl
               << wz.transpose().head(numContactsCallBack) << std::endl;
  for (int i = 0; i < numContactsCallBack; i++) {
    (*lemkeFile) << wz.transpose().segment(numContactsCallBack + i * numBasis,
                                           numBasis)
                 << std::endl;
  }
  (*lemkeFile) << wz.transpose().tail(numContactsCallBack) << std::endl
               << std::endl;

  (*lemkeFile) << "Validation: " << std::boolalpha << Validation << std::endl;
  (*lemkeFile) << "Error type: " << err << std::endl;

  (*lemkeFile) << std::endl
               << "```````````````````````````````````````````````"
               << std::endl;
  (*lemkeFile) << std::endl;
#endif
}

//==============================================================================
bool LemkeLCPSolver::isSymmetric(size_t _n, double* _A) {
  std::cout << std::setprecision(mPrecision);
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
            std::cout << _A[k * nSkip + l] << " ";
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
bool LemkeLCPSolver::isSymmetric(size_t _n, double* _A, size_t _begin,
                                 size_t _end) {
  std::cout << std::setprecision(mPrecision);
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
            std::cout << _A[k * nSkip + l] << " ";
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
void LemkeLCPSolver::scale(Eigen::MatrixXd& A) {
  int numRow = A.rows();
  int numCol = A.cols();

  assert(numRow == numCol);
  assert(numContactsCallBack * (2 + numBasis) == numRow);

  /*
   * double MYCFM = 1e-9;
   * // adding eps to diagonal terms
   * for (size_t i = 0; i < numRow; i++) {
   *   A(i,i) = (1 + MYCFM) * A(i,i);
   * }
   */

  // scale mu and E
  int mDim = 1 + numBasis;
  double h = 4e-3;

  A.block(numContactsCallBack * mDim, 0, numContactsCallBack,
          numContactsCallBack) *= h;
  A.block(numContactsCallBack * mDim, numContactsCallBack, numContactsCallBack,
          numContactsCallBack * numBasis) *= h;
  A.block(numContactsCallBack, numContactsCallBack * mDim,
          numContactsCallBack * numBasis, numContactsCallBack) *= h;
}

//==============================================================================
void LemkeLCPSolver::decompose(const Eigen::VectorXd& z,
                               std::vector<Eigen::VectorXd>& zGroups) {
  int numContactsToLearn = z.rows() / (numBasis + 2);
  assert(numContactsToLearn == numContactsCallBack);

  // decompose z
  Eigen::VectorXd zFn(numContactsToLearn);
  zFn = z.head(numContactsToLearn);

  Eigen::VectorXd zFd(numContactsToLearn * numBasis);
  zFd = z.segment(numContactsToLearn, numContactsToLearn * numBasis);

  Eigen::VectorXd zLambda(numContactsToLearn);
  zLambda = z.tail(numContactsToLearn);

  zGroups.clear();
  for (int i = 0; i < numContactsToLearn; i++) {
    Eigen::VectorXd eachZ(numBasis + 2);
    eachZ << zFn(i), zFd.segment(i * numBasis, numBasis), zLambda(i);
    zGroups.push_back(eachZ);
  }
}

//==============================================================================
void LemkeLCPSolver::recordLCPSolve(const Eigen::MatrixXd& A,
                                    const Eigen::VectorXd& b,
                                    const Eigen::VectorXd& z) {
#ifdef OUTPUT2FILE
  int nSize = b.rows();
  int numContactsToLearn = nSize / (numBasis + 2);
  assert(numContactsToLearn == numContactsCallBack);

  std::shared_ptr<std::fstream> outputFile =
      outputFiles[numContactsToLearn - 1];

  std::vector<Eigen::VectorXd> zGroups;
  decompose(z, zGroups);

  double RECORD_ZERO = 1e-12;
  Eigen::VectorXi value_array(numContactsToLearn);
  bool nonZerofdException = false;

  for (int i = 0; i < numContactsToLearn; i++) {
    Eigen::VectorXd eachZ = zGroups[i];

    int value = 9;
    // Convention: numbasis = 8, so total 10 elements
    if (eachZ(0) < RECORD_ZERO)  // fn = 0, break
    {
      value = 9;
    } else if (eachZ(numBasis + 1) < RECORD_ZERO) {  // lambda = 0, static
      value = 8;
    } else {  // random choose non-zero in fd

      // Hard-coding for numBasis = 4
      assert(numBasis == 4);
      // eachZ [1, numBasis]
      std::vector<int> nonZerofd;
      nonZerofd.clear();
      for (int j = 0; j < numBasis; j++) {
        if (eachZ(j + 1) > RECORD_ZERO) {
          nonZerofd.push_back(j);
        }
      }

      if (nonZerofd.size() == 0 || nonZerofd.size() > 2) {
        dterr << "ERROR: the number of non-zeros in fd is wrong..."
              << std::endl;
        nonZerofdException = true;
        break;
      }

      if (nonZerofd.size() == 2) {
        if (nonZerofd[0] == 0 && nonZerofd[1] == 3) {
          nonZerofd[0] = 4;
        }
      }

      int sum = 0;
      for (int j = 0; j < nonZerofd.size(); j++) {
        sum = sum + nonZerofd[j];
      }

      value = sum * 2 / nonZerofd.size();
    }
    value_array(i) = value;
  }

  if (nonZerofdException) {
    return;
  }

  // std::cout << "Matrix A" << std::endl << A << std::endl;
  // std::cout << "Vector b" << std::endl << b.transpose() << std::endl;
  // std::cout << "Vector z" << std::endl << z.transpose() << std::endl;
  // for (int i = 0; i < numContactsToLearn; i++) {
  //   std::cout << "Value " << i << " :" << value_array(i) << std::endl;
  // }

  bool recordAllClasses = false;
  if (recordAllClasses) {
    // -------------------------------------------------------------------------
    // record all classes for all contact points
    // if (value_array.minCoeff() >= 8) {
    if (true) {
      for (int i = 0; i < nSize - numContactsToLearn; i++) {
        for (int j = i; j < nSize - numContactsToLearn; j++) {
          (*outputFile) << A(i, j) << ",";
        }
      }

      for (int i = 0; i < numContactsToLearn; i++) {
        (*outputFile) << A(nSize - numContactsToLearn + i, i) << ",";
      }

      for (int i = 0; i < nSize - numContactsToLearn; i++) {
        (*outputFile) << b(i) << ",";
      }

      for (int i = 0; i < numContactsToLearn; i++) {
        (*outputFile) << value_array(i);
        if (i < numContactsToLearn - 1) {
          (*outputFile) << ",";
        }
      }

      (*outputFile) << std::endl;
      counters[numContactsToLearn - 1] += 1;
    }
  } else {
    // -------------------------------------------------------------------------
    // record one class each time and permute A and b
    for (int mIdxCnstrnt = 0; mIdxCnstrnt < numContactsCallBack;
         ++mIdxCnstrnt) {
      Eigen::MatrixXd newA(nSize, nSize);
      Eigen::VectorXd newb(nSize);
      newA = A;
      newb = b;
      if (mIdxCnstrnt > 0) {
        permuteAandBforRecord(newA, newb, A, b, 0, mIdxCnstrnt);

        /*
        // assert
        Eigen::VectorXd* newZ = new Eigen::VectorXd(numContactsCallBack * (2 +
        numBasis)); int newErr = dart::lcpsolver::YT::Lemke(newA, newb, newZ);
        bool newValidation = dart::lcpsolver::YT::validate(newA, newb, (*newZ));
        std::vector<Eigen::VectorXd> newZGroups;
        decompose((*newZ), newZGroups);

        int newValue = 9;
        // Convention: numbasis = 8, so total 10 elements
        if (newZGroups[0](0) < RECORD_ZERO)  // fn = 0, break
        {
          newValue = 9;
        } else if (newZGroups[0](numBasis + 1) < RECORD_ZERO) {  // lambda = 0,
        static newValue = 8; } else {  // random choose non-zero in fd

          // Hard-coding for numBasis = 4
          assert(numBasis == 4);
          // newZGroups[0] [1, numBasis]
          std::vector<int> nonZerofd;
          nonZerofd.clear();
          for (int j = 0; j < numBasis; j++) {
            if (newZGroups[0](j + 1) > RECORD_ZERO) {
              nonZerofd.push_back(j);
            }
          }

          if (nonZerofd.size() == 0 || nonZerofd.size() > 2) {
            dterr << "ERROR: the number of non-zeros in fd is wrong..."
                  << std::endl;
            nonZerofdException = true;
            break;
          }

          if (nonZerofd.size() == 2) {
            if (nonZerofd[0] == 0 && nonZerofd[1] == 3) {
              nonZerofd[0] = 4;
            }
          }

          int sum = 0;
          for (int j = 0; j < nonZerofd.size(); j++) {
            sum = sum + nonZerofd[j];
          }

          newValue = sum * 2 / nonZerofd.size();
        }

        // std::cout << "Old Matrix A: " << std::endl << A << std::endl;
        // std::cout << "Old vector b: " << std::endl << b.transpose() <<
        std::endl;
        // std::cout << "Old vector z: " << std::endl << z.transpose() <<
        std::endl;
        // std::cout << "New Matrix A: " << std::endl << newA << std::endl;
        // std::cout << "New vector b: " << std::endl << newb.transpose() <<
        std::endl;
        // std::cout << "New vector z: " << std::endl << (*newZ).transpose() <<
        std::endl; std::cout << "Expected first contact class: " <<
        value_array(mIdxCnstrnt) << std::endl; std::cout << "new contact class:
        " << newValue << std::endl; std::cin.get();
        */
      }

      // if (value_array.minCoeff() >= 8) {
      if (true) {
        for (int i = 0; i < nSize - numContactsToLearn; i++) {
          for (int j = i; j < nSize - numContactsToLearn; j++) {
            (*outputFile) << newA(i, j) << ",";
          }
        }

        for (int i = 0; i < numContactsToLearn; i++) {
          (*outputFile) << newA(nSize - numContactsToLearn + i, i) << ",";
        }

        for (int i = 0; i < nSize - numContactsToLearn; i++) {
          (*outputFile) << newb(i) << ",";
        }

        (*outputFile) << value_array(mIdxCnstrnt);

        (*outputFile) << std::endl;
      }
    }
    counters[numContactsToLearn - 1] += 1;
  }
#endif
}

//==============================================================================
void LemkeLCPSolver::permuteAandBforRecord(Eigen::MatrixXd& newA,
                                           Eigen::VectorXd& newb,
                                           const Eigen::MatrixXd& A,
                                           const Eigen::VectorXd& b, int idx0,
                                           int idx1) {
  assert(newA.size() == A.size());
  assert(newb.size() == b.size());
  assert(newb.size() == (numBasis + 2) * numContactsCallBack);
  assert(idx0 >= 0 && idx0 < numContactsCallBack);
  assert(idx1 >= 0 && idx1 < numContactsCallBack);

  int mDim = numBasis + 1;
  Eigen::MatrixXd T(mDim * numContactsCallBack, mDim * numContactsCallBack);
  T.setIdentity();

  T.row(idx0).swap(T.row(idx1));
  T.block(numContactsCallBack + idx0 * numBasis, 0, numBasis,
          mDim * numContactsCallBack)
      .swap(T.block(numContactsCallBack + idx1 * numBasis, 0, numBasis,
                    mDim * numContactsCallBack));
  assert(T * T.transpose() == Eigen::MatrixXd::Identity(T.rows(), T.cols()));
  newA.topLeftCorner(T.rows(), T.cols()) =
      T * A.topLeftCorner(T.rows(), T.cols()) * T.transpose();
  newb.head(T.rows()) = T * b.head(T.rows());

  // permute friction coefficients
  T.resize(numContactsCallBack, numContactsCallBack);
  T.setIdentity();
  T.row(idx0).swap(T.row(idx1));
  assert(T * T.transpose() == Eigen::MatrixXd::Identity(T.rows(), T.cols()));
  newA.bottomLeftCorner(numContactsCallBack, numContactsCallBack) =
      T * A.bottomLeftCorner(numContactsCallBack, numContactsCallBack) *
      T.transpose();
}
}  // namespace CntctLrnin
