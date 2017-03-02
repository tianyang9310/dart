#include "LemkeLCPSolver.h"

namespace CntctLrnin {
//==============================================================================
LemkeLCPSolver::LemkeLCPSolver(double _timestep, dart::gui::SimWindow* _mWindow)
    : DantzigLCPSolver(_timestep) {
  numBasis = NUMBASIS;
  mPrecision = PRECISION;
  mWindow = _mWindow;
  numLemkeFail = 0;
  outputFileOpen();
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
    std::string trainingFile = "/tmp/CntctLrnin/lcp_data" +
                               std::to_string(numContactsToLearn) + ".csv";
    std::shared_ptr<std::fstream> outputFile =
        std::make_shared<std::fstream>(trainingFile, std::fstream::out);

    outputFile->precision(mPrecision);
    outputFiles.push_back(outputFile);
    counters.push_back(0);
  }
#endif
}

//==============================================================================
void LemkeLCPSolver::outputFileClose() {
#ifdef OUTPUT2FILE
  for (int numContactsToLearn = 1; numContactsToLearn < 5;
       numContactsToLearn++) {
    outputFiles[numContactsToLearn - 1]->close();
  }
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
  int nSkip = dPAD(n);  // YT: Don't use nSkip
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
    mu(i, i) = dynamic_cast<MyContactConstraint*>(constraint)->mFrictionCoeff;
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
  Eigen::MatrixXd T(numConstraints * (1 + numBasis),
                    numConstraints * (1 + numBasis));
  lemkeB.setZero();
  T.setZero();

  permuteNegateAugumentB(b, lemkeB, preLemkeB, T);

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
  lemkeA.setZero();

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

  // ---------------------------------------------------------------------------
  // Using Lemke to solve
  dtmsg << "# ct points: " << numContactsCallBack << std::endl;
  Eigen::VectorXd* z = new Eigen::VectorXd(numConstraints * (2 + numBasis));
  int err = dart::lcpsolver::YT::Lemke(lemkeA, lemkeB, z);

  double err_dist = 0.0;
  bool Validation =
      dart::lcpsolver::YT::validate(lemkeA, (*z), lemkeB, err_dist);

// -------------------------------------------------------------------------
// Lemke failure remedy:

#ifdef RECALL_SOLVE
  // 1. recalling Lemke to solve (sometimes effective due to randomness in
  // Lemke implementation)
  int Lemke_try = 30;
  while (!Validation && Lemke_try > 0) {
    err = dart::lcpsolver::YT::Lemke(lemkeA, lemkeB, z);
    err_dist = 0.0;
    Validation = dart::lcpsolver::YT::validate(lemkeA, (*z), lemkeB, err_dist);
    Lemke_try--;
    if (Validation) {
      std::cout << "#Find a solution after " << 30 - Lemke_try << " times try"
                << std::endl;
      break;
    }
  }
  if (Lemke_try == 0) {
    std::cout << "Recalling Lemke still cannot find a solution" << std::endl;
  }
#endif

#ifdef SNOPT_SOLVE
  // 2. Using snopt LCP to solve
  if (!Validation) {
    std::cout << "#Trying to use Snopt LCP to solve..." << std::endl;
    SnoptWrapper mSnoptLCPSolver(lemkeA, lemkeB);
    mSnoptLCPSolver.solveLCP((*z));
    err_dist = 0.0;
    Validation = dart::lcpsolver::YT::validate(lemkeA, (*z), lemkeB, err_dist);
    if (Validation) {
      std::cout << "Using Snopt LCP find a solution!" << std::endl;

      // print(lemkeA, lemkeB, (*z), Validation, err);
      // std::cin.get();
    } else {
      std::cout << "Snopt fails to find a solution either!" << std::endl;
    }
    // mWindow->keyboard('y', 0, 0);
  }
#endif

#ifdef BRUTE_SOLVE
  // 3. Using brute force to solve
  if (!Validation) {
    std::cout << "#Trying to use brute force to solve..." << std::endl;
    int dim_var = lemkeB.size();
    Eigen::VectorXd z_pattern(dim_var);
    z_pattern.setZero();
    std::vector<Eigen::VectorXd> ret_list;
    DFS(z_pattern, 0, lemkeA, lemkeB, ret_list);
    if (!ret_list.empty()) {
      std::cout << "Using brute force find a solution!" << std::endl;
      Validation = true;
    } else {
      std::cout << "Brute force fails to find a solution" << std::endl;
    }
    (*z) = ret_list[0];
    std::cout << "Solution is " << (*z).transpose() << std::endl;
    print(lemkeA, lemkeB, (*z), Validation, err);
    mWindow->keyboard('y', 0, 0);
  }
#endif

  // If fail anyway, set z as 0 to make it free from breaking
  if (!Validation) {
    numLemkeFail++;
    print(lemkeA, lemkeB, (*z), Validation, err);
    z->setZero();
    // std::cin.get();
  }
  // -------------------------------------------------------------------------

  print(lemkeA, lemkeB, (*z), Validation, err);

#ifdef IMPULSE_CHANGE
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
  std::cout << std::endl;
#endif

#ifdef SANITY_CHECK
  // -------------------------------------------------------------------------
  double SanityCheckZero = 1e-6;
  std::vector<Eigen::VectorXd> z_groups;
  decompose((*z), z_groups);
  for (int i = 0; i < numConstraints; i++) {
    // Corner case where fn==0, fd>0 and lambda>0
    if (z_groups[i][0] > -SanityCheckZero && z_groups[i][0] < SanityCheckZero &&
        z_groups[i].segment(1, numBasis).array().maxCoeff() > SanityCheckZero &&
        z_groups[i][numBasis + 1] > SanityCheckZero) {
      std::cerr << "ERROR: fn==0, fd>0 and lambda>0" << std::endl;
      std::cerr << "Lemke A is " << std::endl;
      std::cerr << lemkeA << std::endl;
      std::cerr << "Lemke b is ";
      std::cerr << lemkeB.transpose() << std::endl;
      std::cerr << "[z]" << (*z).transpose() << std::endl;
      std::cerr << "[w]" << (lemkeA * (*z) + lemkeB).transpose() << std::endl;
      std::cerr << "[z].*[w]"
                << ((*z).array() * (lemkeA * (*z) + lemkeB).array()).transpose()
                << std::endl;
      std::cerr << z_groups[i] << std::endl;
      std::cin.get();
    }
    // Corner case where fn==0, fd>0 and lambda=0
    if (z_groups[i][0] > -SanityCheckZero && z_groups[i][0] < SanityCheckZero &&
        z_groups[i].segment(1, numBasis).array().maxCoeff() > SanityCheckZero &&
        z_groups[i][numBasis + 1] > -SanityCheckZero &&
        z_groups[i][9] < SanityCheckZero) {
      std::cerr << "ERROR: fn==0, fd>0 and lambda==0" << std::endl;
      std::cerr << "Lemke A is " << std::endl;
      std::cerr << lemkeA << std::endl;
      std::cerr << "Lemke b is ";
      std::cerr << lemkeB.transpose() << std::endl;
      std::cerr << "[z]" << (*z).transpose() << std::endl;
      std::cerr << "[w]" << (lemkeA * (*z) + lemkeB).transpose() << std::endl;
      std::cerr << "[z].*[w]"
                << ((*z).array() * (lemkeA * (*z) + lemkeB).array()).transpose()
                << std::endl;
      std::cerr << z_groups[i] << std::endl;
      std::cin.get();
    }
    // Corner case where fn>0, fd=0 and lambda>0
    if (z_groups[i][0] > SanityCheckZero &&
        (z_groups[i].segment(1, numBasis).array() > -SanityCheckZero &&
         z_groups[i].segment(1, numBasis).array() < SanityCheckZero)
            .all() &&
        z_groups[i][numBasis + 1] > SanityCheckZero) {
      std::cerr << "ERROR: fn>0, fd==0 and lambda>0" << std::endl;
      std::cerr << "Lemke A is " << std::endl;
      std::cerr << lemkeA << std::endl;
      std::cerr << "Lemke b is ";
      std::cerr << lemkeB.transpose() << std::endl;
      std::cerr << "[z]" << (*z).transpose() << std::endl;
      std::cerr << "[w]" << (lemkeA * (*z) + lemkeB).transpose() << std::endl;
      std::cerr << "[z].*[w]"
                << ((*z).array() * (lemkeA * (*z) + lemkeB).array()).transpose()
                << std::endl;
      std::cerr << z_groups[i] << std::endl;
      std::cin.get();
    }
    // Count how many non-zero does fd have
    double fd_nz = 0;
    fd_nz = (z_groups[i].segment(1, numBasis).array() > SanityCheckZero)
                .matrix()
                .cast<double>()
                .sum();
    std::cout << i << "th contact has " << fd_nz << " nonzero in fd. ";
    if (z_groups[i](1 + numBasis) > SanityCheckZero) {
      std::cout << "lambda"
                << " > 0 " << std::endl;
    } else {
      std::cout << "lambda"
                << " = 0 " << std::endl;
    }
  }
#endif

  if (Validation) {
    //  ---------------------------------------
    // justify the (*z)
    // assert(!(Eigen::isnan((*z).array()).any()));

    MyContactConstraint* Mycntctconstraint;
    // (*z); N; B
    Eigen::VectorXd fn((*z).head(numConstraints));
    Eigen::VectorXd fd((*z).segment(numConstraints, numConstraints * numBasis));
    // Eigen::VectorXd lambda((*z).tail(numConstraints));

    // Using Lemke to simulate
    for (size_t idx_cnstrnt = 0; idx_cnstrnt < numConstraints; ++idx_cnstrnt) {
      double fn_each = fn(idx_cnstrnt);
      Eigen::VectorXd fd_each = fd.segment(idx_cnstrnt * numBasis, numBasis);

      Mycntctconstraint = dynamic_cast<MyContactConstraint*>(
          _group->getConstraint(idx_cnstrnt));
      Mycntctconstraint->MyapplyImpulse(fn_each, fd_each, true);

      Mycntctconstraint->excite();
    }
  } else {
    std::cout << "Lemke fails!!!" << std::endl;
    // mWindow->keyboard('y', 0, 0);
    // std::cin.get();
  }

// std::cout << std::endl;

/*
 *     for (size_t i = 0; i < numConstraints; i++) {
 *       ContactConstraint* cntctconstraint =
 *           dynamic_cast<ContactConstraint*>(_group->getConstraint(i));
 *       std::cout << "BodyNode1 new constraint impulse "
 *                 <<
 * cntctconstraint->mBodyNode1->mConstraintImpulse.transpose()
 *                 << std::endl;
 *       std::cout << "BodyNode2 new constraint impulse "
 *                 <<
 * cntctconstraint->mBodyNode2->mConstraintImpulse.transpose()
 *                 << std::endl;
 *     }
 *
 */

#ifdef IMPULSE_CHANGE
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
#endif

#ifdef OUTPUT2FILE
  // output to file after all necessary computation and valid Lemke results
  if (Validation && (counters[numConstraints - 1] < dataSize)) {
    recordLCPSolve(lemkeA, (*z), lemkeB);
  }
  // early stopping
  bool early_stopping = true;
  for (int mDCT_idx = 0; mDCT_idx < numDesiredFiles; mDCT_idx++) {
    if (counters[mDCT_idx] < dataSize) {
      early_stopping = false;
      continue;
    }
  }
  if (early_stopping) {
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
void LemkeLCPSolver::permuteNegateAugumentB(double* b, Eigen::VectorXd& lemkeB,
                                            const Eigen::VectorXd& preLemkeB,
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
    lemkeB[mIdxConstraint] = *(b + mIdxConstraint * mDim);
    T.row(mIdxConstraint) = Tcache.row(mIdxConstraint * mDim);

    lemkeB.segment(numContactsCallBack + mIdxConstraint * numBasis, numBasis) =
        Eigen::Map<Eigen::VectorXd>((b + mIdxConstraint * mDim + 1), numBasis);
    T.block(numContactsCallBack + mIdxConstraint * numBasis, 0, numBasis,
            Tcols) =
        Tcache.block(mIdxConstraint * mDim + 1, 0, numBasis, Tcols);
  }

  // // Assert transformation
  // std::cout << "T*b" << std::endl << T*preLemkeB << std::endl;
  // std::cout << "b' " << std::endl << lemkeB.head(preLemkeB.rows()) <<
  // std::endl;
  assert(T * preLemkeB == lemkeB.head(preLemkeB.rows()));

  // Negate b
  lemkeB = -lemkeB;

  // Augment b
  lemkeB.tail(numContactsCallBack).setZero();
}

//==============================================================================
void LemkeLCPSolver::permuteAugumentA(const Eigen::MatrixXd& preLemkeA,
                                      Eigen::MatrixXd& lemkeA,
                                      const Eigen::MatrixXd& T,
                                      const Eigen::MatrixXd& mu,
                                      const Eigen::MatrixXd& E) {
  size_t mDim = 1 + numBasis;
  // Permute A
  lemkeA.block(0, 0, preLemkeA.rows(), preLemkeA.cols()) =
      T * preLemkeA * T.inverse();

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
  std::cout << std::setprecision(mPrecision);
  size_t nSkip;
  if (numBasis != 2) {
    nSkip = _n;
  } else {
    nSkip = dPAD(_n);
  }
  std::cout << "A: " << std::endl;
  for (size_t i = 0; i < _n; ++i) {
    for (size_t j = 0; j < nSkip; ++j) {
      std::cout << _A[i * nSkip + j] << " ";
    }
    std::cout << std::endl;
  }

  std::cout << "b: ";
  for (size_t i = 0; i < _n; ++i) {
    std::cout << b[i] << " ";
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
  for (int i = 0; i < _n; ++i) {
    std::cout << lo[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "ub: ";
  for (int i = 0; i < _n; ++i) {
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
    std::cout << _x[i] * w[i] << " ";
  }
  std::cout << std::endl;

  delete[] Ax;
#endif
}

//==============================================================================
void LemkeLCPSolver::print(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                           const Eigen::VectorXd& z, bool Validation, int err) {
#ifdef LEMKE_PRINT
  Eigen::IOFormat CSVFmt(Eigen::FullPrecision, Eigen::DontAlignCols, ",\t");

  std::cout << std::endl
            << "```````````````````````````````````````````````" << std::endl;

  std::cout << "Matrix A " << std::endl
            << A.format(CSVFmt) << std::endl
            << std::endl;
  std::cout << "Vector b " << std::endl
            << b.transpose().format(CSVFmt) << std::endl
            << std::endl;
  std::cout << "Vector z " << std::endl
            << z.transpose().head(numContactsCallBack) << std::endl;
  for (int i = 0; i < numContactsCallBack; i++) {
    std::cout << z.transpose().segment(numContactsCallBack + i * numBasis,
                                       numBasis)
              << std::endl;
  }
  std::cout << z.transpose().tail(numContactsCallBack) << std::endl
            << std::endl;

  Eigen::VectorXd w = (A * z + b).eval();
  std::cout << "Vector w " << std::endl
            << w.transpose().head(numContactsCallBack) << std::endl;
  for (int i = 0; i < numContactsCallBack; i++) {
    std::cout << w.transpose().segment(numContactsCallBack + i * numBasis,
                                       numBasis)
              << std::endl;
  }
  std::cout << w.transpose().tail(numContactsCallBack) << std::endl
            << std::endl;

  Eigen::VectorXd wz = (z.array() * w.array()).eval();
  std::cout << "[w].*[z]" << std::endl
            << wz.transpose().head(numContactsCallBack) << std::endl;
  for (int i = 0; i < numContactsCallBack; i++) {
    std::cout << wz.transpose().segment(numContactsCallBack + i * numBasis,
                                        numBasis)
              << std::endl;
  }
  std::cout << wz.transpose().tail(numContactsCallBack) << std::endl
            << std::endl;

  std::cout << "Validation: " << std::boolalpha << Validation << std::endl;
  std::cout << "Error type: " << err << std::endl;

  std::cout << std::endl
            << "```````````````````````````````````````````````" << std::endl;
  std::cout << std::endl;

// std::cin.get();
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
                               std::vector<Eigen::VectorXd>& z_groups) {
  int numContactsToLearn = z.rows() / (numBasis + 2);
  // decompose z
  Eigen::VectorXd z_fn(numContactsToLearn);
  z_fn = z.head(numContactsToLearn);
  Eigen::VectorXd z_fd(numContactsToLearn * numBasis);
  z_fd = z.segment(numContactsToLearn, numContactsToLearn * numBasis);
  Eigen::VectorXd z_lambda(numContactsToLearn);
  z_lambda = z.tail(numContactsToLearn);

  z_groups.clear();
  for (int i = 0; i < numContactsToLearn; i++) {
    Eigen::VectorXd each_z;
    each_z.resize(numBasis + 2);
    each_z << z_fn(i), z_fd.segment(i * numBasis, numBasis), z_lambda(i);
    z_groups.push_back(each_z);
  }
}

//==============================================================================

void LemkeLCPSolver::recordLCPSolve(const Eigen::MatrixXd A,
                                    const Eigen::VectorXd z,
                                    const Eigen::VectorXd b) {
#ifdef OUTPUT2FILE
  int nSize = b.rows();
  int numContactsToLearn = nSize / (numBasis + 2);
  assert(numContactsToLearn == numContactsCallBack);

  std::shared_ptr<std::fstream> outputFile =
      outputFiles[numContactsToLearn - 1];

  // decompose z
  Eigen::VectorXd z_fn(numContactsToLearn);
  z_fn = z.head(numContactsToLearn);
  Eigen::VectorXd z_fd(numContactsToLearn * numBasis);
  z_fd = z.segment(numContactsToLearn, numContactsToLearn * numBasis);
  Eigen::VectorXd z_lambda(numContactsToLearn);
  z_lambda = z.tail(numContactsToLearn);

  double RECORD_ZERO = 1e-12;
  std::vector<Eigen::VectorXd> each_z(numContactsToLearn);
  Eigen::VectorXi value_array(numContactsToLearn);
  bool nonZerofdException = false;
  for (int i = 0; i < numContactsToLearn; i++) {
    each_z[i].resize(numBasis + 2);
    each_z[i] << z_fn(i), z_fd.segment(i * numBasis, numBasis), z_lambda(i);

    int value = 9;
    // Convention: numbasis = 8, so total 10 elements
    if (each_z[i](0) < RECORD_ZERO)  // fn = 0, break
    {
      value = 9;
    } else if (each_z[i](numBasis + 1) < RECORD_ZERO) {  // lambda = 0, static
      value = 8;
    } else {  // random choose non-zero in fd
      std::vector<int> nonZerofd;
      nonZerofd.clear();
      for (int j = 0; j < numBasis; j++) {
        if (each_z[i](j + 1) > RECORD_ZERO) {
          nonZerofd.push_back(j);
        }
      }
      //      assert(nonZerofd.size() <= 2);
      //      assert(nonZerofd.size() > 0);
      if (nonZerofd.size() > 0) {
        value = nonZerofd[std::rand() % nonZerofd.size()];
      } else {
        Eigen::IOFormat CSVFmt(Eigen::FullPrecision, Eigen::DontAlignCols,
                               ",\t");
        std::cout << "Exceptoin: catch a exception because of 0 non-zero in fd "
                     "while fn>0, lambda>0"
                  << std::endl;
        /*
         *  std::cout << "Matrix A" << std::endl << A.format(CSVFmt) <<
         * std::endl;
         *  std::cout << "Vector b" << std::endl << b.transpose().format(CSVFmt)
         * << std::endl;
         *
         *  double err_dist = 0.0;
         *  bool Validation =
         *      dart::lcpsolver::YT::validate(A, z, b, err_dist);
         *
         *  std::cout << "Vector z " << std::endl
         *            << z.transpose().head(numContactsCallBack) << std::endl;
         *  for (int i = 0; i < numContactsCallBack; i++) {
         *    std::cout << z.transpose().segment(numContactsCallBack + i *
         * numBasis,
         *                                       numBasis)
         *              << std::endl;
         *  }
         *  std::cout << z.transpose().tail(numContactsCallBack) << std::endl
         *            << std::endl;
         *
         *  Eigen::VectorXd w = (A * z + b).eval();
         *  std::cout << "Vector w " << std::endl
         *            << w.transpose().head(numContactsCallBack) << std::endl;
         *  for (int i = 0; i < numContactsCallBack; i++) {
         *    std::cout << w.transpose().segment(numContactsCallBack + i *
         * numBasis,
         *                                       numBasis)
         *              << std::endl;
         *  }
         *  std::cout << w.transpose().tail(numContactsCallBack) << std::endl
         *            << std::endl;
         *
         *  Eigen::VectorXd wz = (z.array() * w.array()).eval();
         *  std::cout << "[w].*[z]" << std::endl
         *            << wz.transpose().head(numContactsCallBack) << std::endl;
         *  for (int i = 0; i < numContactsCallBack; i++) {
         *    std::cout << wz.transpose().segment(numContactsCallBack + i *
         * numBasis,
         *                                        numBasis)
         *              << std::endl;
         *  }
         *  std::cout << wz.transpose().tail(numContactsCallBack) << std::endl
         *            << std::endl;
         *
         *  std::cout << "Validation: " << std::boolalpha << Validation <<
         * std::endl;
         */

        nonZerofdException = true;
        break;
      }
    }
    value_array(i) = value;
  }

  if (nonZerofdException) {
    return;
  }

  // if (value_array.minCoeff() >= 8) {
  if (true) {
    //  output A, z, and b
    //  // since all friction coeffs are the same, no need to output them
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
#endif
}
}
