#include "MyDantzigLCPSolver.h"
#include "MyWindow.h"

#define LEMKE_OUTPUT  // Lemke A, b, err, z, w
#define LEMKE_OUTPUT_DETAILS  // Lemke N, B, Skeletons velocities, M, E
#define LEMKE_FAIL_PRINT  // print lemke fail prompt information
#define ODE_OUTPUT        // ODE A, b, x, w (true if ALWAYS_ODE or !validation)
// #define OUTPUT2FILE  // output data to file

#define ALWAYS_ODE         // always solve ode whether Lemke is valid or not
#define LEMKE_APPLY_PRINT  // print applying Lemke constraint
#define ODE_APPLY_PRINT    // print applying ODE constraint

// #define CLAMP_DANTZIG

/*
 * * fn=0, fd=0, lambda=0		contact break
 * * fn=0, fd=0, lambda>0		has relative tangential velocities but no friction
 * * fn=0, fd>0, lambda=0 	X
 * * fn=0, fd>0, lambda>0		X
 * * fn>0, fd=0, lambda=0		static, no relative tangential velocities, no relative tangential acc
 * * fn>0, fd=0, lambda>0		X
 * * fn>0, fd>0, lambda=0		static friction, no relative tangential velocities, relative tangential acc
 * * fn>0, fd>0, lambda>0		slide
 */

MyDantzigLCPSolver::MyDantzigLCPSolver(double _timestep, int _totalDOF,
                                       MyWindow* mWindow)
    : DantzigLCPSolver(_timestep), totalDOF(_totalDOF), mWindow(mWindow) {
  numBasis = 8;
  dataSize = 30000;
  LemkeFailCounter = 0;

#ifdef OUTPUT2FILE
  for (int numContactsToLearn = 1; numContactsToLearn < 5;
       numContactsToLearn++) {
    std::string trainingFile =
        "/tmp/lcp_data" + std::to_string(numContactsToLearn) + ".csv";
    std::shared_ptr<std::fstream> outputFile =
        std::make_shared<std::fstream>(trainingFile, std::fstream::out);
    outputFile->precision(10);
    outputFiles.push_back(outputFile);
    counters.push_back(0);
  }
#endif

  ODE_FILE = std::make_shared<std::fstream>("/tmp/ODE.txt", std::fstream::out);
  ODE_FILE->precision(10);
  Lemke_FILE =
      std::make_shared<std::fstream>("/tmp/Lemke.txt", std::fstream::out);
  Lemke_FILE->precision(10);
}

MyDantzigLCPSolver::~MyDantzigLCPSolver() {
  for (int numContactsToLearn = 1; numContactsToLearn < 5;
       numContactsToLearn++) {
    outputFiles[numContactsToLearn - 1]->close();
  }

  ODE_FILE->close();
  Lemke_FILE->close();
}

void MyDantzigLCPSolver::solve(ConstrainedGroup* _group) {
  // If there is no constraint, then just return true.
  // numConstraints is exactly the number of contact points
  size_t numConstraints = _group->getNumConstraints();
  numContactsCallBack = numConstraints;
  if (numConstraints == 0) {
    return;
  } else {
#ifdef LEMKE_OUTPUT
    (*Lemke_FILE) << "There are " << numConstraints << " contact points"
                  << std::endl;
#endif
  }

  // Build LCP terms by aggregating them from constraints
  // n = 3*numConstraints if mIsFrictionOn else numConstraints
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

  // Compute offset indices. If there are more than 1 contact points,
  // like say exactly 2 contact points, then we need these indices to access
  // information for 2nd contact point
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
      if (findex[offset[i] + j] >= 0) {
        // std::cout<<"Adject findex for global index"<<std::endl;
        findex[offset[i] + j] += offset[i];
        // std::cin.get();
      }

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

    // assert(isSymmetric(n, A, offset[i],
    //                    offset[i] + constraint->getDimension() - 1));

    constraint->unexcite();
  }

  // assert(isSymmetric(n, A));

  // Print LCP formulation
  // std::cout << "Before solve:" << std::endl;
  // print(n, A, x, lo, hi, b, w, findex);
  // std::cout << std::endl;

  //  ---------------------------------------
  // establish A and b matrix for Lemke algorithm
  ContactConstraint* cntctconstraint;
  Eigen::MatrixXd M(Eigen::MatrixXd::Identity(totalDOF, totalDOF));
  Eigen::MatrixXd N(Eigen::MatrixXd::Zero(totalDOF, numConstraints));
  Eigen::MatrixXd B(Eigen::MatrixXd::Zero(totalDOF, numConstraints * numBasis));
  Eigen::MatrixXd mu(Eigen::MatrixXd::Zero(numConstraints, numConstraints));
  Eigen::MatrixXd E(
      Eigen::MatrixXd::Zero(numConstraints * numBasis, numConstraints));
  Eigen::VectorXd tau(Eigen::VectorXd::Zero(totalDOF));

  // Convention: bodyNode1 ==> cube, bodyNode2 ==> ground
  dart::dynamics::BodyNodePtr bodyNode1 = nullptr;
  dart::dynamics::BodyNodePtr bodyNode2 = nullptr;
  int BodyNode1_dim = -1;
  int BodyNode2_dim = -1;
  Eigen::MatrixXd M1;
  Eigen::MatrixXd M2;
  Eigen::VectorXd tau1;
  Eigen::VectorXd tau2;
  dart::math::LinearJacobian J1;
  dart::math::LinearJacobian J2;
  for (size_t idx_cnstrnt = 0; idx_cnstrnt < numConstraints; idx_cnstrnt++) {
    cntctconstraint =
        dynamic_cast<ContactConstraint*>(_group->getConstraint(idx_cnstrnt));
    if (cntctconstraint->mContacts.size() > 1) {
      std::cerr << "ERROR: more than one contact for current ContactConstraint"
                << std::endl;
      std::cerr << "There should always be one contact point for one contact "
                   "constraint"
                << std::endl;
      std::cin.get();
    }

    // Here check the convention and make sure that once bodyNode1 and bodyNode2
    // have been chosen,
    // they will not be changed during the following iterations
    dart::collision::Contact* ct = cntctconstraint->mContacts[0];
    if (bodyNode1 == nullptr) {
      bodyNode1 = cntctconstraint->mBodyNode1;
    } else if (bodyNode1 != cntctconstraint->mBodyNode1) {
      std::cerr << "ERROR: violating convention that bodyNode1 always is cube"
                << std::endl;
      std::cin.get();
    }
    if (bodyNode2 == nullptr) {
      bodyNode2 = cntctconstraint->mBodyNode2;
    } else if (bodyNode2 != cntctconstraint->mBodyNode2) {
      std::cerr << "ERROR: violating convention that bodyNode2 always is ground"
                << std::endl;
      std::cin.get();
    }

    // Body point location in the local frame
    Eigen::Vector3d bodyPoint1;
    Eigen::Vector3d bodyPoint2;
    bodyPoint1.noalias() =
        bodyNode1->getTransform().inverse() * ct->point;
    bodyPoint2.noalias() =
        bodyNode2->getTransform().inverse() * ct->point;

// -----------------------------------------------------------------------------
    // Here only care about normal force and friction force, there is no
    // torques, thus only Linear Jacobian

    /*
     * J1 = bodyNode1->getLinearJacobian(bodyPoint1);
     * J2 = bodyNode2->getLinearJacobian(bodyPoint2);
     */

    J1 = bodyNode1->getLinearJacobian(bodyPoint1,bodyNode1);
    J2 = bodyNode2->getLinearJacobian(bodyPoint2,bodyNode2);

    assert(J1.rows() == J2.rows());
// -----------------------------------------------------------------------------
    // Determine DOF belongs to each body node, again here we are going to
    // assume the convention
    if (BodyNode1_dim == -1) {
      BodyNode1_dim = J1.cols();
    } else if (BodyNode1_dim != J1.cols()) {
      std::cerr << "ERROR: violating convention that bodyNode1 always is cube"
                << std::endl;
      std::cin.get();
    }
    if (BodyNode2_dim == -1) {
      BodyNode2_dim = J2.cols();
    } else if (BodyNode2_dim != J2.cols()) {
      std::cerr << "ERROR: violating convention that bodyNode2 always is ground"
                << std::endl;
      std::cin.get();
    }

// -----------------------------------------------------------------------------
    // normDirection ==> global
    // normDirection1 ==> bodyNode1 frame
    // normDirection2 ==> bodyNode2 frame
    Eigen::Vector3d normDirection;
    Eigen::Vector3d normDirection1;
    Eigen::Vector3d normDirection2;
    normDirection = ct->normal;

    // normal direction should also be in the world frame
    normDirection1 = ct->normal;
    normDirection2 = - ct->normal;

    /*
     * normDirection1 = bodyNode1->getTransform().linear().transpose() * ct->normal;
     * normDirection2 = - bodyNode2->getTransform().linear().transpose() * ct->normal;
     */
// -----------------------------------------------------------------------------

#ifdef LEMKE_OUTPUT_DETAILS
    (*Lemke_FILE) << "Contact "<<idx_cnstrnt<< " bodyNode1 transform: "<< std::endl << bodyNode1->getTransform().matrix() <<std::endl;
    (*Lemke_FILE) << "Contact "<<idx_cnstrnt<< " artificial Jacobian is "<< std::endl << bodyNode1->getLinearJacobian(Eigen::Vector3d(0.05,-0.05,0.05), bodyNode1) <<std::endl;
    (*Lemke_FILE) << "Contact "<<idx_cnstrnt<< " ct->point is "<< std::endl << ct->point.transpose() <<std::endl;
    (*Lemke_FILE) << "Contact "<<idx_cnstrnt<< " bodypoint1 is "<< std::endl << bodyPoint1.transpose() <<std::endl;
    (*Lemke_FILE) << "Contact "<<idx_cnstrnt<< " Jacobian1 is "<< std::endl << J1 <<std::endl;
    (*Lemke_FILE) << "Contact "<<idx_cnstrnt<< " Jacobian2 is " << std::endl << J2 <<std::endl;
    (*Lemke_FILE) << "Contact "<<idx_cnstrnt<< " World Jacobian1 is "<< std::endl << bodyNode1->getLinearJacobian(bodyPoint1) <<std::endl;
    (*Lemke_FILE) << "Contact "<<idx_cnstrnt<< " World Jacobian2 is " << std::endl << bodyNode2->getLinearJacobian(bodyPoint2) <<std::endl;
#endif

#ifdef LEMKE_OUTPUT_DETAILS
    (*Lemke_FILE) <<"Contact "<<idx_cnstrnt<<" normal direction is "<<normDirection.transpose()<<std::endl;
#endif
    N.col(idx_cnstrnt) << J1.transpose() * normDirection1,
        J2.transpose() * normDirection2;
#ifdef LEMKE_OUTPUT_DETAILS
    (*Lemke_FILE) <<"Contact "<<idx_cnstrnt<<" N is "<<N.transpose()<<std::endl;
#endif

// -----------------------------------------------------------------------------
    // B matrix
    Eigen::MatrixXd D(3, numBasis);
    Eigen::MatrixXd D1(3, numBasis);
    Eigen::MatrixXd D2(3, numBasis);
    D = getTangentBasisMatrixLemke(normDirection, numBasis);

    // tangential direction in the world frame
    D1 = D;
    D2 = -D;

    /*
     * D1 = bodyNode1->getTransform().linear().transpose() * D;
     * D2 =  - bodyNode1->getTransform().linear().transpose() * D;
     */

// -----------------------------------------------------------------------------
#ifdef LEMKE_OUTPUT_DETAILS
    (*Lemke_FILE) <<"Contact "<<idx_cnstrnt<<" D matrix is "<<std::endl<<D<<std::endl;
#endif
    B.block(0, idx_cnstrnt * numBasis, totalDOF, numBasis)
        << J1.transpose() * D1,
        J2.transpose() * D2;

    // friction coeff
    // std::cout<<"The friction coeff for the two BodyNodes are: ";
    // std::cout<<bodyNode1->getFrictionCoeff()<<"   ";
    // std::cout<<bodyNode2->getFrictionCoeff()<<std::endl;
    mu(idx_cnstrnt, idx_cnstrnt) =
        std::min(bodyNode1->getFrictionCoeff(),
                 bodyNode2->getFrictionCoeff());

    // Ones matrix
    E.block(idx_cnstrnt * numBasis, idx_cnstrnt, numBasis, 1) =
        Eigen::VectorXd::Ones(numBasis);
  }
#ifdef LEMKE_OUTPUT_DETAILS
  (*Lemke_FILE) << "mBodyNode1 Jacobian" << std::endl << J1 <<std::endl;
  (*Lemke_FILE) << "mBodyNode2 Jacobian" << std::endl << J2 <<std::endl;
#endif

  // Convention check
  if (bodyNode1 != nullptr) {
    M1 = bodyNode1->getSkeleton()->getMassMatrix();
    // std::cout << "mass: " << std::endl;
    // Eigen::Matrix3d R =  ((dart::dynamics::FreeJoint*)bodyNode1->getParentJoint())->convertToTransform(bodyNode1->getSkeleton()->getPositions()).linear();
    // std::cout<< "Before" <<std::endl;
    // std::cout << M1 <<std::endl;
    // M1.block(0,0,3,3) = R.transpose() * M1.block(0,0,3,3) * R;
    // std::cout <<((dart::dynamics::FreeJoint*)bodyNode1->getParentJoint())->convertToTransform(bodyNode1->getSkeleton()->getPositions()).linear() <<std::endl;
    // std::cout<< "After" <<std::endl;
    // std::cout << M1 <<std::endl;
    M.block(0, 0, M1.rows(), M1.cols()) = M1;

// -----------------------------------------------------------------------------
// It doesn't matter because internal forces are always zero if setForces is
// never called
    // get internal forces
    Eigen::VectorXd mInternalForces(bodyNode1->getSkeleton()->getNumDofs());
    mInternalForces.setZero();
    int dofCounter = 0;
    for (size_t idxJoint = 0;
         idxJoint < bodyNode1->getSkeleton()->getNumJoints(); idxJoint++) {
      int DofofThisJoint =
          bodyNode1->getSkeleton()->getJoint(idxJoint)->getNumDofs();
      mInternalForces.segment(dofCounter, DofofThisJoint) =
          bodyNode1->getSkeleton()->getJoint(idxJoint)->getForces();
      dofCounter += DofofThisJoint;
    }
    /*
     * std::cout << "Internal forces are " << mInternalForces.transpose()
     *          << std::endl;
     */
// -----------------------------------------------------------------------------

    // get external forces
    Eigen::VectorXd mExternalForces(bodyNode1->getSkeleton()->getNumDofs());
    mExternalForces = bodyNode1->getSkeleton()->getExternalForces();
    //  mExternalForces = bodyNode1->getExternalForceLocal();

    /*
     * std::cout << "mBox external forces are: "
     *          << mExternalForces.transpose() - bodyNode1->getSkeleton()->getExternalForces().transpose()
     *          << std::endl;
     */

    // print out Coriolis and Gravity forces
    /*
     *std::cout
     *    << "Coriolis and gravity forces are "
     *    << bodyNode1->getSkeleton()->getCoriolisAndGravityForces().transpose()
     *    << std::endl;
     */

    // print out tau1
    /*
     *std::cout << "current velocity is "
     *          << mSkeletonVelocitiesLock[bodyNode1->getSkeleton()].transpose()
     *          << std::endl;
     *std::cout << "[No ExtForces] tau1 is "
     *          << (M1 * mSkeletonVelocitiesLock[bodyNode1->getSkeleton()] -
     *              mTimeStep *
     *                  (bodyNode1->getSkeleton()->getCoriolisAndGravityForces()
     *-
     *                   mInternalForces))
     *                 .transpose()
     *          << std::endl;
     *std::cout << "[Full] tau1 is "
     *          << (M1 * mSkeletonVelocitiesLock[bodyNode1->getSkeleton()] -
     *              mTimeStep *
     *                  (bodyNode1->getSkeleton()->getCoriolisAndGravityForces()
     *-
     *                   mInternalForces - mExternalForces))
     *                 .transpose()
     *          << std::endl;
     */

    // Disable external force when computing tau, in this case this should be
    // exactly as the equation of LCP paper
    // mExternalForces.setZero();

    tau1 =
        M1 * mSkeletonVelocitiesLock[bodyNode1->getSkeleton()] -
        mTimeStep * (bodyNode1->getSkeleton()->getCoriolisAndGravityForces() -
                     mInternalForces - mExternalForces);
    tau.head(tau1.rows()) = tau1;
  } else {
    std::cerr << "ERROR: violating convention that bodyNode1 always is cube"
              << std::endl;
    std::cin.get();
  }
  if (bodyNode2 != nullptr) {
    M2 = bodyNode2->getSkeleton()->getMassMatrix();
    M.block(M1.rows(), M1.cols(), M2.rows(), M2.cols()) = M2;
    // get internal forces
    Eigen::VectorXd mInternalForces(bodyNode2->getSkeleton()->getNumDofs());
    mInternalForces.setZero();
    int dofCounter = 0;
    for (size_t idxJoint = 0;
         idxJoint < bodyNode2->getSkeleton()->getNumJoints(); idxJoint++) {
      int DofofThisJoint =
          bodyNode2->getSkeleton()->getJoint(idxJoint)->getNumDofs();
      mInternalForces.segment(dofCounter, DofofThisJoint) =
          bodyNode2->getSkeleton()->getJoint(idxJoint)->getForces();
      dofCounter += DofofThisJoint;
    }
    /*
     *std::cout << "Internal forces are " << mInternalForces.transpose()
     *          << std::endl;
     */

    // get external forces
    Eigen::VectorXd mExternalForces(bodyNode2->getSkeleton()->getNumDofs());
    mExternalForces = bodyNode2->getSkeleton()->getExternalForces();
    /*
     *std::cout << "mBox external forces are: "
     *          << bodyNode2->getSkeleton()->getExternalForces().transpose()
     *          << std::endl;
     */

    tau2 =
        M2 * mSkeletonVelocitiesLock[bodyNode2->getSkeleton()] -
        mTimeStep * (bodyNode2->getSkeleton()->getCoriolisAndGravityForces() -
                     mInternalForces - mExternalForces);
    tau.tail(tau2.rows()) = tau2;
  } else {
    std::cerr << "ERROR: violating convention that bodyNode2 always is ground"
              << std::endl;
    std::cin.get();
  }

  // _TimeStep =  1.0 means using calculating impulse in Lemke
  // otherwise _TimeStep  = mTimeStep means calculating force in Lemke
  double _TimeStep = 1.0;  // mTimeStep;
  // using ldlt().solve() since mass matrix is positive definite
  Eigen::MatrixXd Lemke_A(Eigen::MatrixXd::Zero(
      numConstraints * (2 + numBasis), numConstraints * (2 + numBasis)));
  Lemke_A.block(0, 0, numConstraints, numConstraints) =
      _TimeStep * N.transpose() * M.ldlt().solve(N);
  Lemke_A.block(0, numConstraints, numConstraints, numConstraints * numBasis) =
      _TimeStep * N.transpose() * M.ldlt().solve(B);
  Lemke_A.block(numConstraints, 0, numConstraints * numBasis, numConstraints) =
      _TimeStep * B.transpose() * M.ldlt().solve(N);
  Lemke_A.block(numConstraints, numConstraints, numConstraints * numBasis,
                numConstraints * numBasis) =
      _TimeStep * B.transpose() * M.ldlt().solve(B);
  Lemke_A.block(numConstraints * (numBasis + 1), 0, numConstraints,
                numConstraints) = mu / (_TimeStep == 1.0 ? mTimeStep : 1.0);
  Lemke_A.block(numConstraints * (numBasis + 1), numConstraints, numConstraints,
                numConstraints * numBasis) =
      -E.transpose() / (_TimeStep == 1.0 ? mTimeStep : 1.0);
  Lemke_A.block(numConstraints, numConstraints * (numBasis + 1),
                numConstraints * numBasis, numConstraints) =
      E / (_TimeStep == 1.0 ? mTimeStep : 1.0);

  Eigen::VectorXd Lemke_b(
      Eigen::VectorXd::Zero(numConstraints * (2 + numBasis)));
  Lemke_b.head(numConstraints) = N.transpose() * M.ldlt().solve(tau);
  Lemke_b.segment(numConstraints, numConstraints * numBasis) =
      B.transpose() * M.ldlt().solve(tau);

#ifdef LEMKE_OUTPUT
  (*Lemke_FILE) << "^^^^^^Lemke Preparation ^^^^^" << std::endl;
  (*Lemke_FILE) << "Lemke A is " << std::endl;
  (*Lemke_FILE) << Lemke_A << std::endl;
  (*Lemke_FILE) << "Lemke b is ";
  (*Lemke_FILE) << Lemke_b.transpose() << std::endl;
  (*Lemke_FILE) << "Solving LCP with Lemke" << std::endl;
#endif
  Eigen::VectorXd* z = new Eigen::VectorXd(numConstraints * (2 + numBasis));
  int err = dart::lcpsolver::YT::Lemke(Lemke_A, Lemke_b, z);

  // ---------------------------------------------------------------------------
  double CornerZero = 1e-16;
  std::vector<Eigen::VectorXd> z_groups;
  decompose((*z), z_groups);
  for (int i = 0; i < numConstraints; i++) {
    // Corner case where fn==0, fd>0 and lambda>0
    if (z_groups[i][0] > -CornerZero && z_groups[i][0] < CornerZero &&
        z_groups[i].segment(1, numBasis).array().maxCoeff() > CornerZero &&
        z_groups[i][9] > CornerZero) {
      std::cerr << "ERROR: fn==0, fd>0 and lambda>0" << std::endl;
      (*Lemke_FILE) << "Lemke A is " << std::endl;
      (*Lemke_FILE) << Lemke_A << std::endl;
      (*Lemke_FILE) << "Lemke b is ";
      (*Lemke_FILE) << Lemke_b.transpose() << std::endl;
      (*Lemke_FILE) << "[z]" << (*z).transpose() << std::endl;
      (*Lemke_FILE) << "[w]" << (Lemke_A * (*z) + Lemke_b).transpose()
                    << std::endl;
      (*Lemke_FILE)
          << "[z].*[w]"
          << ((*z).array() * (Lemke_A * (*z) + Lemke_b).array()).transpose()
          << std::endl;
      std::cerr << z_groups[i] << std::endl;
      std::cin.get();
    }
    // Corner case where fn==0, fd>0 and lambda=0
    if (z_groups[i][0] > -CornerZero && z_groups[i][0] < CornerZero &&
        z_groups[i].segment(1, numBasis).array().maxCoeff() > CornerZero &&
        z_groups[i][9] > -CornerZero && z_groups[i][9] < CornerZero) {
      std::cerr << "ERROR: fn==0, fd>0 and lambda==0" << std::endl;
      (*Lemke_FILE) << "Lemke A is " << std::endl;
      (*Lemke_FILE) << Lemke_A << std::endl;
      (*Lemke_FILE) << "Lemke b is ";
      (*Lemke_FILE) << Lemke_b.transpose() << std::endl;
      (*Lemke_FILE) << "[z]" << (*z).transpose() << std::endl;
      (*Lemke_FILE) << "[w]" << (Lemke_A * (*z) + Lemke_b).transpose()
                    << std::endl;
      (*Lemke_FILE)
          << "[z].*[w]"
          << ((*z).array() * (Lemke_A * (*z) + Lemke_b).array()).transpose()
          << std::endl;
      std::cerr << z_groups[i] << std::endl;
      std::cin.get();
    }
    // Corner case where fn>0, fd=0 and lambda>0
    if (z_groups[i][0] > CornerZero &&
        (z_groups[i].segment(1, numBasis).array() > -CornerZero &&
         z_groups[i].segment(1, numBasis).array() < CornerZero)
            .all() &&
        z_groups[i][9] > CornerZero) {
      std::cerr << "ERROR: fn>0, fd==0 and lambda>0" << std::endl;
      (*Lemke_FILE) << "Lemke A is " << std::endl;
      (*Lemke_FILE) << Lemke_A << std::endl;
      (*Lemke_FILE) << "Lemke b is ";
      (*Lemke_FILE) << Lemke_b.transpose() << std::endl;
      (*Lemke_FILE) << "[z]" << (*z).transpose() << std::endl;
      (*Lemke_FILE) << "[w]" << (Lemke_A * (*z) + Lemke_b).transpose()
                    << std::endl;
      (*Lemke_FILE)
          << "[z].*[w]"
          << ((*z).array() * (Lemke_A * (*z) + Lemke_b).array()).transpose()
          << std::endl;
      std::cerr << z_groups[i] << std::endl;
      std::cin.get();
    }
  }

#ifdef LEMKE_OUTPUT
  (*Lemke_FILE) << "err: " << err << std::endl;
  (*Lemke_FILE) << std::boolalpha;
  (*Lemke_FILE) << "LCP manually validation" << std::endl;
  (*Lemke_FILE) << "[z]" << std::endl
                << (*z).transpose().head(numConstraints) << std::endl;
  for (int i = 0; i < numConstraints; i++) {
    (*Lemke_FILE) << (*z).transpose().segment(numConstraints + i * numBasis,
                                              numBasis)
                  << std::endl;
  }
  (*Lemke_FILE)  << (*z).transpose().tail(numConstraints) << std::endl;

  Eigen::VectorXd mW = (Lemke_A * (*z) + Lemke_b).eval();
  (*Lemke_FILE) << "[w]" << std::endl
                << (mW).transpose().head(numConstraints) << std::endl;
  for (int i = 0; i < numConstraints; i++) {
    (*Lemke_FILE) << (mW).transpose().segment(numConstraints + i * numBasis,
                                              numBasis)
                  << std::endl;
  }
  (*Lemke_FILE)  << (mW).transpose().tail(numConstraints) << std::endl;

  Eigen::VectorXd mWZ = ((*z).array() * mW.array()).eval();
  (*Lemke_FILE) << "[z].*[w]" << std::endl
                << (mWZ).transpose().head(numConstraints) << std::endl;
  for (int i = 0; i < numConstraints; i++) {
    (*Lemke_FILE) << (mWZ).transpose().segment(numConstraints + i * numBasis,
                                              numBasis)
                  << std::endl;
  }
  (*Lemke_FILE)  << (mWZ).transpose().tail(numConstraints) << std::endl;
  (*Lemke_FILE) << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << std::endl;
#endif
  double err_dist = 0.0;
  bool Validation =
      dart::lcpsolver::YT::validate(Lemke_A, (*z), Lemke_b, err_dist);
  // std::cout << "err distance: " << err_dist << std::endl;
#ifdef LEMKE_OUTPUT_DETAILS
  (*Lemke_FILE) << "Validation: " << Validation << std::endl;
  (*Lemke_FILE) << "Lemke N is " << N.transpose() << std::endl;
  (*Lemke_FILE) << "Lemke B is " << std::endl << B << std::endl;
  (*Lemke_FILE) << "Skeleton 1 is " << bodyNode1->getSkeleton()->getName()
                << std::endl
                << " its velocity is"
                << mSkeletonVelocitiesLock[bodyNode1->getSkeleton()].transpose()
                << std::endl;
  (*Lemke_FILE) << "Skeleton 2 is " << bodyNode2->getSkeleton()->getName()
                << std::endl
                << " its velocity is"
                << mSkeletonVelocitiesLock[bodyNode2->getSkeleton()].transpose()
                << std::endl;
  (*Lemke_FILE) << "Lemke M is " << std::endl << M << std::endl;
  (*Lemke_FILE) << "Lemke E is " << E.transpose() << std::endl;
  (*Lemke_FILE) << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << std::endl;
#endif

  //  ---------------------------------------
  // Apply constraint impulses
  if (!Validation) {
    LemkeFailCounter++;
#ifdef LEMKE_FAIL_PRINT
    std::cerr << "ERROR: Lemke fails for current time step!!!" << std::endl;
    std::cerr << "Resort ODE LCP solver to solve the problem!!!" << std::endl;
    std::cerr << "Current frame is " << mWindow->getWorld()->getSimFrames()
              << std::endl;
    (*Lemke_FILE) << "Lemke A is " << std::endl;
    (*Lemke_FILE) << Lemke_A << std::endl;
    (*Lemke_FILE) << "Lemke b is ";
    (*Lemke_FILE) << Lemke_b.transpose() << std::endl;
    (*Lemke_FILE) << "[z]" << (*z).transpose() << std::endl;
    (*Lemke_FILE) << "[w]" << (Lemke_A * (*z) + Lemke_b).transpose()
                  << std::endl;
    (*Lemke_FILE)
        << "[z].*[w]"
        << ((*z).array() * (Lemke_A * (*z) + Lemke_b).array()).transpose()
        << std::endl;
    (*Lemke_FILE) << "err: " << err << std::endl;
    (*Lemke_FILE) << std::boolalpha;
    (*Lemke_FILE) << "Validation: " << Validation << std::endl;
#endif
    // mWindow->keyboard('y',0,0);
  }

  // bookkeeping old A and old b
  double* old_A = new double[n * nSkip];
  double* old_b = new double[n];
  for (int i = 0; i < n * nSkip; i++) old_A[i] = A[i];
  for (int i = 0; i < n; i++) old_b[i] = b[i];

// Solve LCP using ODE's Dantzig algorithm when Lemke cannot solve the problem
#ifndef ALWAYS_ODE
  if (!Validation) {
#endif
    dSolveLCP(n, A, x, b, w, 0, lo, hi, findex);
#ifndef ALWAYS_ODE
  }
#endif

#ifdef ODE_OUTPUT
  // Print LCP formulation
  (*ODE_FILE) << "After solve:" << std::endl;
  print(n, old_A, x, lo, hi, old_b, w, findex, ODE_FILE);
  (*ODE_FILE) << std::endl;
#endif

  /*
   * // Clamp zero or negative
   * clampZero((*z));
   * clampNeg((*z), Validation);
   */

  // ---------------------------------------------------------------------------
  // print out Lemke solution to investigate the pattern
  /*
   *if (Validation) {
   *  if (numConstraints == 1) {
   *    Eigen::IOFormat CSVFmt(Eigen::FullPrecision, Eigen::DontAlignCols,
   *",\t");
   *    std::cout << (*z).transpose().format(CSVFmt) << std::endl;
   *  }
   *}
   */
  // ---------------------------------------------------------------------------

  // Validation = false;

  // ---------------------------------------------------------------------------
  // If Lemke solution is valid, then use Lemke solution to simulate,
  // otherwise use ODE to simulate
  (*Lemke_FILE) << "[Lemke] before I use: "
                << bodyNode1->mConstraintImpulse.transpose()<< std::endl;
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
      Eigen::VectorXd N_each = N.col(idx_cnstrnt);
      Eigen::MatrixXd B_each = B.block(
          0, idx_cnstrnt * numBasis, (BodyNode1_dim + BodyNode2_dim), numBasis);

      Mycntctconstraint = dynamic_cast<MyContactConstraint*>(
          _group->getConstraint(idx_cnstrnt));
      Mycntctconstraint->MyapplyImpulse(fn_each, fd_each, N_each, B_each,
                                        BodyNode1_dim, BodyNode2_dim,
                                        (_TimeStep == 1.0));

      Mycntctconstraint->excite();
    }
    (*Lemke_FILE) << "[Lemke] what I use: "
                  << bodyNode1->mConstraintImpulse.transpose()<< std::endl;
#ifdef CLAMP_DANTZIG
    // It is very important to clamp to zero in this step
    clampZero(bodyNode1->mConstraintImpulse);
#endif
  } else {
    //  ---------------------------------------
    // Using ODE LCP to simulate
    for (size_t i = 0; i < numConstraints; ++i) {
      constraint = _group->getConstraint(i);
      constraint->applyImpulse(x + offset[i]);
      constraint->excite();
    }
  }

  // std::cerr << "bodyNode1 constraint impulse: "
  //       << bodyNode1->mConstraintImpulse.transpose() << std::endl;

  // ---------------------------------------------------------------------------
  Eigen::Vector3d allForce;
  Eigen::Vector3d allTorque;
  Eigen::VectorXd GeneralizedForces(6);
  Eigen::IOFormat CSVFmt(Eigen::FullPrecision, Eigen::DontAlignCols, ",\t");
// print out Lemke apply impulse
#ifdef LEMKE_APPLY_PRINT
  allForce = Eigen::Vector3d::Zero();
  allTorque = Eigen::Vector3d::Zero();
  GeneralizedForces.setZero();
  for (size_t idx_cnstrnt = 0; idx_cnstrnt < numConstraints; ++idx_cnstrnt) {
    MyContactConstraint* Mycntctconstraint;
    // (*z); N; B
    Eigen::VectorXd fn((*z).head(numConstraints));
    Eigen::VectorXd fd((*z).segment(numConstraints, numConstraints * numBasis));
    // Eigen::VectorXd lambda((*z).tail(numConstraints));
    double fn_each = fn(idx_cnstrnt);
    Eigen::VectorXd fd_each = fd.segment(idx_cnstrnt * numBasis, numBasis);
    Eigen::VectorXd N_each = N.col(idx_cnstrnt);
    Eigen::MatrixXd B_each = B.block(0, idx_cnstrnt * numBasis,
                                     (BodyNode1_dim + BodyNode2_dim), numBasis);

    Mycntctconstraint =
        dynamic_cast<MyContactConstraint*>(_group->getConstraint(idx_cnstrnt));
    Eigen::Vector3d indForce(Eigen::Vector3d::Zero());
    Eigen::Vector3d indTorque(Eigen::Vector3d::Zero());
    Eigen::VectorXd indGeneralized(6);
    indGeneralized.setZero();
    Mycntctconstraint->My2LemkeapplyImpulse(
        fn_each, fd_each, N_each, B_each, BodyNode1_dim, BodyNode2_dim,
        (_TimeStep == 1.0), indForce, indTorque,indGeneralized, Lemke_FILE);
    allForce += indForce;
    allTorque += indTorque;
    GeneralizedForces += indGeneralized;
    (*Lemke_FILE) << "[Lemke] all contact GeneralizedForces counter: "
                  << GeneralizedForces.transpose().format(CSVFmt) << std::endl;

    // check mJacobians1 and N relationship
    (*Lemke_FILE) << "My Computation of N is " << N_each.transpose() << std::endl;
    (*Lemke_FILE) << "My Computation of B is " << B_each.transpose() << std::endl;
  }
  (*Lemke_FILE) << "[Lemke] all contact forces: "
                << allForce.transpose().format(CSVFmt) << std::endl;
  (*Lemke_FILE) << "[Lemke] all contact torques: "
                << allTorque.transpose().format(CSVFmt) << std::endl;
  (*Lemke_FILE) << "[Lemke] all contact GeneralizedForces: "
                << GeneralizedForces.transpose().format(CSVFmt) << std::endl;
#endif
// ---------------------------------------------------------------------------
// print out ODE apply impulse
#ifdef ODE_APPLY_PRINT
  allForce = Eigen::Vector3d::Zero();
  allTorque = Eigen::Vector3d::Zero();
  GeneralizedForces.setZero();
  for (size_t idx_cnstrnt = 0; idx_cnstrnt < numConstraints; ++idx_cnstrnt) {
    MyContactConstraint* Mycntctconstraint;
    Mycntctconstraint =
        dynamic_cast<MyContactConstraint*>(_group->getConstraint(idx_cnstrnt));
    Eigen::Vector3d indForce(Eigen::Vector3d::Zero());
    Eigen::Vector3d indTorque(Eigen::Vector3d::Zero());
    Eigen::VectorXd indGeneralized(6);
    indGeneralized.setZero();
    Mycntctconstraint->My2ODEapplyImpulse(x + offset[idx_cnstrnt], indForce,
                                          indTorque,indGeneralized, ODE_FILE);
    allForce += indForce;
    allTorque += indTorque;
    GeneralizedForces += indGeneralized;
    (*ODE_FILE) << "DART's default N    is " << Mycntctconstraint->mJacobians1[0].transpose() << std::endl;
    (*ODE_FILE) << "DART's default B1   is " << Mycntctconstraint->mJacobians1[1].transpose() << std::endl;
    (*ODE_FILE) << "DART's default B2   is " << Mycntctconstraint->mJacobians1[2].transpose() << std::endl;
  }
  (*ODE_FILE) << "[ODE] all contact forces: "
              << allForce.transpose().format(CSVFmt) << std::endl;
  (*ODE_FILE) << "[ODE] all contact torques: "
              << allTorque.transpose().format(CSVFmt) << std::endl;
  (*ODE_FILE) << "[ODE] all contact GeneralizedForces: "
              << GeneralizedForces.transpose().format(CSVFmt) << std::endl;
#endif
// ---------------------------------------------------------------------------

// std::cin.get();

//  ---------------------------------------
#ifdef OUTPUT2FILE
  // output to file after all necessary computation and valid Lemke results
  if (Validation && (counters[numConstraints - 1] < dataSize) && err == 0) {
    recordLCPSolve(Lemke_A, (*z), Lemke_b);
  }
  // early stopping
  if ((counters[0] == dataSize) && (counters[1] == dataSize) &&
      (counters[2] == dataSize) && (counters[3] == dataSize)) {
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

void MyDantzigLCPSolver::print(size_t _n, double* _A, double* _x, double* lo,
                               double* hi, double* b, double* w, int* findex,
                               std::shared_ptr<std::fstream> ODE_FILE) {
  size_t nSkip = dPAD(_n);
  (*ODE_FILE) << "A: " << std::endl;
  for (size_t i = 0; i < _n; ++i) {
    for (size_t j = 0; j < nSkip; ++j) {
      (*ODE_FILE) << std::setprecision(10) << _A[i * nSkip + j] << " ";
    }
    (*ODE_FILE) << std::endl;
  }

  (*ODE_FILE) << "b: ";
  for (size_t i = 0; i < _n; ++i) {
    (*ODE_FILE) << std::setprecision(10) << b[i] << " ";
  }
  (*ODE_FILE) << std::endl;

  (*ODE_FILE) << "w: ";
  for (size_t i = 0; i < _n; ++i) {
    (*ODE_FILE) << w[i] << " ";
  }
  (*ODE_FILE) << std::endl;

  (*ODE_FILE) << "x: ";
  for (size_t i = 0; i < _n; ++i) {
    (*ODE_FILE) << _x[i] << " ";
  }
  (*ODE_FILE) << std::endl;

  //  (*ODE_FILE) << "lb: ";
  //  for (int i = 0; i < dim; ++i)
  //  {
  //    (*ODE_FILE) << lb[i] << " ";
  //  }
  //  (*ODE_FILE) << std::endl;

  //  (*ODE_FILE) << "ub: ";
  //  for (int i = 0; i < dim; ++i)
  //  {
  //    (*ODE_FILE) << ub[i] << " ";
  //  }
  //  (*ODE_FILE) << std::endl;

  // (*ODE_FILE) << "frictionIndex: ";
  // for (size_t i = 0; i < _n; ++i)
  // {
  //     (*ODE_FILE) << findex[i] << " ";
  // }
  // (*ODE_FILE) << std::endl;

  double* Ax = new double[_n];

  for (size_t i = 0; i < _n; ++i) {
    Ax[i] = 0.0;
  }

  for (size_t i = 0; i < _n; ++i) {
    for (size_t j = 0; j < _n; ++j) {
      Ax[i] += _A[i * nSkip + j] * _x[j];
    }
  }

  (*ODE_FILE) << "Ax   : ";
  for (size_t i = 0; i < _n; ++i) {
    (*ODE_FILE) << Ax[i] << " ";
  }
  (*ODE_FILE) << std::endl;

  (*ODE_FILE) << "b + w: ";
  for (size_t i = 0; i < _n; ++i) {
    (*ODE_FILE) << b[i] + w[i] << " ";
  }
  (*ODE_FILE) << std::endl;

  delete[] Ax;
}

bool MyDantzigLCPSolver::isSymmetric(size_t _n, double* _A) {
  size_t nSkip = dPAD(_n);
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
bool MyDantzigLCPSolver::isSymmetric(size_t _n, double* _A, size_t _begin,
                                     size_t _end) {
  size_t nSkip = dPAD(_n);
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

void MyDantzigLCPSolver::pushVelocities(
    dart::dynamics::SkeletonPtr mSkeletonPtr,
    const Eigen::VectorXd& mVelocities) {
  // std::cout<<"Pushing "<<mVelocities.transpose()<<" to
  // "<<mSkeletonPtr->getName()<<std::endl;
  mSkeletonVelocitiesLock[mSkeletonPtr] = mVelocities;
}

std::map<dart::dynamics::SkeletonPtr, Eigen::VectorXd>&
MyDantzigLCPSolver::getSkeletonVelocitiesLock() {
  return mSkeletonVelocitiesLock;
}

int MyDantzigLCPSolver::getLemkeFailCounter() { return LemkeFailCounter; }

void MyDantzigLCPSolver::recordLCPSolve(const Eigen::MatrixXd A,
                                        const Eigen::VectorXd z,
                                        const Eigen::VectorXd b) {
  int nSize = b.rows();
  int numContactsToLearn = nSize / (numBasis + 2);

  std::shared_ptr<std::fstream> outputFile =
      outputFiles[numContactsToLearn - 1];
  counters[numContactsToLearn - 1] += 1;

  //  output A, z, and b
  //  // since all friction coeffs are the same, no need to output them
  for (int i = 0; i < nSize - numContactsToLearn; i++) {
    for (int j = i; j < nSize - numContactsToLearn; j++) {
      (*outputFile) << A(i, j) << ",";
    }
  }

  for (int i = 0; i < nSize - numContactsToLearn; i++) {
    (*outputFile) << b(i) << ",";
  }

  // decompose z
  Eigen::VectorXd z_fn(numContactsToLearn);
  z_fn = z.head(numContactsToLearn);
  Eigen::VectorXd z_fd(numContactsToLearn * numBasis);
  z_fd = z.segment(numContactsToLearn, numContactsToLearn * numBasis);
  Eigen::VectorXd z_lambda(numContactsToLearn);
  z_lambda = z.tail(numContactsToLearn);

  std::vector<Eigen::VectorXd> each_z(numContactsToLearn);
  for (int i = 0; i < numContactsToLearn; i++) {
    each_z[i].resize(numBasis + 2);
    each_z[i] << z_fn(i), z_fd.segment(i * numBasis, numBasis), z_lambda(i);
    int value = 9;

    // Convention: numbasis = 8, so total 10 elements
    if (each_z[i](0) < MY_DART_ZERO)  // fn = 0, break
    {
      value = 9;
    } else if (each_z[i](9) < MY_DART_ZERO)  // lambda = 0, static
    {
      value = 8;
    } else if (each_z[i](1) > MY_DART_ZERO)  // fd_1 > 0
    {
      value = 0;
    } else if (each_z[i](2) > MY_DART_ZERO)  // fd_2 > 0
    {
      value = 1;
    } else if (each_z[i](3) > MY_DART_ZERO)  // fd_3 > 0
    {
      value = 2;
    } else if (each_z[i](4) > MY_DART_ZERO)  // fd_4 > 0
    {
      value = 3;
    } else if (each_z[i](5) > MY_DART_ZERO)  // fd_5 > 0
    {
      value = 4;
    } else if (each_z[i](6) > MY_DART_ZERO)  // fd_6 > 0
    {
      value = 5;
    } else if (each_z[i](7) > MY_DART_ZERO)  // fd_7 > 0
    {
      value = 6;
    } else if (each_z[i](8) > MY_DART_ZERO)  // fd_8 > 0
    {
      value = 7;
    } else {
      std::cerr << "ERROR: unknown LCP solution!!!" << std::endl;
      std::cin.get();
    }
    (*outputFile) << value;
    if (i < numContactsToLearn - 1) {
      (*outputFile) << ",";
    }
  }

  (*outputFile) << std::endl;
}

void MyDantzigLCPSolver::decompose(const Eigen::VectorXd& z,
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
