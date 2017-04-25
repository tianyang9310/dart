#include <fstream>
#include <iostream>

#include "MyContactConstraint.h"
#include "dart/common/Console.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/math/Helpers.h"

#define DART_ERROR_ALLOWANCE 0.0
#define DART_ERP 0.01
#define DART_MAX_ERV 1e-3
#define DART_CFM 1e-5
// #define DART_MAX_NUMBER_OF_CONTACTS 32

#define DART_RESTITUTION_COEFF_THRESHOLD 1e-3
#define DART_FRICTION_COEFF_THRESHOLD 1e-3
#define DART_BOUNCING_VELOCITY_THRESHOLD 1e-1
#define DART_MAX_BOUNCING_VELOCITY 1e+2
// #define DART_CONTACT_CONSTRAINT_EPSILON  1e-6

// #define OUTPUT

namespace CntctLrnin {

//==============================================================================
MyContactConstraint::MyContactConstraint(collision::Contact& _contact,
                                         double _timestep)
    : ContactConstraint() {
  mTimeStep = (_timestep);
  mFirstFrictionalDirection = (Eigen::Vector3d::UnitZ());
  mIsFrictionOn = (true);
  mAppliedImpulseIndex = (-1);
  mIsBounceOn = (false);
  mActive = (false);
  numBasis = NUMBASIS;
  mPrecision = PRECISION;

  // TODO(JS): Assumed single contact
  mContacts.push_back(&_contact);

  // TODO(JS):
  mBodyNode1 = _contact.bodyNode1.lock();
  mBodyNode2 = _contact.bodyNode2.lock();

  //----------------------------------------------
  // Bounce
  //----------------------------------------------
  mRestitutionCoeff =
      mBodyNode1->getRestitutionCoeff() * mBodyNode2->getRestitutionCoeff();
  if (mRestitutionCoeff > DART_RESTITUTION_COEFF_THRESHOLD)
    mIsBounceOn = true;
  else
    mIsBounceOn = false;

  //----------------------------------------------
  // Friction
  //----------------------------------------------
  // TODO(JS): Assume the frictional coefficient can be changed during
  //           simulation steps.
  // Update mFrictionalCoff
  mFrictionCoeff =
      std::min(mBodyNode1->getFrictionCoeff(), mBodyNode2->getFrictionCoeff());
  if (mFrictionCoeff > DART_FRICTION_COEFF_THRESHOLD) {
    mIsFrictionOn = true;

    // Update frictional direction
    updateFirstFrictionalDirection();
  } else {
    mIsFrictionOn = false;
  }

  // Compute local contact Jacobians expressed in body frame
  if (mIsFrictionOn) {
    // Set the dimension of this constraint. 1 is for Normal direction
    // constraint.
    // TODO(JS): Assumed that the number of contact is not static.
    // TODO(JS): Adjust following code once use of mNumFrictionConeBases is
    //           implemented.
    //  mDim = mContacts.size() * (1 + mNumFrictionConeBases);
    mDim = mContacts.size() * (1 + numBasis);

    mJacobians1.resize(mDim);
    mJacobians2.resize(mDim);

    // Intermediate variables
    size_t idx = 0;

    Eigen::Vector3d bodyDirection1;
    Eigen::Vector3d bodyDirection2;

    Eigen::Vector3d bodyPoint1;
    Eigen::Vector3d bodyPoint2;

    for (size_t i = 0; i < mContacts.size(); ++i) {
      collision::Contact* ct = mContacts[i];

      // TODO(JS): Assumed that the number of tangent basis is 2.
      Eigen::MatrixXd D;
      if (numBasis != 2) {
        D = getTangentBasisMatrixLemke(ct->normal, numBasis);
      } else {
        D = getTangentBasisMatrixODE(ct->normal);
      }

      /*
       * assert(std::abs(ct->normal.dot(D.col(0))) < DART_EPSILON);
       * assert(std::abs(ct->normal.dot(D.col(1))) < DART_EPSILON);
       * //      if (D.col(0).dot(D.col(1)) > 0.0)
       * //        std::cout << "D.col(0).dot(D.col(1): " <<
       * D.col(0).dot(D.col(1))
       * //        << std::endl;
       * assert(std::abs(D.col(0).dot(D.col(1))) < DART_EPSILON);
       */

      // Jacobian for normal contact
      bodyDirection1.noalias() =
          mBodyNode1->getTransform().linear().transpose() * ct->normal;
      bodyDirection2.noalias() =
          mBodyNode2->getTransform().linear().transpose() * -ct->normal;

      bodyPoint1.noalias() = mBodyNode1->getTransform().inverse() * ct->point;
      bodyPoint2.noalias() = mBodyNode2->getTransform().inverse() * ct->point;

#ifdef OUTPUT
      // print out normal and D
      std::cout << std::setprecision(mPrecision) << std::endl;
      std::cout << "*********************************************" << std::endl;
      std::cout << "bodyPoint1: " << bodyPoint1.transpose() << std::endl;
      std::cout << "bodeDirection1: " << bodyDirection1.transpose()
                << std::endl;
// std::cout << "D: " << std::endl << D << std::endl;
#endif

      mJacobians1[idx].head<3>() = bodyPoint1.cross(bodyDirection1);
      mJacobians2[idx].head<3>() = bodyPoint2.cross(bodyDirection2);

      mJacobians1[idx].tail<3>() = bodyDirection1;
      mJacobians2[idx].tail<3>() = bodyDirection2;

      idx++;

      for (; idx < (i + 1) * (numBasis + 1); idx++) {
        // Jacobian for directional friction 1
        bodyDirection1.noalias() =
            mBodyNode1->getTransform().linear().transpose() *
            D.col((idx % (1 + numBasis)) - 1);
        bodyDirection2.noalias() =
            mBodyNode2->getTransform().linear().transpose() *
            -D.col((idx % (1 + numBasis)) - 1);

        // bodyPoint1.noalias() =
        //     mBodyNode1->getWorldTransform().inverse() * ct->point;
        // bodyPoint2.noalias() =
        //     mBodyNode2->getWorldTransform().inverse() * ct->point;

        // std::cout << "bodyDirection2: " << std::endl
        //           << bodyDirection2 << std::endl;

        mJacobians1[idx].head<3>() = bodyPoint1.cross(bodyDirection1);
        mJacobians2[idx].head<3>() = bodyPoint2.cross(bodyDirection2);

        mJacobians1[idx].tail<3>() = bodyDirection1;
        mJacobians2[idx].tail<3>() = bodyDirection2;
      }

      /*
       *
       *  ++idx;
       *
       *  // Jacobian for directional friction 1
       *  bodyDirection1.noalias() =
       *      mBodyNode1->getTransform().linear().transpose() * D.col(0);
       *  bodyDirection2.noalias() =
       *      mBodyNode2->getTransform().linear().transpose() * -D.col(0);
       *
       *  // bodyPoint1.noalias()
       *  //     = mBodyNode1->getWorldTransform().inverse() * ct->point;
       *  // bodyPoint2.noalias()
       *  //     = mBodyNode2->getWorldTransform().inverse() * ct->point;
       *
       *  // std::cout << "bodyDirection2: " << std::endl << bodyDirection2 <<
       *  // std::endl;
       *
       *  mJacobians1[idx].head<3>() = bodyPoint1.cross(bodyDirection1);
       *  mJacobians2[idx].head<3>() = bodyPoint2.cross(bodyDirection2);
       *
       *  mJacobians1[idx].tail<3>() = bodyDirection1;
       *  mJacobians2[idx].tail<3>() = bodyDirection2;
       *
       *  ++idx;
       *
       *  // Jacobian for directional friction 2
       *  bodyDirection1.noalias() =
       *      mBodyNode1->getTransform().linear().transpose() * D.col(1);
       *  bodyDirection2.noalias() =
       *      mBodyNode2->getTransform().linear().transpose() * -D.col(1);
       *
       *  // bodyPoint1.noalias()
       *  //     = mBodyNode1->getWorldTransform().inverse() * ct->point;
       *  // bodyPoint2.noalias()
       *  //     = mBodyNode2->getWorldTransform().inverse() * ct->point;
       *
       *  // std::cout << "bodyDirection2: " << std::endl << bodyDirection2 <<
       *  // std::endl;
       *
       *  mJacobians1[idx].head<3>() = bodyPoint1.cross(bodyDirection1);
       *  mJacobians2[idx].head<3>() = bodyPoint2.cross(bodyDirection2);
       *
       *  mJacobians1[idx].tail<3>() = bodyDirection1;
       *  mJacobians2[idx].tail<3>() = bodyDirection2;
       *
       *  ++idx;
       */
    }
  } else {
    // Set the dimension of this constraint.
    mDim = mContacts.size();

    mJacobians1.resize(mDim);
    mJacobians2.resize(mDim);

    Eigen::Vector3d bodyDirection1;
    Eigen::Vector3d bodyDirection2;

    Eigen::Vector3d bodyPoint1;
    Eigen::Vector3d bodyPoint2;

    for (size_t i = 0; i < mContacts.size(); ++i) {
      collision::Contact* ct = mContacts[i];

      bodyDirection1.noalias() =
          mBodyNode1->getTransform().linear().transpose() * ct->normal;
      bodyDirection2.noalias() =
          mBodyNode2->getTransform().linear().transpose() * -ct->normal;

      bodyPoint1.noalias() = mBodyNode1->getTransform().inverse() * ct->point;
      bodyPoint2.noalias() = mBodyNode2->getTransform().inverse() * ct->point;

      mJacobians1[i].head<3>().noalias() = bodyPoint1.cross(bodyDirection1);
      mJacobians2[i].head<3>().noalias() = bodyPoint2.cross(bodyDirection2);

      mJacobians1[i].tail<3>().noalias() = bodyDirection1;
      mJacobians2[i].tail<3>().noalias() = bodyDirection2;
    }
  }

  //----------------------------------------------------------------------------
  // Union finding
  //----------------------------------------------------------------------------
  //  uniteSkeletons();
}

//==============================================================================
MyContactConstraint::~MyContactConstraint() {
  // pass
}

//==============================================================================
void MyContactConstraint::getInformation(ConstraintInfo* _info) {
  // Fill w, where the LCP form is Ax = b + w (x >= 0, w >= 0, x^T w = 0)
  getRelVelocity(_info->b);

  //----------------------------------------------------------------------------
  // Friction case
  //----------------------------------------------------------------------------
  if (mIsFrictionOn) {
    size_t index = 0;
    for (size_t i = 0; i < mContacts.size(); ++i) {
      // Bias term, w, should be zero
      for (size_t mIdxOffset = 0; mIdxOffset < 1 + numBasis; mIdxOffset++) {
        assert(_info->w[index + mIdxOffset] == 0.0);

        // Initialization lower and upper bounds
        if (mIdxOffset == 0) {
          _info->lo[index] = 0.0;
          _info->hi[index] = dInfinity;
          assert(_info->findex[index] == -1);
        } else {
          /*
           * _info->lo[index + mIdxOffset] = -mFrictionCoeff;
           * _info->hi[index + mIdxOffset] = mFrictionCoeff;
           */
          _info->lo[index + mIdxOffset] = -1 * mFrictionCoeff;
          _info->hi[index + mIdxOffset] = 1 * mFrictionCoeff;
          _info->findex[index + mIdxOffset] = index;
        }

        // Initial guess
        _info->x[index + mIdxOffset] = 0.0;
      }

      /*
       * // Upper and lower bounds of normal impulsive force
       * _info->lo[index] = 0.0;
       * _info->hi[index] = dInfinity;
       * assert(_info->findex[index] == -1);
       *
       * // Upper and lower bounds of tangential direction-1 impulsive force
       * _info->lo[index + 1] = -mFrictionCoeff;
       * _info->hi[index + 1] = mFrictionCoeff;
       * _info->findex[index + 1] = index;
       *
       * // Upper and lower bounds of tangential direction-2 impulsive force
       * _info->lo[index + 2] = -mFrictionCoeff;
       * _info->hi[index + 2] = mFrictionCoeff;
       * _info->findex[index + 2] = index;
       */

      //      std::cout << "_frictionalCoff: " << _frictionalCoff << std::endl;

      //      std::cout << "_lcp->ub[_idx + 1]: " << _lcp->ub[_idx + 1] <<
      //      std::endl;
      //      std::cout << "_lcp->ub[_idx + 2]: " << _lcp->ub[_idx + 2] <<
      //      std::endl;

      //------------------------------------------------------------------------
      // Bouncing
      //------------------------------------------------------------------------
      // A. Penetration correction
      double bouncingVelocity =
          mContacts[i]->penetrationDepth - mErrorAllowance;
      if (bouncingVelocity < 0.0) {
        bouncingVelocity = 0.0;
      } else {
        bouncingVelocity *= mErrorReductionParameter * _info->invTimeStep;
        if (bouncingVelocity > mMaxErrorReductionVelocity)
          bouncingVelocity = mMaxErrorReductionVelocity;
      }

      // B. Restitution
      if (mIsBounceOn) {
        double& negativeRelativeVel = _info->b[index];
        double restitutionVel = negativeRelativeVel * mRestitutionCoeff;

        if (restitutionVel > DART_BOUNCING_VELOCITY_THRESHOLD) {
          if (restitutionVel > bouncingVelocity) {
            bouncingVelocity = restitutionVel;

            if (bouncingVelocity > DART_MAX_BOUNCING_VELOCITY) {
              bouncingVelocity = DART_MAX_BOUNCING_VELOCITY;
            }
          }
        }
      }

      //
      //      _lcp->b[_idx] = _lcp->b[_idx] * 1.1;
      //      std::cout << "_lcp->b[_idx]: " << _lcp->b[_idx] << std::endl;
      /// No ODE such a hack of adding bouncing velocity
      // _info->b[index] += bouncingVelocity;
      //      std::cout << "_lcp->b[_idx]: " << _lcp->b[_idx] << std::endl;

      /*
       * // TODO(JS): Initial guess
       * // x
       * _info->x[index] = 0.0;
       * _info->x[index + 1] = 0.0;
       * _info->x[index + 2] = 0.0;
       */

      // Increase index
      index += (1 + numBasis);
    }
  }
  //----------------------------------------------------------------------------
  // Frictionless case
  //----------------------------------------------------------------------------
  else {
    for (size_t i = 0; i < mContacts.size(); ++i) {
      // Bias term, w, should be zero
      _info->w[i] = 0.0;

      // Upper and lower bounds of normal impulsive force
      _info->lo[i] = 0.0;
      _info->hi[i] = dInfinity;
      assert(_info->findex[i] == -1);

      //------------------------------------------------------------------------
      // Bouncing
      //------------------------------------------------------------------------
      // A. Penetration correction
      double bouncingVelocity =
          mContacts[i]->penetrationDepth - DART_ERROR_ALLOWANCE;
      if (bouncingVelocity < 0.0) {
        bouncingVelocity = 0.0;
      } else {
        bouncingVelocity *= mErrorReductionParameter * _info->invTimeStep;
        if (bouncingVelocity > mMaxErrorReductionVelocity)
          bouncingVelocity = mMaxErrorReductionVelocity;
      }

      // B. Restitution
      if (mIsBounceOn) {
        double& negativeRelativeVel = _info->b[i];
        double restitutionVel = negativeRelativeVel * mRestitutionCoeff;

        if (restitutionVel > DART_BOUNCING_VELOCITY_THRESHOLD) {
          if (restitutionVel > bouncingVelocity) {
            bouncingVelocity = restitutionVel;

            if (bouncingVelocity > DART_MAX_BOUNCING_VELOCITY)
              bouncingVelocity = DART_MAX_BOUNCING_VELOCITY;
          }
        }
      }

      //
      //      _lcp->b[_idx] = _lcp->b[_idx] * 1.1;
      //      std::cout << "_lcp->b[_idx]: " << _lcp->b[_idx] << std::endl;
      _info->b[i] += bouncingVelocity;
      //      std::cout << "_lcp->b[_idx]: " << _lcp->b[_idx] << std::endl;

      // TODO(JS): Initial guess
      // x
      _info->x[i] = 0.0;

      // Increase index
    }
  }
}

//==============================================================================
void MyContactConstraint::MyapplyImpulse(double fn, const Eigen::VectorXd& fd,
                                         bool impulse_flag) {
  //----------------------------------------------------------------------------
  // Friction case
  //----------------------------------------------------------------------------
  if (mIsFrictionOn) {
    size_t index = 0;

    for (size_t i = 0; i < mContacts.size(); ++i) {
      //      std::cout << "_lambda1: " << _lambda[_idx] << std::endl;
      //      std::cout << "_lambda2: " << _lambda[_idx + 1] << std::endl;
      //      std::cout << "_lambda3: " << _lambda[_idx + 2] << std::endl;

      //      std::cout << "imp1: " << mJacobians2[i * 3 + 0] * _lambda[_idx] <<
      //      std::endl;
      //      std::cout << "imp2: " << mJacobians2[i * 3 + 1] * _lambda[_idx +
      //      1] << std::endl;
      //      std::cout << "imp3: " << mJacobians2[i * 3 + 2] * _lambda[_idx +
      //      2] << std::endl;

      assert(!math::isNan(fn));

      // Store contact impulse (force) toward the normal w.r.t. world frame
      mContacts[i]->force =
          mContacts[i]->normal * fn / (impulse_flag ? mTimeStep : 1);

      // Normal impulsive force
      //      mContacts[i]->lambda[0] = _lambda[_idx];
      if (mBodyNode1->isReactive())
        mBodyNode1->addConstraintImpulse(mJacobians1[index] * fn);
      if (mBodyNode2->isReactive())
        mBodyNode2->addConstraintImpulse(mJacobians2[index] * fn);
      //      std::cout << "_lambda: " << _lambda[_idx] << std::endl;
      index++;

      assert(!(Eigen::isnan(fd.array()).any()));

      Eigen::MatrixXd D;
      if (numBasis != 2) {
        D = getTangentBasisMatrixLemke(mContacts[i]->normal, numBasis);
      } else {
        D = getTangentBasisMatrixODE(mContacts[i]->normal);
      }

      Eigen::VectorXd oldConstraintImp = mBodyNode1->mConstraintImpulse;

      for (; index < (i + 1) * (numBasis + 1); index++) {
        int mFrctnBsIdx = index % (1 + numBasis) - 1;
        mContacts[i]->force += D.col(mFrctnBsIdx) * fd(mFrctnBsIdx) /
                               (impulse_flag ? mTimeStep : 1);

        if (mBodyNode1->isReactive())
          mBodyNode1->addConstraintImpulse(mJacobians1[index] *
                                           fd(mFrctnBsIdx));
        if (mBodyNode2->isReactive())
          mBodyNode2->addConstraintImpulse(mJacobians2[index] *
                                           fd(mFrctnBsIdx));
      }

      /*
       *    assert(!math::isNan(_lambda[index]));
       *
       *    // Add contact impulse (force) toward the tangential w.r.t. world
       * frame
       *    Eigen::MatrixXd D = getTangentBasisMatrixODE(mContacts[i]->normal);
       *    mContacts[i]->force += D.col(0) * _lambda[index] / mTimeStep;
       *
       *    // Tangential direction-1 impulsive force
       * //   mContacts[i]->lambda[1] = _lambda[_idx];
       *    if (mBodyNode1->isReactive())
       *      mBodyNode1->addConstraintImpulse(mJacobians1[index] *
       * _lambda[index]);
       *    if (mBodyNode2->isReactive())
       *      mBodyNode2->addConstraintImpulse(mJacobians2[index] *
       * _lambda[index]);
       * //   std::cout << "_lambda: " << _lambda[_idx] << std::endl;
       *    index++;
       *
       *    assert(!math::isNan(_lambda[index]));
       *
       *    // Add contact impulse (force) toward the tangential w.r.t. world
       * frame
       *    mContacts[i]->force += D.col(1) * _lambda[index] / mTimeStep;
       *
       *    // Tangential direction-2 impulsive force
       * //   mContacts[i]->lambda[2] = _lambda[_idx];
       *    if (mBodyNode1->isReactive())
       *      mBodyNode1->addConstraintImpulse(mJacobians1[index] *
       * _lambda[index]);
       *    if (mBodyNode2->isReactive())
       *      mBodyNode2->addConstraintImpulse(mJacobians2[index] *
       * _lambda[index]);
       * //   std::cout << "_lambda: " << _lambda[_idx] << std::endl;
       *    index++;
       */
    }
  }
  //----------------------------------------------------------------------------
  // Frictionless case
  //----------------------------------------------------------------------------
  else {
    for (size_t i = 0; i < mContacts.size(); ++i) {
      // Normal impulsive force
      //			pContactPts[i]->lambda[0] = _lambda[i];
      if (mBodyNode1->isReactive())
        mBodyNode1->addConstraintImpulse(mJacobians1[i] * fn);

      if (mBodyNode2->isReactive())
        mBodyNode2->addConstraintImpulse(mJacobians2[i] * fn);

      // Store contact impulse (force) toward the normal w.r.t. world frame
      mContacts[i]->force =
          mContacts[i]->normal * fn / (impulse_flag ? mTimeStep : 1);
    }
  }
}

//==============================================================================
void MyContactConstraint::applyImpulse(double* _lambda) {
  //----------------------------------------------------------------------------
  // Friction case
  //----------------------------------------------------------------------------
  if (mIsFrictionOn) {
    size_t index = 0;

    for (size_t i = 0; i < mContacts.size(); ++i) {
      //      std::cout << "_lambda1: " << _lambda[_idx] << std::endl;
      //      std::cout << "_lambda2: " << _lambda[_idx + 1] << std::endl;
      //      std::cout << "_lambda3: " << _lambda[_idx + 2] << std::endl;

      //      std::cout << "imp1: " << mJacobians2[i * 3 + 0] * _lambda[_idx] <<
      //      std::endl;
      //      std::cout << "imp2: " << mJacobians2[i * 3 + 1] * _lambda[_idx +
      //      1] << std::endl;
      //      std::cout << "imp3: " << mJacobians2[i * 3 + 2] * _lambda[_idx +
      //      2] << std::endl;

      assert(!math::isNan(_lambda[index]));

      // Store contact impulse (force) toward the normal w.r.t. world frame
      mContacts[i]->force = mContacts[i]->normal * _lambda[index] / mTimeStep;

      // Normal impulsive force
      //      mContacts[i]->lambda[0] = _lambda[_idx];
      if (mBodyNode1->isReactive())
        mBodyNode1->addConstraintImpulse(mJacobians1[index] * _lambda[index]);
      if (mBodyNode2->isReactive())
        mBodyNode2->addConstraintImpulse(mJacobians2[index] * _lambda[index]);
      //      std::cout << "_lambda: " << _lambda[_idx] << std::endl;
      index++;

      Eigen::VectorXd oldConstraintImp = mBodyNode1->mConstraintImpulse;

      assert(!math::isNan(_lambda[index]));

      // Add contact impulse (force) toward the tangential w.r.t. world frame
      Eigen::MatrixXd D = getTangentBasisMatrixODE(mContacts[i]->normal);
      mContacts[i]->force += D.col(0) * _lambda[index] / mTimeStep;

      // Tangential direction-1 impulsive force
      //      mContacts[i]->lambda[1] = _lambda[_idx];
      if (mBodyNode1->isReactive())
        mBodyNode1->addConstraintImpulse(mJacobians1[index] * _lambda[index]);
      if (mBodyNode2->isReactive())
        mBodyNode2->addConstraintImpulse(mJacobians2[index] * _lambda[index]);
      //      std::cout << "_lambda: " << _lambda[_idx] << std::endl;

      index++;

      assert(!math::isNan(_lambda[index]));

      // Add contact impulse (force) toward the tangential w.r.t. world frame
      mContacts[i]->force += D.col(1) * _lambda[index] / mTimeStep;

      // Tangential direction-2 impulsive force
      //      mContacts[i]->lambda[2] = _lambda[_idx];
      if (mBodyNode1->isReactive())
        mBodyNode1->addConstraintImpulse(mJacobians1[index] * _lambda[index]);
      if (mBodyNode2->isReactive())
        mBodyNode2->addConstraintImpulse(mJacobians2[index] * _lambda[index]);
      //      std::cout << "_lambda: " << _lambda[_idx] << std::endl;
      index++;
    }
  }
  //----------------------------------------------------------------------------
  // Frictionless case
  //----------------------------------------------------------------------------
  else {
    for (size_t i = 0; i < mContacts.size(); ++i) {
      // Normal impulsive force
      //			pContactPts[i]->lambda[0] = _lambda[i];
      if (mBodyNode1->isReactive())
        mBodyNode1->addConstraintImpulse(mJacobians1[i] * _lambda[i]);

      if (mBodyNode2->isReactive())
        mBodyNode2->addConstraintImpulse(mJacobians2[i] * _lambda[i]);

      // Store contact impulse (force) toward the normal w.r.t. world frame
      mContacts[i]->force = mContacts[i]->normal * _lambda[i] / mTimeStep;
    }
  }
}

//==============================================================================
void MyContactConstraint::getRelVelocity(double* _relVel) {
  assert(_relVel != nullptr && "Null pointer is not allowed.");

  for (size_t i = 0; i < mDim; ++i) {
    _relVel[i] = 0.0;

    if (mBodyNode1->isReactive())
      _relVel[i] -= mJacobians1[i].dot(mBodyNode1->getSpatialVelocity());

    if (mBodyNode2->isReactive())
      _relVel[i] -= mJacobians2[i].dot(mBodyNode2->getSpatialVelocity());

    //    std::cout << "_relVel[i + _idx]: " << _relVel[i + _idx] << std::endl;
  }
}

//==============================================================================
void MyContactConstraint::applyUnitImpulse(size_t _idx) {
  assert(_idx < mDim && "Invalid Index.");
  assert(isActive());
  assert(mBodyNode1->isReactive() || mBodyNode2->isReactive());

  const dynamics::SkeletonPtr& skel1 = mBodyNode1->getSkeleton();
  const dynamics::SkeletonPtr& skel2 = mBodyNode2->getSkeleton();

  // Self collision case
  if (skel1 == skel2) {
    skel1->clearConstraintImpulses();

    if (mBodyNode1->isReactive()) {
      // Both bodies are reactive
      if (mBodyNode2->isReactive()) {
        skel1->updateBiasImpulse(mBodyNode1, mJacobians1[_idx], mBodyNode2,
                                 mJacobians2[_idx]);
      }
      // Only body1 is reactive
      else {
        skel1->updateBiasImpulse(mBodyNode1, mJacobians1[_idx]);
      }
    } else {
      // Only body2 is reactive
      if (mBodyNode2->isReactive()) {
        skel2->updateBiasImpulse(mBodyNode2, mJacobians2[_idx]);
      }
      // Both bodies are not reactive
      else {
        // This case should not be happed
        assert(0);
      }
    }

    skel1->updateVelocityChange();
  }
  // Colliding two distinct skeletons
  else {
    if (mBodyNode1->isReactive()) {
      skel1->clearConstraintImpulses();
      skel1->updateBiasImpulse(mBodyNode1, mJacobians1[_idx]);
      skel1->updateVelocityChange();
    }

    if (mBodyNode2->isReactive()) {
      skel2->clearConstraintImpulses();
      skel2->updateBiasImpulse(mBodyNode2, mJacobians2[_idx]);
      skel2->updateVelocityChange();
    }
  }

  mAppliedImpulseIndex = _idx;
}

//==============================================================================
void MyContactConstraint::getVelocityChange(double* _vel, bool _withCfm) {
  assert(_vel != nullptr && "Null pointer is not allowed.");

  for (size_t i = 0; i < mDim; ++i) {
    _vel[i] = 0.0;

    if (mBodyNode1->getSkeleton()->isImpulseApplied() &&
        mBodyNode1->isReactive()) {
      _vel[i] += mJacobians1[i].dot(mBodyNode1->getBodyVelocityChange());
    }

    if (mBodyNode2->getSkeleton()->isImpulseApplied() &&
        mBodyNode2->isReactive()) {
      _vel[i] += mJacobians2[i].dot(mBodyNode2->getBodyVelocityChange());
    }
  }

  // Add small values to the diagnal to keep it away from singular, similar to
  // cfm variable in ODE
  if (_withCfm) {
    _vel[mAppliedImpulseIndex] +=
        _vel[mAppliedImpulseIndex] * mConstraintForceMixing;
  }
}
}  // namespace CntctLrnin
