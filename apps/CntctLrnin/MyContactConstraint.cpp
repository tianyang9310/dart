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

#define OUTPUT

#define CLAMP_CONTACT_CONSTRAINT

namespace dart {
namespace constraint {

MyContactConstraint::MyContactConstraint(collision::Contact& _contact,
                                         double _timestep)
    : ContactConstraint(_contact, _timestep) {
  numBasis = 8;
}

MyContactConstraint::~MyContactConstraint() {}

// my own apply impulse method for using Lemke result to simulate
void MyContactConstraint::MyapplyImpulse(double fn, const Eigen::VectorXd& fd,
                                         const Eigen::VectorXd& N,
                                         const Eigen::MatrixXd& B,
                                         int BodyNode1_dim, int BodyNode2_dim,
                                         bool impulse_flag) {
  if (mIsFrictionOn) {
    for (size_t i = 0; i < mContacts.size(); ++i) {
      // normal impulsive force and tangential direction impulsive force
      Eigen::VectorXd normal_impulsive_force;
      Eigen::VectorXd tangential_directional_force;

      normal_impulsive_force = N * fn * (impulse_flag ? 1 : mTimeStep);
      tangential_directional_force = B * fd * (impulse_flag ? 1 : mTimeStep);

      Eigen::MatrixXd D =
          getTangentBasisMatrixLemke(mContacts[i]->normal, numBasis);
      mContacts[i]->force =
          mContacts[i]->normal * fn / (impulse_flag ? mTimeStep : 1);
      mContacts[i]->force += D * fd / (impulse_flag ? mTimeStep : 1);

#ifdef  CLAMP_CONTACT_CONSTRAINT
      // Clamp zero
      clampZero(normal_impulsive_force);
      clampZero(tangential_directional_force);
      clampZero(mContacts[i]->force);
#endif

      if (mBodyNode1->isReactive()) {
        mBodyNode1->addConstraintImpulse(
            normal_impulsive_force.head(BodyNode1_dim));
        mBodyNode1->addConstraintImpulse(
            tangential_directional_force.head(BodyNode1_dim));
      }

      if (mBodyNode2->isReactive()) {
        mBodyNode2->addConstraintImpulse(
            normal_impulsive_force.tail(BodyNode2_dim));
        mBodyNode2->addConstraintImpulse(
            tangential_directional_force.tail(BodyNode2_dim));
      }
    }
  } else {
    for (size_t i = 0; i < mContacts.size(); ++i) {
      Eigen::VectorXd normal_impulsive_force;
      normal_impulsive_force = N * fn * (impulse_flag ? 1 : mTimeStep);
      if (mBodyNode1->isReactive()) {
        mBodyNode1->addConstraintImpulse(
            normal_impulsive_force.head(BodyNode1_dim));
      }

      if (mBodyNode2->isReactive()) {
        mBodyNode2->addConstraintImpulse(
            normal_impulsive_force.tail(BodyNode2_dim));
      }

      mContacts[i]->force =
          mContacts[i]->normal * fn / (impulse_flag ? mTimeStep : 1);
    }
  }
}

void MyContactConstraint::My2LemkeapplyImpulse(
    double fn, const Eigen::VectorXd& fd, const Eigen::VectorXd& N,
    const Eigen::MatrixXd& B, int BodyNode1_dim, int BodyNode2_dim,
    bool impulse_flag, Eigen::Vector3d& allForce, Eigen::Vector3d& allTorque) {
#ifdef OUTPUT
  std::cout << "============================================" << std::endl;
  std::cout << "[Lemke LCP] Using MyContactConstraint class! " << std::endl;
  std::cout << "[Lemke LCP] fn: " << fn << std::endl;
  std::cout << "[Lemke LCP] fd: " << fd.transpose() << std::endl;
// std::cin.get();
#endif

  if (mIsFrictionOn) {
    for (size_t i = 0; i < mContacts.size(); ++i) {
      // normal impulsive force and tangential direction impulsive force
      Eigen::VectorXd normal_impulsive_force;
      Eigen::VectorXd tangential_directional_force;

      normal_impulsive_force = N * fn * (impulse_flag ? 1 : mTimeStep);
      tangential_directional_force = B * fd * (impulse_flag ? 1 : mTimeStep);

      Eigen::MatrixXd D =
          getTangentBasisMatrixLemke(mContacts[i]->normal, numBasis);
      Eigen::Vector3d Myforce;
      Myforce = mContacts[i]->normal * fn / (impulse_flag ? mTimeStep : 1);
      Myforce += D * fd / (impulse_flag ? mTimeStep : 1);

#ifdef  CLAMP_CONTACT_CONSTRAINT
      // Clamp zero
      clampZero(normal_impulsive_force);
      clampZero(tangential_directional_force);
      clampZero(Myforce);
#endif
      
      Eigen::Vector3d bodyPoint1;
      bodyPoint1.noalias() =
          mBodyNode1->getTransform().inverse() * mContacts[i]->point;

      allTorque.setZero();
      if (mBodyNode1->isReactive()) {
        Eigen::Vector3d normal_force_tmp;
        normal_force_tmp =
            mContacts[i]->normal * fn / (impulse_flag ? mTimeStep : 1);
#ifdef  CLAMP_CONTACT_CONSTRAINT
        clampZero(normal_force_tmp);
#endif
        allTorque += bodyPoint1.cross(normal_force_tmp);
        Eigen::Vector3d friction_force_tmp;
        friction_force_tmp = D * fd / (impulse_flag ? mTimeStep : 1);
#ifdef  CLAMP_CONTACT_CONSTRAINT
        clampZero(friction_force_tmp);
#endif
        allTorque += bodyPoint1.cross(friction_force_tmp);
      }

/*
 * Eigen::Vector3d bodyPoint2;
 * bodyPoint2.noalias() =
 *     mBodyNode2->getTransform().inverse() * mContacts[i]->point;
 * // Here still suppose contact convention
 * if (mBodyNode2->isReactive()) {
 *  allTorque += bodyPoint2.cross(
 *      (mContacts[i]->normal * fn / (impulse_flag ? mTimeStep : 1))
 *          .eval());
 *  allTorque +=
 *      bodypoint2.cross((D * fd / (impulse_flag ? mTimeStep : 1)).eval());
 * }
 */

#ifdef OUTPUT
      std::cout << "[Lemke LCP] normal impulsive force is "
                << normal_impulsive_force.transpose() << std::endl;
      std::cout << "[Lemke LCP] tangential directional force is "
                << tangential_directional_force.transpose() << std::endl;
      std::cout << "[Lemke LCP] overall force is " << Myforce.transpose()
                << std::endl;
      std::cout << "[Lemke LCP] first force is "
                << (mContacts[i]->normal * fn / mTimeStep).transpose()
                << std::endl;
      std::cout << "[Lemke LCP] second force is "
                << (D * fd / mTimeStep).transpose() << std::endl;
#endif
      allForce = Myforce;
    }
  } else {
    dterr << "No implementation for frictionless case!" << std::endl;
  }
}

void MyContactConstraint::My2ODEapplyImpulse(double* _lambda,
                                             Eigen::Vector3d& allForce,
                                             Eigen::Vector3d& allTorque) {
#ifdef OUTPUT
  std::cout << "============================================" << std::endl;
  std::cout << "[ODE LCP] Using MyContactConstraint class! " << std::endl;
  std::cout << "[ODE LCP] fn: " << _lambda[0] << std::endl;
  std::cout << "[ODE LCP] fd: " << _lambda[1] << " " << _lambda[2] << std::endl;
// std::cin.get();
#endif

  //----------------------------------------------------------------------------
  // Friction case
  //----------------------------------------------------------------------------
  if (mIsFrictionOn) {
    size_t index = 0;

    for (size_t i = 0; i < mContacts.size(); ++i) {
      Eigen::VectorXd Myforce = mContacts[i]->normal * _lambda[0] / mTimeStep;

      Eigen::MatrixXd D = getTangentBasisMatrixODE(mContacts[i]->normal);
      Myforce += D.col(0) * _lambda[1] / mTimeStep;
      Myforce += D.col(1) * _lambda[2] / mTimeStep;

      Eigen::Vector3d bodyPoint1;
      bodyPoint1.noalias() =
          mBodyNode1->getTransform().inverse() * mContacts[i]->point;

      allTorque.setZero();
      if (mBodyNode1->isReactive()) {
        allTorque +=
            bodyPoint1.cross(mContacts[i]->normal * _lambda[0] / mTimeStep);
        Eigen::Vector3d friction_force_tmp = D.col(0) * _lambda[1] / mTimeStep +
                                             D.col(1) * _lambda[2] / mTimeStep;
        allTorque += bodyPoint1.cross(friction_force_tmp);
      }

/*
 * Eigen::Vector3d bodyPoint2;
 * bodyPoint2.noalias() =
 *     mBodyNode2->getTransform().inverse() * mContacts[i]->point;
 * // Here still suppose contact convention
 * if (mBodyNode2->isReactive()) {
 *  allTorque += bodyPoint2.cross(
 *      (mContacts[i]->normal * fn / (impulse_flag ? mTimeStep : 1))
 *          .eval());
 *  allTorque +=
 *      bodypoint2.cross((D * fd / (impulse_flag ? mTimeStep : 1)).eval());
 * }
 */

#ifdef OUTPUT
      std::cout
          << "[ODE LCP] normal impulsive force is "
          << (mJacobians1[i] * _lambda[0]).transpose()
          << std::
                 endl;  // <<(mJacobians2[i]*_lambda[0]).transpose()<<std::endl;
      // std::cout<<std::boolalpha;
      // std::cout<<"mBodyNode1 isReactive: "<<mBodyNode1->isReactive()<<"
      // mBodyNode2 isReactive: "<<mBodyNode2->isReactive()<<std::endl;
      // std::cout<<"[ODE LCP] tangential directional force is
      // "<<(mJacobians1[1] * _lambda[1]).transpose()<<std::endl; //
      // <<(mJacobians2[1] * _lambda[1]).transpose()<<std::endl;
      // std::cout<<"[ODE LCP] tangential directional force is
      // "<<(mJacobians1[2] * _lambda[2]).transpose()<<std::endl; //
      // <<(mJacobians2[2] * _lambda[2]).transpose()<<std::endl;
      std::cout << "[ODE LCP] tangential directional force is "
                << ((mJacobians1[1] * _lambda[1]).transpose() +
                    (mJacobians1[2] * _lambda[2]).transpose())
                << std::endl;  // <<(mJacobians2[1] *
                               // _lambda[1]).transpose()<<std::endl;
      std::cout << "[ODE LCP] overall force is " << Myforce.transpose()
                << std::endl;
      std::cout << "[ODE LCP] overall first is "
                << (mContacts[i]->normal * _lambda[0] / mTimeStep).transpose()
                << std::endl;
      std::cout << "[ODE LCP] overall second 1 is "
                << (D.col(0) * _lambda[1] / mTimeStep).transpose() << std::endl;
      std::cout << "[ODE LCP] overall second 2 is "
                << (D.col(0) * _lambda[2] / mTimeStep).transpose() << std::endl;
#endif
      allForce = Myforce;
    }
  }
  //----------------------------------------------------------------------------
  // Frictionless case
  //----------------------------------------------------------------------------
  else {
    dterr << "No implementation for frictionless case!" << std::endl;
  }
}

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
      assert(_info->w[index] == 0.0);
      assert(_info->w[index + 1] == 0.0);
      assert(_info->w[index + 2] == 0.0);

      // Upper and lower bounds of normal impulsive force
      _info->lo[index] = 0.0;
      _info->hi[index] = dInfinity;
      assert(_info->findex[index] == -1);

      // Upper and lower bounds of tangential direction-1 impulsive force
      _info->lo[index + 1] = -mFrictionCoeff;
      _info->hi[index + 1] = mFrictionCoeff;
      _info->findex[index + 1] = index;

      // Upper and lower bounds of tangential direction-2 impulsive force
      _info->lo[index + 2] = -mFrictionCoeff;
      _info->hi[index + 2] = mFrictionCoeff;
      _info->findex[index + 2] = index;

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
      _info->b[index] += bouncingVelocity;
      //      std::cout << "_lcp->b[_idx]: " << _lcp->b[_idx] << std::endl;

      // TODO(JS): Initial guess
      // x
      _info->x[index] = 0.0;
      _info->x[index + 1] = 0.0;
      _info->x[index + 2] = 0.0;

      // Increase index
      index += 3;
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
      // _info->b[i] += bouncingVelocity;
      //      std::cout << "_lcp->b[_idx]: " << _lcp->b[_idx] << std::endl;

      // TODO(JS): Initial guess
      // x
      _info->x[i] = 0.0;

      // Increase index
    }
  }
}
}
}
