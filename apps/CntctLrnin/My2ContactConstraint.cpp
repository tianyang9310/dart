#include <fstream>
#include <iostream>

#include "My2ContactConstraint.h"
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

// #define CLAMP_CONTACT_CONSTRAINT

namespace dart {
namespace constraint {

My2ContactConstraint::My2ContactConstraint(collision::Contact& _contact,
                                           double _timestep)
    : ContactConstraint() {
  mTimeStep = (_timestep);
  mFirstFrictionalDirection = (Eigen::Vector3d::UnitZ());
  mIsFrictionOn = (true);
  mAppliedImpulseIndex = (-1);
  mIsBounceOn = (false);
  mActive = (false);
  numBasis = 2;

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
      // D = getTangentBasisMatrixLemke(ct->normal, numBasis);
      D = getTangentBasisMatrixODE(ct->normal);

      assert(std::abs(ct->normal.dot(D.col(0))) < DART_EPSILON);
      assert(std::abs(ct->normal.dot(D.col(1))) < DART_EPSILON);
      //      if (D.col(0).dot(D.col(1)) > 0.0)
      //        std::cout << "D.col(0).dot(D.col(1): " << D.col(0).dot(D.col(1))
      //        << std::endl;
      assert(std::abs(D.col(0).dot(D.col(1))) < DART_EPSILON);

      //      std::cout << "D: " << std::endl << D << std::endl;

      // Jacobian for normal contact
      bodyDirection1.noalias() =
          mBodyNode1->getTransform().linear().transpose() * ct->normal;
      bodyDirection2.noalias() =
          mBodyNode2->getTransform().linear().transpose() * -ct->normal;

      bodyPoint1.noalias() = mBodyNode1->getTransform().inverse() * ct->point;
      bodyPoint2.noalias() = mBodyNode2->getTransform().inverse() * ct->point;

      mJacobians1[idx].head<3>() = bodyPoint1.cross(bodyDirection1);
      mJacobians2[idx].head<3>() = bodyPoint2.cross(bodyDirection2);

      mJacobians1[idx].tail<3>() = bodyDirection1;
      mJacobians2[idx].tail<3>() = bodyDirection2;

      ++idx;

      // Jacobian for directional friction 1
      bodyDirection1.noalias() =
          mBodyNode1->getTransform().linear().transpose() * D.col(0);
      bodyDirection2.noalias() =
          mBodyNode2->getTransform().linear().transpose() * -D.col(0);

      //      bodyPoint1.noalias()
      //          = mBodyNode1->getWorldTransform().inverse() * ct->point;
      //      bodyPoint2.noalias()
      //          = mBodyNode2->getWorldTransform().inverse() * ct->point;

      //      std::cout << "bodyDirection2: " << std::endl << bodyDirection2 <<
      //      std::endl;

      mJacobians1[idx].head<3>() = bodyPoint1.cross(bodyDirection1);
      mJacobians2[idx].head<3>() = bodyPoint2.cross(bodyDirection2);

      mJacobians1[idx].tail<3>() = bodyDirection1;
      mJacobians2[idx].tail<3>() = bodyDirection2;

      ++idx;

      // Jacobian for directional friction 2
      bodyDirection1.noalias() =
          mBodyNode1->getTransform().linear().transpose() * D.col(1);
      bodyDirection2.noalias() =
          mBodyNode2->getTransform().linear().transpose() * -D.col(1);

      //      bodyPoint1.noalias()
      //          = mBodyNode1->getWorldTransform().inverse() * ct->point;
      //      bodyPoint2.noalias()
      //          = mBodyNode2->getWorldTransform().inverse() * ct->point;

      //      std::cout << "bodyDirection2: " << std::endl << bodyDirection2 <<
      //      std::endl;

      mJacobians1[idx].head<3>() = bodyPoint1.cross(bodyDirection1);
      mJacobians2[idx].head<3>() = bodyPoint2.cross(bodyDirection2);

      mJacobians1[idx].tail<3>() = bodyDirection1;
      mJacobians2[idx].tail<3>() = bodyDirection2;

      ++idx;
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

My2ContactConstraint::~My2ContactConstraint() {}
}
}
