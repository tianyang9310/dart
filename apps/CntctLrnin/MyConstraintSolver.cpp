#include "MyConstraintSolver.h"

#include "dart/collision/dart/DARTCollisionDetector.h"
#include "dart/collision/fcl_mesh/FCLMeshCollisionDetector.h"
#include "dart/common/Console.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/SoftBodyNode.h"
#ifdef HAVE_BULLET_COLLISION
#include "dart/collision/bullet/BulletCollisionDetector.h"
#endif
#include "dart/constraint/ConstrainedGroup.h"
#include "dart/constraint/ContactConstraint.h"
#include "dart/constraint/JointCoulombFrictionConstraint.h"
#include "dart/constraint/JointLimitConstraint.h"
#include "dart/constraint/SoftContactConstraint.h"

namespace CntctLrnin {

MyConstraintSolver::MyConstraintSolver(double _timeStep)
    : ConstraintSolver(_timeStep) {}

MyConstraintSolver::~MyConstraintSolver() {}

void MyConstraintSolver::updateConstraints() {
  // Clear previous active constraint list
  mActiveConstraints.clear();

  //----------------------------------------------------------------------------
  // Update manual constraints
  //----------------------------------------------------------------------------
  for (auto& manualConstraint : mManualConstraints) {
    manualConstraint->update();

    if (manualConstraint->isActive())
      mActiveConstraints.push_back(manualConstraint);
  }

  //----------------------------------------------------------------------------
  // Update automatic constraints: contact constraints
  //----------------------------------------------------------------------------
  mCollisionDetector->clearAllContacts();
  mCollisionDetector->detectCollision(true, true);

  // Destroy previous contact constraints
  for (const auto& contactConstraint : mContactConstraints)
    delete contactConstraint;
  mContactConstraints.clear();

  // Destroy previous soft contact constraints
  for (const auto& softContactConstraint : mSoftContactConstraints)
    delete softContactConstraint;
  mSoftContactConstraints.clear();

  // Create new contact constraints
  for (size_t i = 0; i < mCollisionDetector->getNumContacts(); ++i) {
    collision::Contact& ct = mCollisionDetector->getContact(i);

    if (isSoftContact(ct)) {
      mSoftContactConstraints.push_back(
          new SoftContactConstraint(ct, mTimeStep));
    } else {
#ifdef LEMKE_SOLVER
      mContactConstraints.push_back(new MyContactConstraint(ct, mTimeStep));
#else
      mContactConstraints.push_back(new ContactConstraint(ct, mTimeStep));
#endif
    }
  }

  // Add the new contact constraints to dynamic constraint list
  for (const auto& contactConstraint : mContactConstraints) {
    contactConstraint->update();

    if (contactConstraint->isActive())
      mActiveConstraints.push_back(contactConstraint);
  }

  // Add the new soft contact constraints to dynamic constraint list
  for (const auto& softContactConstraint : mSoftContactConstraints) {
    softContactConstraint->update();

    if (softContactConstraint->isActive())
      mActiveConstraints.push_back(softContactConstraint);
  }

  //----------------------------------------------------------------------------
  // Update automatic constraints: joint limit constraints
  //----------------------------------------------------------------------------
  // Destroy previous joint limit constraints
  for (const auto& jointLimitConstraint : mJointLimitConstraints)
    delete jointLimitConstraint;
  mJointLimitConstraints.clear();

  // Create new joint limit constraints
  for (const auto& skel : mSkeletons) {
    const size_t numBodyNodes = skel->getNumBodyNodes();
    for (size_t i = 0; i < numBodyNodes; i++) {
      dynamics::Joint* joint = skel->getBodyNode(i)->getParentJoint();

      if (joint->isDynamic() && joint->isPositionLimitEnforced())
        mJointLimitConstraints.push_back(new JointLimitConstraint(joint));
    }
  }

  // Add active joint limit
  for (auto& jointLimitConstraint : mJointLimitConstraints) {
    jointLimitConstraint->update();

    if (jointLimitConstraint->isActive())
      mActiveConstraints.push_back(jointLimitConstraint);
  }

  //----------------------------------------------------------------------------
  // Update automatic constraints: joint Coulomb friction constraints
  //----------------------------------------------------------------------------
  // Destroy previous joint limit constraints
  for (const auto& jointFrictionConstraint : mJointCoulombFrictionConstraints)
    delete jointFrictionConstraint;
  mJointCoulombFrictionConstraints.clear();

  // Create new joint limit constraints
  for (const auto& skel : mSkeletons) {
    const size_t numBodyNodes = skel->getNumBodyNodes();
    for (size_t i = 0; i < numBodyNodes; i++) {
      dynamics::Joint* joint = skel->getBodyNode(i)->getParentJoint();

      if (joint->isDynamic()) {
        const size_t dof = joint->getNumDofs();
        for (size_t i = 0; i < dof; ++i) {
          if (joint->getCoulombFriction(i) != 0.0) {
            mJointCoulombFrictionConstraints.push_back(
                new JointCoulombFrictionConstraint(joint));
            break;
          }
        }
      }
    }
  }

  // Add active joint limit
  for (auto& jointFrictionConstraint : mJointCoulombFrictionConstraints) {
    jointFrictionConstraint->update();

    if (jointFrictionConstraint->isActive())
      mActiveConstraints.push_back(jointFrictionConstraint);
  }
}
}  // namespace CntctLrnin
