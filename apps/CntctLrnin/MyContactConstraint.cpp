#include <iostream>

#include "MyContactConstraint.h"
#include "dart/common/Console.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/math/Helpers.h"

// #define OUTPUT

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
      normal_impulsive_force = N * fn;
      if (mBodyNode1->isReactive()) {
        mBodyNode1->addConstraintImpulse(
            normal_impulsive_force.head(BodyNode1_dim));
      }

      if (mBodyNode2->isReactive()) {
        mBodyNode2->addConstraintImpulse(
            normal_impulsive_force.tail(BodyNode2_dim));
      }

      mContacts[i]->force = mContacts[i]->normal * fn / mTimeStep;
    }
  }
}

void MyContactConstraint::My2LemkeapplyImpulse(
    double fn, const Eigen::VectorXd& fd, const Eigen::VectorXd& N,
    const Eigen::MatrixXd& B, int BodyNode1_dim, int BodyNode2_dim,
    bool impulse_flag) {
#ifndef OUTPUT
  return;
#endif

  std::cout << "============================================" << std::endl;
  std::cout << "[Lemke LCP] Using MyContactConstraint class! " << std::endl;
  // std::cin.get();

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

      std::cout << "[Lemke LCP] normal impulsive force is "
                << normal_impulsive_force.transpose() << std::endl;
      std::cout << "[Lemke LCP] tangential directional force is "
                << tangential_directional_force.transpose() << std::endl;
      std::cout << "[Lemke LCP] overall force is " << Myforce.transpose()
                << std::endl;
      // std::cout<<"[Lemke LCP] first force is "<<(mContacts[i]->normal * fn /
      // mTimeStep).transpose()<<std::endl;
      // std::cout<<"[Lemke LCP] second force is "<<( D * fd / mTimeStep
      // ).transpose()<<std::endl;
    }
  } else {
    dterr << "No implementation for frictionless case!" << std::endl;
  }
}

void MyContactConstraint::My2ODEapplyImpulse(double* _lambda) {
#ifndef OUTPUT
  return;
#endif
  std::cout << "============================================" << std::endl;
  std::cout << "[ODE LCP] Using MyContactConstraint class! " << std::endl;
  // std::cin.get();

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
      // std::cout<<"[ODE LCP] overall first is "<<( mContacts[i]->normal *
      // _lambda[0] / mTimeStep ).transpose()<<std::endl;
      // std::cout<<"[ODE LCP] overall second 1 is "<<( D.col(0) * _lambda[1] /
      // mTimeStep).transpose()<<std::endl;
      // std::cout<<"[ODE LCP] overall second 2 is "<<( D.col(0) * _lambda[2] /
      // mTimeStep).transpose()<<std::endl;
    }
  }
  //----------------------------------------------------------------------------
  // Frictionless case
  //----------------------------------------------------------------------------
  else {
    dterr << "No implementation for frictionless case!" << std::endl;
  }
}
}
}
