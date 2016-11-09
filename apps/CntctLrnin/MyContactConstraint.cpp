#include <iostream>

#include "MyContactConstraint.h"
#include "dart/common/Console.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"

namespace dart{
namespace constraint{

MyContactConstraint::MyContactConstraint(collision::Contact& _contact,  double _timestep):ContactConstraint(_contact, _timestep)
{
    numBasis = 8;
}

MyContactConstraint::~MyContactConstraint()
{

}

void MyContactConstraint::MyapplyImpulse(double fn, const Eigen::VectorXd & fd, const Eigen::VectorXd &  N, const Eigen::MatrixXd & B, int BodyNode1_dim, int BodyNode2_dim)
{
//  std::cout<< "[Lemke LCP]Using MyContactConstraint class! "<<std::endl;
//  std::cin.get();

    if (mIsFrictionOn)
    {
        for (size_t i = 0; i < mContacts.size(); ++i)
        {
            // normal impulsive force and tangential direction impulsive force
            Eigen::VectorXd normal_impulsive_force;
            Eigen::VectorXd tangential_directional_force;

            normal_impulsive_force = N*fn;
            tangential_directional_force = B*fd;

            Eigen::MatrixXd D = getTangentBasisMatrixLemke(mContacts[i]->normal,numBasis);
            mContacts[i]->force = mContacts[i]->normal * fn / mTimeStep;
            mContacts[i]->force += D * fd / mTimeStep;

            // std::cout<<"[Lemke LCP] normal impulsive force is "<<std::endl<<normal_impulsive_force.transpose()<<std::endl;
            // std::cout<<"[Lemke LCP] tangential directional force is "<<std::endl<<tangential_directional_force.transpose()<<std::endl;
            if (mBodyNode1->isReactive())
            {
                mBodyNode1->addConstraintImpulse(normal_impulsive_force.head(BodyNode1_dim));
                mBodyNode1->addConstraintImpulse(tangential_directional_force.head(BodyNode1_dim));
            }
            
            if (mBodyNode2->isReactive())
            {
                mBodyNode2->addConstraintImpulse(normal_impulsive_force.tail(BodyNode2_dim));
                mBodyNode2->addConstraintImpulse(tangential_directional_force.tail(BodyNode2_dim));
            }
        }
    }
    else
    {
        for (size_t i = 0; i < mContacts.size(); ++i)
        {
            Eigen::VectorXd normal_impulsive_force;
            normal_impulsive_force = N*fn;
            // std::cout<<"[Lemke LCP] normal impulsive force is "<<std::endl<<normal_impulsive_force.transpose()<<std::endl;
            if (mBodyNode1->isReactive())
            {
                mBodyNode1->addConstraintImpulse(normal_impulsive_force.head(BodyNode1_dim));
            }
            
            if (mBodyNode2->isReactive())
            {
                mBodyNode2->addConstraintImpulse(normal_impulsive_force.tail(BodyNode2_dim));
            }
            
            mContacts[i]->force = mContacts[i]->normal * fn / mTimeStep;
        }
    }
}

void MyContactConstraint::applyImpulse(double* _lambda)
{
//  std::cout<< "[ODE LCP]Using MyContactConstraint class! "<<std::endl;
//  std::cin.get();

  //----------------------------------------------------------------------------
  // Friction case
  //----------------------------------------------------------------------------
  if (mIsFrictionOn)
  {
    size_t index = 0;

    for (size_t i = 0; i < mContacts.size(); ++i)
    {
//      std::cout << "_lambda1: " << _lambda[_idx] << std::endl;
//      std::cout << "_lambda2: " << _lambda[_idx + 1] << std::endl;
//      std::cout << "_lambda3: " << _lambda[_idx + 2] << std::endl;

//      std::cout << "imp1: " << mJacobians2[i * 3 + 0] * _lambda[_idx] << std::endl;
//      std::cout << "imp2: " << mJacobians2[i * 3 + 1] * _lambda[_idx + 1] << std::endl;
//      std::cout << "imp3: " << mJacobians2[i * 3 + 2] * _lambda[_idx + 2] << std::endl;

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

      std::cout<<"[ODE LCP] normal impulsive force is "<<std::endl<<(mJacobians1[i]*_lambda[0]).transpose()<<std::endl<<(mJacobians2[i]*_lambda[0]).transpose()<<std::endl;
      std::cout<<std::boolalpha;
      std::cout<<"mBodyNode1 isReactive: "<<mBodyNode1->isReactive()<<" mBodyNode2 isReactive: "<<mBodyNode2->isReactive()<<std::endl;
      std::cout<<"[ODE LCP] tangential directional force is "<<std::endl<<(mJacobians1[1] * _lambda[1]).transpose()<<std::endl<<(mJacobians2[1] * _lambda[1]).transpose()<<std::endl;
      std::cout<<"[ODE LCP] tangential directional force is "<<std::endl<<(mJacobians1[2] * _lambda[2]).transpose()<<std::endl<<(mJacobians2[2] * _lambda[2]).transpose()<<std::endl;
    }
  }
  //----------------------------------------------------------------------------
  // Frictionless case
  //----------------------------------------------------------------------------
  else
  {
    for (size_t i = 0; i < mContacts.size(); ++i)
    {
      // Normal impulsive force
//			pContactPts[i]->lambda[0] = _lambda[i];
      if (mBodyNode1->isReactive())
        mBodyNode1->addConstraintImpulse(mJacobians1[i] * _lambda[i]);

      if (mBodyNode2->isReactive())
        mBodyNode2->addConstraintImpulse(mJacobians2[i] * _lambda[i]);

      // Store contact impulse (force) toward the normal w.r.t. world frame
      mContacts[i]->force = mContacts[i]->normal * _lambda[i] / mTimeStep;

      // std::cout<<"[ODE LCP] normal impulsive force is "<<std::endl<<(mJacobians1[i]*_lambda[i]).transpose()<<std::endl<<(mJacobians2[i]*_lambda[i]).transpose()<<std::endl;
      // std::cout<<std::boolalpha;
      // std::cout<<"mBodyNode1 isReactive: "<<mBodyNode1->isReactive()<<" mBodyNode2 isReactive: "<<mBodyNode2->isReactive()<<std::endl;
    }
  }
}

}
}
