/*************************************************************************
    > File Name: Controller.h
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Tue Apr  5 10:09:48 2016
 ************************************************************************/

#ifndef CONTROLLER_H
#define CONTROLLER_H 

#include "dart/dart.h"
#include "StateMachine.h"
#include "State.h"
#include "TerminalCondition.h"

namespace nonlinear{

using namespace dart::common;
using namespace dart::utils;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;
using namespace dart::gui;


class Controller
{
public:
	Controller(const SkeletonPtr& biped);
	void update(double _currentTime);
	void buildStateMachines();
	StateMachine* createWalkingStateMachine();
//--------------------------------------------------------------------------------------  
	//void setTargetPositions(const Eigen::VectorXd& pose);
//--------------------------------------------------------------------------------------  
protected:
  SkeletonPtr mBiped;

  /// Current State Machine
  StateMachine* mCurrentStateMachine;

//--------------------------------------------------------------------------------------  
  /// Joint forces for the biped (output of the Controller)
  Eigen::VectorXd mForces;
  
  /// Control gains for the proportional error terms in the PD controller
  Eigen::MatrixXd mKp;

  /// Control gains for the derivative error terms in the PD controller
  Eigen::MatrixXd mKd;
//--------------------------------------------------------------------------------------  
};




} // namespace nonlinear
#endif
