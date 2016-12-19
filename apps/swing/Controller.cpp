/*
 * Copyright (c) 2011-2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "Controller.h"
#define USING_COM_CHEKC_SWING_PHASE

Controller::Controller(dart::dynamics::SkeletonPtr _skel, dart::constraint::ConstraintSolver* _constrSolver, double _t) {
  mSkel = _skel;
  mConstraintSolver = _constrSolver;
  mTimestep = _t;

  int nDof = mSkel->getNumDofs();
  mKp = Eigen::MatrixXd::Identity(nDof, nDof);
  mKd = Eigen::MatrixXd::Identity(nDof, nDof);
  mTorques.resize(nDof);
  mDefaultPose.resize(nDof);
  mDesiredDofs.resize(nDof);
  
  // Set default pose as the initial pose when the controller is instantiated
  mDefaultPose = mSkel->getPositions();
  mDesiredDofs = mDefaultPose;
  
  mTorques.setZero();

  // Using SPD results in simple spring coefficients
  for (int i = 0; i < nDof; i++)
    mKp(i, i) = 400.0;
  for (int i = 0; i < nDof; i++)
    mKd(i, i) = 40.0;

  // Global dofs don't have PD control
  for (int i = 0; i < 6; i++) {
    mKp(i, i) = 0.0;
    mKd(i, i) = 0.0;
  }

  // Make shoulders and elbows loose
  std::vector<int> dofIndex;
  dofIndex.push_back((mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_bicep_left_x")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_forearm_left")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_bicep_right_x")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_forearm_right")->getIndexInSkeleton()));
  for (int i = 0; i < dofIndex.size(); i++) {
    int index = dofIndex[i];
    mKp(index, index) = 20.0;
    mKd(index, index) = 2.0;
  }

  // Make wrists even looser
  dofIndex.clear();
  dofIndex.push_back((mSkel->getDof("j_hand_left_1")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_hand_left_2")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_hand_right_1")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_hand_right_2")->getIndexInSkeleton()));
  for (int i = 0; i < dofIndex.size(); i++) {
    int index = dofIndex[i];
    mKp(index, index) = 1.0;
    mKd(index, index) = 0.1;
  }

  for (int i = 0; i < nDof; i++)
    mSkel->getDof(i)->setDampingCoefficient(0.01);
  mPreOffset = 0.0;
  mLeftHandHold = NULL;
  mRightHandHold = NULL;
  mFootContact = NULL;
  mLeftHandContact = NULL;
  mRightHandContact = NULL;
  mTimer = 300;
  mState = "STAND";
  mSwingState = "NULL";
  mVision = new Eigen::MatrixXi(480,640);
  (*mVision).setZero();
}

Controller::~Controller() {
  delete mVision;
}

Eigen::VectorXd Controller::getTorques() {
  return mTorques;
}

double Controller::getTorque(int _index) {
  return mTorques[_index];
}

void Controller::setDesiredDof(int _index, double _val) {
  mDesiredDofs[_index] = _val;
}

void Controller::computeTorques(int _currentFrame) {
  mCurrentFrame = _currentFrame;
  mTorques.setZero();
  if (mState == "STAND") {
    stand();
  } else if (mState == "CROUCH") {
    crouch();
  } else if (mState == "JUMP") {
    jump();
  } else if (mState == "REACH") {
    reach();
  } else if (mState == "GRAB") {
    grab();
  } else if (mState == "RELEASE") {
    release();
  } else if (mState == "SWING") {
    swing();
  } else if (mState == "ReachingGround") {
    ReachingGround();
  } else if (mState =="StandingUp") {
    StandingUp();
  } else if (mState =="fly"){
    fly();
  } else {
    std::cout << "Illegal state: " << mState << std::endl;
  }

  // Just to make sure no illegal torque is used. Do not remove this.
  for (int i = 0; i < 6; i++) {
    mTorques[i] = 0.0;
  }
}

void Controller::checkContactState() {
  mFootContact = NULL;
  mToeContact = NULL;
  mLeftHandContact = NULL;
  mRightHandContact = NULL;
  dart::collision::CollisionDetector* cd = mConstraintSolver->getCollisionDetector();
  int nContacts = cd->getNumContacts();
  for (int i = 0; i < nContacts; i++) {
    dart::dynamics::BodyNodePtr body1 = cd->getContact(i).bodyNode1.lock().get();
    dart::dynamics::BodyNodePtr body2 = cd->getContact(i).bodyNode2.lock().get();
  
    if (body1 == mSkel->getBodyNode("h_heel_left") || body1 == mSkel->getBodyNode("h_toe_left")
        || body1 == mSkel->getBodyNode("h_heel_right") || body1 == mSkel->getBodyNode("h_toe_right"))
      mFootContact = body2;

    if (body2 == mSkel->getBodyNode("h_heel_left") || body2 == mSkel->getBodyNode("h_toe_left")
        || body2 == mSkel->getBodyNode("h_heel_right") || body2 == mSkel->getBodyNode("h_toe_right"))
      mFootContact = body1;

    // check toe contact
    if (body1 == mSkel->getBodyNode("h_toe_left") || body1 == mSkel->getBodyNode("h_toe_right"))
      mToeContact = body2;

    if (body2 == mSkel->getBodyNode("h_toe_left") || body2 == mSkel->getBodyNode("h_toe_right"))
      mToeContact = body1;

    if (body1->isCollidable() && body1 == mSkel->getBodyNode("h_hand_left"))
      mLeftHandContact = body2;
    if (body2->isCollidable() && body2 == mSkel->getBodyNode("h_hand_left"))
      mLeftHandContact = body1;
    if (body1->isCollidable() && body1 == mSkel->getBodyNode("h_hand_right"))
      mRightHandContact = body2;
    if (body2->isCollidable() && body2 == mSkel->getBodyNode("h_hand_right"))
      mRightHandContact = body1;
  }
}

void Controller::stand() {
  // Change to default standing pose
  mDesiredDofs = mDefaultPose;
  stablePD();
  ankleStrategy();
  mTimer--;

  // Switch to crouch if time is up
  if (mTimer == 0) {
    mState = "CROUCH";
    mTimer = 500;
    std::cout << mCurrentFrame << ": " << "STAND -> CROUCH" << std::endl;
  }
}

void Controller::crouch() {

  // Change to crouching pose
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = 0.7;
  mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = 0.7;
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = -1.1;
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = -1.1;
  mDesiredDofs[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] = 0.6;
  mDesiredDofs[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] = 0.6;
  mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = -0.2;

  // After a while, lean forward
  if (mTimer < 200) {
    mDesiredDofs[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] = 1.0;
    mDesiredDofs[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] = 1.0;
  }

  stablePD();
  ankleStrategy();
  mTimer--;

  if (mTimer == 0) {
    mState = "JUMP";
    std::cout << mCurrentFrame << ": " << "CROUCH -> JUMP" << std::endl;

  }
}

void Controller::jump() {
  // Change to leaping pose
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = 0.2;
  mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = 0.2;
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()] = 0.3;
  mDesiredDofs[mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()] = -1.0;
  mDesiredDofs[mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()] = 0.3;
  mDesiredDofs[mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()] = 1.0;
  mDesiredDofs[mSkel->getDof("j_forearm_left")->getIndexInSkeleton()] = 0.5;
  mDesiredDofs[mSkel->getDof("j_forearm_right")->getIndexInSkeleton()] = 0.5;
  
  stablePD();

  // Use Jacobian transpose to compute pushing torques
  Eigen::Vector3d vf(-1100.0, -2600, 0.0);
  Eigen::Vector3d offset(0.05, -0.02, 0.0);
  virtualForce(vf, mSkel->getBodyNode("h_heel_left"), offset);
  virtualForce(vf, mSkel->getBodyNode("h_heel_right"), offset);

  checkContactState();
  if (mFootContact == NULL) {
    mState = "REACH";
    std::cout << mCurrentFrame << ": " << "JUMP -> REACH" << std::endl;
  }
}

void Controller::reach() {
  // Change to reaching pose
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = 0.2;
  mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = 0.2;
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()] = 0.7;
  mDesiredDofs[mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()] = -2.3;
  mDesiredDofs[mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()] = 0.7;
  mDesiredDofs[mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()] = 2.3;
  mDesiredDofs[mSkel->getDof("j_forearm_left")->getIndexInSkeleton()] = 0.4;
  mDesiredDofs[mSkel->getDof("j_forearm_right")->getIndexInSkeleton()] = 0.4;
  stablePD();

  checkContactState();
  if (mFootContact) { // If feet are in contact again, go back to JUMP and continue to push
    mState = "JUMP";
    std::cout << mCurrentFrame << ": " << "REACH -> JUMP" << std::endl;
  } else if (mLeftHandContact || mRightHandContact) {
    mState = "GRAB";
    mTimer = 500;  // changed from 500 to 5;
    std::cout << mCurrentFrame << ": " << "REACH -> GRAB" << std::endl;
  } else {
    mState = "REACH";
  }
}

void Controller::grab() {
  leftHandGrab();
  rightHandGrab();

  mDesiredDofs = mDefaultPose;
  mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = 0.2;
  mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = 0.2;
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = -0.2;

  mDesiredDofs[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()] = 0.7;
  mDesiredDofs[mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()] = -2.3;
  mDesiredDofs[mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()] = 0.7;
  mDesiredDofs[mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()] = 2.3;
  mDesiredDofs[mSkel->getDof("j_forearm_left")->getIndexInSkeleton()] = 0.4;
  mDesiredDofs[mSkel->getDof("j_forearm_right")->getIndexInSkeleton()] = 0.4;
  /*
  Eigen::Vector3d FlyTargetLow;
  FlyTargetLow << 0.8,-0.5,-1.3;
  mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = FlyTargetLow(0);
  mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = FlyTargetLow(1);
  mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = FlyTargetLow(1);
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = FlyTargetLow(2);
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = FlyTargetLow(2);
  */
  //setUpperBodyPD(25,5);
  //setLowerBodyPD(300,30);
  stablePD();
  mTimer--;
  if (LastplatformP != 0) {
    double temp = getPlatformVel();
    if (temp != 0)
  LastplatformV = temp;
  }
  LastplatformP = getPlatformDis(); // initialize pos so that can call vel.
  if (mTimer == 0) {
    mState = "SWING";

    mSwingFrame = mCurrentFrame;
    std::cout << mCurrentFrame << ": " << "GRAB -> SWING" << std::endl;
  }
}  

void Controller::CheckSwingPhase() {
#ifdef USING_COM_CHEKC_SWING_PHASE
  // using COM to check phase
  double bias = 0.85;
  double pos = mSkel->getCOM()(0) - bias;
  double vel = mSkel->getCOMLinearVelocity()(0);
  //std::cout << "j_abdomen_2: " << mSkel->getPosition(mSkel->getDof("j_abdomen_2")->getIndexInSkeleton())<<std::endl;
  // std::cout<<"mCurrentFrame: "<< mCurrentFrame<<" COM pos: "<<pos<<" COM vel: "<<vel<<std::endl;

  if ( pos > 0 &&  vel >0) {
    mSwingState = "Fwd_Pos_Fwd_Vel";
  } else if ( pos >0 &&  vel <0) {
    mSwingState = "Fwd_Pos_Bwd_Vel";
    // std::cout << "mSwingFrame: " << mSwingFrame <<std::endl;
    // std::cin.get();
  } else if ( pos <0 &&  vel <0) {
    mSwingState = "Bwd_Pos_Bwd_Vel";
  } else if ( pos <0 &&  vel >0) {
    mSwingState = "Bwd_Pos_Fwd_Vel";
  } else {
    mSwingState = "NULL";
  }
#else 
 // using wrist to check phase
  double bias = -0.3675;
  double pos = mSkel->getPositions().transpose()(mSkel->getDof("j_hand_left_1")->getIndexInSkeleton()) - bias;
  double vel = mSkel->getVelocities().transpose()(mSkel->getDof("j_hand_left_1")->getIndexInSkeleton());
  std::cout<<"wrist pos: "<<pos<<" wrist vel: "<<vel<<std::endl;

  if ( pos < 0 &&  vel <0) {
    mSwingState = "Fwd_Pos_Fwd_Vel";
  } else if ( pos <0 &&  vel >0) {
    mSwingState = "Fwd_Pos_Bwd_Vel";
  } else if ( pos >0 &&  vel >0) {
    mSwingState = "Bwd_Pos_Bwd_Vel";
  } else if ( pos >0 &&  vel <0) {
    mSwingState = "Bwd_Pos_Fwd_Vel";
  } else {
    mSwingState = "NULL";
  }
#endif
}

void Controller::GeneratePose(){
  //generate joint angles for swing phase
  double posX = mSkel->getCOM()(0) - 0.85;
  double posY = 1- mSkel->getCOM()(1) ;
  //Ball Angle;
  double ang = atan(posX/posY);
  //std::cout<<"comPx "<< posX << "comPy "<< posY<<std::endl;
  //std::cout<< "ang " << ang<< std::endl;
  Eigen::Vector3d TargetAngs;
  // generate 3 actuation angles based on com angle
  TargetAngs(0) = -2.0*ang; // abdom
  TargetAngs(1) = 2.0*ang; // thigh
  TargetAngs(2) = 2.0*ang; // ankle
  mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = TargetAngs(0);//-1.2;
  mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = TargetAngs(1);
  mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = TargetAngs(1);
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = TargetAngs(2);
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = TargetAngs(2);
}

void Controller::swing() {
  // ===========================================================================
  // change forearm Kp and Kd
  setUpperBodyPD(2,.5);
  setLowerBodyPD(20,2);
  // ===========================================================================

  // TODO: Need a better controller to increase the speed
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()] = -1;
  mDesiredDofs[mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()] = -2.6;
  mDesiredDofs[mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()] = -1;
  mDesiredDofs[mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()] = 2.6;

  //forearm ...> 0
  mDesiredDofs[mSkel->getDof("j_forearm_left")->getIndexInSkeleton()] = 1.4;
  mDesiredDofs[mSkel->getDof("j_forearm_right")->getIndexInSkeleton()] = 1.4;

  //GeneratePose();

  CheckSwingPhase();
  double posX = mSkel->getCOM()(0) - 0.85;
  double posY = 1 - mSkel->getCOM()(1) ;
  double ang = atan(posX/posY);
  Eigen::Vector3d ComVel = mSkel->getCOMLinearVelocity(); //idx0 is forward, idx1 is upward

  if (mSwingState == "Fwd_Pos_Fwd_Vel") {
    //TargetAngs<< 0,0,0;
    TargetAngs<< -1.5,1.3,-0.2;
  } else if (mSwingState == "Fwd_Pos_Bwd_Vel") {
    TargetAngs<<0,0,0;
  } else if (mSwingState == "Bwd_Pos_Bwd_Vel") {
    //TargetAngs<<0,0,0;
    TargetAngs<<1.5,-1.3,-1.2;
  } else if (mSwingState == "Bwd_Pos_Fwd_Vel") {
    TargetAngs<< 0,0,0;
  } else {
    // NULL mSwingState 
    // pass
  }

  mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = TargetAngs(0);
  mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = TargetAngs(1);
  mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = TargetAngs(1);
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = TargetAngs(2);
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = TargetAngs(2);
  stablePD();


  // TODO: Figure out the condition to release the bar
  double angle = atan(ComVel(0)/ComVel(1));
  // std::cout << "ang "<< ang<< std::endl;
  //std::cout<<"angle "<< angle<< std::endl;
  double platform_vel = getPlatformVel();
   if (platform_vel == 0) platform_vel = LastplatformV; // low sampling 60HZ vision
  LastplatformV = platform_vel;
  //std::cout<<"platPos"<<LastplatformP<< "platVel " << platform_vel<< std::endl;

  bool nearEnough = LastplatformP < 2.78;
  bool coming = platform_vel < 0;

  bool RightState = (mSwingState == "Fwd_Pos_Fwd_Vel");

  bool goodAng1 = (ang > 20.0/180.0*3.14);
  bool goodAng2 = (ang < 10.0/180.0*3.14);
  if (nearEnough && coming && RightState && goodAng1){// && goodAng1 && goodAng2) { // condition about platPos platVel, and Character States
    mState = "RELEASE";
    mTimer = 32;
    setUpperBodyPD(400,40);
    setLowerBodyPD(400,40);
    std::cout << mCurrentFrame << ": " << "SWING -> RELEASE" << std::endl;
  }
}

void Controller::release() {

  Eigen::Vector3d vf(-800,-800,0); //make sure upper body lose
  Eigen::Vector3d offset(0, 0, 0.);
  virtualForce(vf, mSkel->getBodyNode("h_hand_right"), offset);
  virtualForce(vf, mSkel->getBodyNode("h_hand_left"), offset);

  setUpperBodyPD(400,40);
  setLowerBodyPD(400,40);
  // mDesiredDofs = mDefaultPose;
  Eigen::Vector3d FlyTargetLow;
  FlyTargetLow << 0,0,-1.3;
  mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = FlyTargetLow(0);
  mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = FlyTargetLow(1);
  mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = FlyTargetLow(1);
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = FlyTargetLow(2);
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = FlyTargetLow(2);
  stablePD();
  mTimer --;

  double platform_vel = getPlatformVel();
  if (platform_vel == 0) platform_vel = LastplatformV; // low sampling 60HZ vision
  LastplatformV = platform_vel;
  //std::cout<<"platPos"<<LastplatformP<< "platVel " << platform_vel<< std::endl;
  bool coming = platform_vel < 0;

  if (mTimer ==0) {
    if  (LastplatformP < 3 ) {// dummy var
      mState = "fly";
      std::cout << "last pos: " << LastplatformP << std::endl;
      std::cout << mCurrentFrame << ": " << "Release -> fly" << std::endl;
      leftHandRelease();
      rightHandRelease();
    } else {
      mState = "SWING";
      mSwingFrame = mCurrentFrame;
      std::cout << "last pos: " << LastplatformP << std::endl;
      std::cout << mCurrentFrame << ": " << "Release -> SWING" << std::endl;
    }
  }
}

void Controller::fly() {
  setUpperBodyPD(400,40);
  setLowerBodyPD(400,40);
  // mDesiredDofs = mDefaultPose;
  Eigen::Vector3d FlyTargetLow;
  FlyTargetLow << 0,1.5,-1.3;
  mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = FlyTargetLow(0);
  mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = FlyTargetLow(1);
  mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = FlyTargetLow(1);
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = FlyTargetLow(2);
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = FlyTargetLow(2);

  Eigen::Vector4d FlyTargetUp;
  FlyTargetUp << 0.3,1.45,0,0;
  mDesiredDofs[mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()] = FlyTargetUp(0);
  mDesiredDofs[mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()] = FlyTargetUp(0);
  mDesiredDofs[mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()] = -FlyTargetUp(1);
  mDesiredDofs[mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()] = FlyTargetUp(1);

  mDesiredDofs[mSkel->getDof("j_bicep_left_x")->getIndexInSkeleton()] = FlyTargetUp(2);
  mDesiredDofs[mSkel->getDof("j_bicep_right_x")->getIndexInSkeleton()] = -FlyTargetUp(2);

  mDesiredDofs[mSkel->getDof("j_forearm_right")->getIndexInSkeleton()] = FlyTargetUp(3);
  mDesiredDofs[mSkel->getDof("j_forearm_left")->getIndexInSkeleton()] = FlyTargetUp(3);

  stablePD();

  checkContactState();
  if (mFootContact) {
    mState = "ReachingGround";
    std::cout << mCurrentFrame << ": " << "fly -> ReachingGround" << std::endl;
  }

}
void Controller::ReachingGround(){
  checkContactState();

  setUpperBodyPD(200,20);
  setLowerBodyPD(200,20);
  Eigen::Vector4d FlyTargetLow;
  FlyTargetLow << 0,0.5,-1.3,0.5;
  mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = FlyTargetLow(0);
  mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = FlyTargetLow(1);
  mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = FlyTargetLow(1);
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = FlyTargetLow(2);
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = FlyTargetLow(2);
  mDesiredDofs[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] = FlyTargetLow(3);
  mDesiredDofs[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] = FlyTargetLow(3);

  int idx1 = mSkel->getDof("j_toe_left")->getIndexInSkeleton();
  int idx2 = mSkel->getDof("j_toe_right")->getIndexInSkeleton();
  mKp(idx1, idx1) = 300;  mKd(idx1, idx1) = 100;
  mKp(idx2, idx2) = 300;  mKd(idx2, idx2) = 100;
  stablePD();

  if (mFootContact && mToeContact) {
    mState = "StandingUp";
    std::cout << mCurrentFrame << ": " << "ReachingGround->StandingUp" << std::endl;
  }
}

void Controller::StandingUp(){

  setUpperBodyPD(400,40);
  setLowerBodyPD(200,40);
  Eigen::Vector4d FlyTargetLow;

  FlyTargetLow << -1.6,1.4,-2.7,0.9;
  //FlyTargetLow << -1.1,1.2,-2.4,0.8;
  mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = FlyTargetLow(0);
  mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = FlyTargetLow(1);
  mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = FlyTargetLow(1);
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = FlyTargetLow(2);
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = FlyTargetLow(2);
  mDesiredDofs[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] = FlyTargetLow(3);
  mDesiredDofs[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] = FlyTargetLow(3);
  mDesiredDofs[mSkel->getDof("j_toe_left")->getIndexInSkeleton()] = 0;
  mDesiredDofs[mSkel->getDof("j_toe_right")->getIndexInSkeleton()] = 0;

  mDesiredDofs[mSkel->getDof("j_forearm_right")->getIndexInSkeleton()] = 1;
  mDesiredDofs[mSkel->getDof("j_forearm_left")->getIndexInSkeleton()] = 1;

  int idx1 = mSkel->getDof("j_toe_left")->getIndexInSkeleton();
  int idx2 = mSkel->getDof("j_toe_right")->getIndexInSkeleton();
  mKp(idx1, idx1) = 500;  mKd(idx1, idx1) = 100;
  mKp(idx2, idx2) = 500;  mKd(idx2, idx2) = 100;
  //anklehipStrategy();
  stablePD();

}


void Controller::stablePD() {
  Eigen::VectorXd q = mSkel->getPositions();
  Eigen::VectorXd dq = mSkel->getVelocities();
  Eigen::VectorXd constrForces = mSkel->getConstraintForces();
  Eigen::MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
  Eigen::VectorXd p = -mKp * (q + dq * mTimestep - mDesiredDofs);
  Eigen::VectorXd d = -mKd * dq;
  Eigen::VectorXd qddot =
      invM * (-mSkel->getCoriolisAndGravityForces() + p + d + constrForces);

  mTorques += p + d - mKd * qddot * mTimestep;
}

void Controller::ankleStrategy() {
  Eigen::Vector3d com = mSkel->getCOM();
  Eigen::Vector3d cop = mSkel->getBodyNode("h_heel_left")->getTransform()
                        * Eigen::Vector3d(0.05, 0, 0);
  double offset = com[0] - cop[0];
   if (offset < 0.1 && offset > 0.0) {
    double k1 = 200.0;
    double k2 = 100.0;
    double kd = 10.0;
    mTorques[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_toe_left")->getIndexInSkeleton()] += -k2 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_toe_right")->getIndexInSkeleton()] += -k2 * offset + kd * (mPreOffset - offset);
    mPreOffset = offset;
  } else if (offset > -0.2 && offset < -0.05) {
    double k1 = 2000.0;
    double k2 = 100.0;
    double kd = 100.0;
    mTorques[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_toe_left")->getIndexInSkeleton()] += -k2 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_toe_right")->getIndexInSkeleton()] += -k2 * offset + kd * (mPreOffset - offset);
    mPreOffset = offset;
  }  
}

void Controller::anklehipStrategy() {
  Eigen::Vector3d com = mSkel->getCOM();
  Eigen::Vector3d cop = mSkel->getBodyNode("h_heel_left")->getTransform()
                        * Eigen::Vector3d(0.05, 0, 0);
  double offset = com[0] - cop[0];
  //std::cout<< "offset"<<offset<< std::endl;
  double leftbound = -0.04; double rightbound = 0.035;
  double forceX = 200; double forceY = -200;
  if (offset < rightbound && offset > leftbound) {
    double k1 = 100.0;
    double k2 = 50.0;
    double kd = 10.0;
    mTorques[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_toe_left")->getIndexInSkeleton()] += -k2 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_toe_right")->getIndexInSkeleton()] += -k2 * offset + kd * (mPreOffset - offset);
    mPreOffset = offset;
  }else if (offset > rightbound){
    //mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = 1.2 - 1.2*(offset>0);
    //mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = 1.2 - 1.2*(offset>0);

    Eigen::Vector3d vf(forceX,forceY,0);
    Eigen::Vector3d offset(0.05, -0.025, 0.);
    virtualForce(vf, mSkel->getBodyNode("h_toe_right"), offset);
    virtualForce(vf, mSkel->getBodyNode("h_toe_left"), offset);

  }
 else if (offset > -0.2 && offset < leftbound) {
    //mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = 1.2-1.2*(offset>0);
    //mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = 1.2-1.2*(offset>0);
    Eigen::Vector3d vf(-forceX,forceY,0);
    Eigen::Vector3d offset(-0.054, -0.027, 0.);
    virtualForce(1.5*vf, mSkel->getBodyNode("h_heel_right"), offset);
    virtualForce(1.5*vf, mSkel->getBodyNode("h_heel_left"), offset);

  }
  mPreOffset = offset;
}

void Controller::virtualForce(Eigen::Vector3d _force, dart::dynamics::BodyNode* _bodyNode, Eigen::Vector3d _offset) {
  Eigen::MatrixXd jacobian = mSkel->getLinearJacobian(_bodyNode, _offset);
  // std::cout<<(jacobian.transpose() * _force).transpose()<<std::endl;
  mTorques += jacobian.transpose() * _force;
}

void Controller::leftHandGrab() {  
  if (mLeftHandHold != NULL)
    return;
  checkContactState();
  if (mLeftHandContact == NULL)
    return;
  dart::dynamics::BodyNode* bd = mSkel->getBodyNode("h_hand_left");
  dart::constraint::WeldJointConstraint *hold = new dart::constraint::WeldJointConstraint(bd, mLeftHandContact);
  mConstraintSolver->addConstraint(hold);
  bd->setCollidable(false);
  mLeftHandHold = hold;
}

void Controller::leftHandRelease() {
  if (mLeftHandHold == NULL)
    return;
  mConstraintSolver->removeConstraint(mLeftHandHold);
  mSkel->getBodyNode("h_hand_left")->setCollidable(true);
  mLeftHandHold = NULL;
}

void Controller::rightHandGrab() {  
  if (mRightHandHold != NULL)
    return;

  checkContactState();
  if (mRightHandContact == NULL)
    return;
  dart::dynamics::BodyNode* bd = mSkel->getBodyNode("h_hand_right");
  dart::constraint::WeldJointConstraint *hold = new dart::constraint::WeldJointConstraint(bd, mRightHandContact);
  mConstraintSolver->addConstraint(hold);
  bd->setCollidable(false);
  mRightHandHold = hold;
}

void Controller::rightHandRelease() {
  if (mRightHandHold == NULL)
    return;
  mConstraintSolver->removeConstraint(mRightHandHold);
  mSkel->getBodyNode("h_hand_right")->setCollidable(true);
  mRightHandHold = NULL;
}

void Controller::setState(std::string _state) {
  mState = _state;
}

std::string Controller::getState() {
  return mState;
}

dart::dynamics::SkeletonPtr Controller::getSkel() {
  return mSkel;
}

Eigen::VectorXd Controller::getDesiredDofs() {
  return mDesiredDofs;
}

Eigen::MatrixXd Controller::getKp() {
  return mKp;
}

Eigen::MatrixXd Controller::getKd() {
  return mKd;
}

double Controller::getPlatformDis() {
  // using col(395) col(255)
  Eigen::VectorXi mBeam(480);
  mBeam = (*mVision).col(395);
  int BlackIdx=0;
  for (BlackIdx=0; BlackIdx<480; BlackIdx++) {
    if (mBeam(BlackIdx) < 128) {
      break;
    }
  }
  // std::cout<< "image boundary pos: " << BlackIdx << ",1" << std::endl;
  double PlatformDis;
  PlatformDis = -0.0164 * BlackIdx + 4.9659;
  return PlatformDis;
}

double Controller::getPlatformVel(){
  double currentPos = getPlatformDis();
  double Velocity = currentPos - LastplatformP;
  LastplatformP = currentPos;
  return Velocity;
}

void Controller::setUpperBodyPD(double kp, double kd){
  std::vector<int> Index;
  Index.push_back((mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()));
  Index.push_back((mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()));
  Index.push_back((mSkel->getDof("j_bicep_left_x")->getIndexInSkeleton()));
  Index.push_back((mSkel->getDof("j_forearm_left")->getIndexInSkeleton()));
  Index.push_back((mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()));
  Index.push_back((mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()));
  Index.push_back((mSkel->getDof("j_bicep_right_x")->getIndexInSkeleton()));
  Index.push_back((mSkel->getDof("j_forearm_right")->getIndexInSkeleton()));
  Index.push_back((mSkel->getDof("j_hand_left_1")->getIndexInSkeleton()));
  Index.push_back((mSkel->getDof("j_hand_left_2")->getIndexInSkeleton()));
  Index.push_back((mSkel->getDof("j_hand_right_1")->getIndexInSkeleton()));
  Index.push_back((mSkel->getDof("j_hand_right_2")->getIndexInSkeleton()));
  Index.push_back((mSkel->getDof("j_scapula_right")->getIndexInSkeleton()));
  Index.push_back((mSkel->getDof("j_scapula_left")->getIndexInSkeleton()));
  for (int i = 0; i < Index.size(); i++) {
    int index = Index[i];
    mKp(index, index) = kp;
    mKd(index, index) = kd;
  }
  Index.clear();

}
void Controller::setLowerBodyPD(double kp, double kd){
  std::vector<int> dof_Index;

  dof_Index.push_back((mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()));
  dof_Index.push_back((mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()));
  dof_Index.push_back((mSkel->getDof("j_shin_left")->getIndexInSkeleton()));
  dof_Index.push_back((mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()));
  dof_Index.push_back((mSkel->getDof("j_shin_right")->getIndexInSkeleton()));

  dof_Index.push_back((mSkel->getDof("j_toe_right")->getIndexInSkeleton()));
  dof_Index.push_back((mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()));
  dof_Index.push_back((mSkel->getDof("j_heel_right_2")->getIndexInSkeleton()));
  dof_Index.push_back((mSkel->getDof("j_toe_left")->getIndexInSkeleton()));
  dof_Index.push_back((mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()));
  dof_Index.push_back((mSkel->getDof("j_heel_left_2")->getIndexInSkeleton()));
  for (int i = 0; i < dof_Index.size(); i++) {
    int index = dof_Index[i];
    mKp(index, index) = kp;
    mKd(index, index) = kd;
  }
  dof_Index.clear();

}
