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

#ifndef APPS_SWING_CONTROLLER_H_
#define APPS_SWING_CONTROLLER_H_

#include <vector>

#include <Eigen/Dense>
#include <caffe/caffe.hpp>

#include "dart/dart.h"
#include "cNNsolver.h"


class Controller {
public:
  Controller(dart::dynamics::SkeletonPtr _skel, dart::constraint::ConstraintSolver* _constrSolver,
             double _t);
  virtual ~Controller();

  Eigen::VectorXd getTorques();
  double getTorque(int _index);
  void setDesiredDof(int _index, double _val);
  void computeTorques(int _currentFrame);
  void setState(std::string _state);
  std::string getState();
  dart::dynamics::SkeletonPtr getSkel();
  Eigen::VectorXd getDesiredDofs();
  Eigen::MatrixXd getKp();
  Eigen::MatrixXd getKd();
  Eigen::MatrixXi* mVision;
  void GeneratePose();

  Eigen::Vector3d TargetAngs;

  double getPlatformDis();
  double getPlatformVel();
  double LastplatformP = 0;
  double LastplatformV;
  void setUpperBodyPD(double kp, double kd);
  void setLowerBodyPD(double kp, double kd);



protected:
  void stand();
  void crouch();
  void jump();
  void reach();
  void grab();
  void swing();
  void release();
  void fly();
  void ReachingGround();
  void StandingUp();
  // Basic control building blocks
  void stablePD();
  void ankleStrategy();
  void anklehipStrategy();

  void virtualForce(Eigen::Vector3d _force, dart::dynamics::BodyNode* _bodyNode, Eigen::Vector3d _offset = Eigen::Vector3d::Zero());
  // Contact states
  void checkContactState();
  void CheckSwingPhase(); 
  void leftHandGrab();
  void rightHandGrab();
  void leftHandRelease();
  void rightHandRelease();

  dart::dynamics::SkeletonPtr mSkel;
  dart::constraint::ConstraintSolver* mConstraintSolver;
  Eigen::VectorXd mTorques;
  Eigen::VectorXd mDefaultPose;
  Eigen::VectorXd mDesiredDofs;
  Eigen::MatrixXd mKp;
  Eigen::MatrixXd mKd;
  double mTimestep;
  double mPreOffset;
  int mTimer;
  std::string mState;
  std::string mSwingState;
  dart::constraint::JointConstraint* mLeftHandHold;
  dart::constraint::JointConstraint* mRightHandHold;
  dart::dynamics::BodyNode* mFootContact;
  dart::dynamics::BodyNode* mToeContact;

  dart::dynamics::BodyNodePtr mLeftHandContact;
  dart::dynamics::BodyNodePtr mRightHandContact;
  int mCurrentFrame;
  int mSwingFrame;
  std::shared_ptr<cNNSolver> mNN;
};

#endif  // APPS_SWING_CONTROLLER_H_
