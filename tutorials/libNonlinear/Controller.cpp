/*************************************************************************
    > File Name: Controller.cpp
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Tue Apr  5 10:09:54 2016
 ************************************************************************/


#include "Controller.h"

namespace nonlinear{


Controller::Controller(const SkeletonPtr& biped)\
	:mBiped(biped)
{
  //mCoronalLeftHip  = mAtlasRobot->getDof("l_leg_hpx")->getIndexInSkeleton();
  //mCoronalRightHip = mAtlasRobot->getDof("r_leg_hpx")->getIndexInSkeleton();
  mSagitalLeftHip  = mBiped->getDof("j_thigh_left")->getIndexInSkeleton();
  mSagitalRightHip = mBiped->getDof("j_thigh_right")->getIndexInSkeleton();

  buildStateMachines();
}

void Controller::update(double _currentTime)
{
	mCurrentStateMachine->computeControlForce(mBiped->getTimeStep());
	/*
	for (size_t i=0;i<mBiped->getNumDofs();i++)
	{
		std::cout<<mBiped->getDof(i)->getForce()<<std::endl;
	}
	*/
}

void Controller::buildStateMachines()
{
	mCurrentStateMachine = createWalkingStateMachine();
	mCurrentStateMachine->begin(0.0);
}

StateMachine* Controller::createWalkingStateMachine()
{
  const double cd02 = 0.0;
  const double cv02 = 0.2;

  const double cd13 = 2.2;
  const double cv13 = 0.0;

  const double pelvis = DART_RADIAN * 0.0;//-4.75;  // angle b/w pelvis and torso

  const double swh02  =  0.40;  // swing hip
  const double swk02  = -1.10;  // swing knee
  //const double swa02  =  0.60;  // swing angle
  const double stk02  = -0.05;  // stance knee
  //const double sta02  =  0.00;  // stance ankle

  const double swh13  = -0.70;  // swing hip
  const double swk13  = -0.05;  // swing knee
  //const double swa13  =  0.15;  // swing angle
  const double stk13  = -0.10;  // stance knee
  //const double sta13  =  0.00;  // stance ankle

  StateMachine* sm = new StateMachine("walking");

  State* state0 = new State(mBiped, "0");
  State* state1 = new State(mBiped, "1");
  State* state2 = new State(mBiped, "2");
  State* state3 = new State(mBiped, "3");

  TerminalCondition* cond0 = new TimerCondition(state0, 0.3);
  TerminalCondition* cond1 = new BodyContactCondition(state1, mBiped->getBodyNode(std::string("h_foot_right")));
  TerminalCondition* cond2 = new TimerCondition(state2, 0.3);
  TerminalCondition* cond3 = new BodyContactCondition(state3, mBiped->getBodyNode(std::string("h_foot_left")));

  state0->setTerminalCondition(cond0);
  state1->setTerminalCondition(cond1);
  state2->setTerminalCondition(cond2);
  state3->setTerminalCondition(cond3);

  state0->setNextState(state1);
  state1->setNextState(state2);
  state2->setNextState(state3);
  state3->setNextState(state0);

  // Set stance foot
  state0->setStanceFootToLeftFoot();
  state1->setStanceFootToLeftFoot();
  state2->setStanceFootToRightFoot();
  state3->setStanceFootToRightFoot();

  // Set global desired pelvis angle
  state0->setDesiredPelvisGlobalAngleOnSagital(DART_RADIAN * 0.0);
  state1->setDesiredPelvisGlobalAngleOnSagital(DART_RADIAN * 0.0);
  state2->setDesiredPelvisGlobalAngleOnSagital(DART_RADIAN * 0.0);
  state3->setDesiredPelvisGlobalAngleOnSagital(DART_RADIAN * 0.0);
 // state0->setDesiredPelvisGlobalAngleOnCoronal(DART_RADIAN * 0.0);
 // state1->setDesiredPelvisGlobalAngleOnCoronal(DART_RADIAN * 0.0);
 // state2->setDesiredPelvisGlobalAngleOnCoronal(DART_RADIAN * 0.0);
 // state3->setDesiredPelvisGlobalAngleOnCoronal(DART_RADIAN * 0.0);

  // Set desired joint position
  //-- State 0
  //---- pelvis
  state0->setDesiredJointPosition("j_pelvis_rot", -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state0->setDesiredJointPosition("j_thigh_right", -swh02); // right hip
  state0->setDesiredJointPosition("j_shin_right", -swk02); // right knee
  //state0->setDesiredJointPosition("r_leg_aky", -swa02); // right ankle
  //---- stance leg
  state0->setDesiredJointPosition("j_shin_left", -stk02); // left knee
  //state0->setDesiredJointPosition("l_leg_aky", -sta02); // left ankle
  //---- arm
  //state0->setDesiredJointPosition("l_arm_shy", DART_RADIAN * -20.00); // left arm
  //state0->setDesiredJointPosition("r_arm_shy", DART_RADIAN * +10.00); // right arm
  //state0->setDesiredJointPosition("l_arm_shx", DART_RADIAN * -80.00); // left arm
  //state0->setDesiredJointPosition("r_arm_shx", DART_RADIAN * +80.00); // right arm
  //---- feedback gain for hip joints
  //state0->setFeedbackCoronalCOMDistance(mCoronalLeftHip,  -cd); // coronal left hip
  //state0->setFeedbackCoronalCOMVelocity(mCoronalLeftHip,  -cv); // coronal left hip
  //state0->setFeedbackCoronalCOMDistance(mCoronalRightHip, -cd); // coronal right hip
  //state0->setFeedbackCoronalCOMVelocity(mCoronalRightHip, -cv); // coronal right hip
  state0->setFeedbackSagitalCOMDistance(mSagitalLeftHip,  -cd02); // sagital left hip
  state0->setFeedbackSagitalCOMVelocity(mSagitalLeftHip,  -cv02); // sagital left hip
  state0->setFeedbackSagitalCOMDistance(mSagitalRightHip, -cd02); // sagital right hip
  state0->setFeedbackSagitalCOMVelocity(mSagitalRightHip, -cv02); // sagital right hip

  //-- State 1
  //---- pelvis
  state1->setDesiredJointPosition("j_pelvis_rot", -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state1->setDesiredJointPosition("j_thigh_left", -swh13); // left hip
  state1->setDesiredJointPosition("j_shin_left", -swk13); // left knee
  //state1->setDesiredJointPosition("l_leg_aky", -swa13); // left ankle
  //---- stance leg
  state1->setDesiredJointPosition("j_shin_right", -stk13); // right knee
  //state1->setDesiredJointPosition("r_leg_aky", -sta13); // right ankle
  //---- arm
  //state1->setDesiredJointPosition("l_arm_shy", DART_RADIAN * +10.00); // left arm
  //state1->setDesiredJointPosition("r_arm_shy", DART_RADIAN * -20.00); // right arm
  //state1->setDesiredJointPosition("l_arm_shx", DART_RADIAN * -80.00); // left arm
  //state1->setDesiredJointPosition("r_arm_shx", DART_RADIAN * +80.00); // right arm
  //---- feedback gain for hip joints
  //state1->setFeedbackCoronalCOMDistance(mCoronalLeftHip,  -cd);  // coronal left hip
  //state1->setFeedbackCoronalCOMVelocity(mCoronalLeftHip,  -cv);  // coronal left hip
  //state1->setFeedbackCoronalCOMDistance(mCoronalRightHip, -cd);  // coronal right hip
  //state1->setFeedbackCoronalCOMVelocity(mCoronalRightHip, -cv);  // coronal right hip
  state1->setFeedbackSagitalCOMDistance(mSagitalLeftHip,  -cd13);  // sagital left hip
  state1->setFeedbackSagitalCOMVelocity(mSagitalLeftHip,  -cv13);  // sagital left hip
  state1->setFeedbackSagitalCOMDistance(mSagitalRightHip, -cd13);  // sagital right hip
  state1->setFeedbackSagitalCOMVelocity(mSagitalRightHip, -cv13);  // sagital right hip

  //-- State 2
  //---- pelvis
  state2->setDesiredJointPosition("j_pelvis_rot", -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state2->setDesiredJointPosition("j_thigh_left", -swh02); // left hip
  state2->setDesiredJointPosition("j_shin_left", -swk02); // left knee
  //state2->setDesiredJointPosition("l_leg_aky", -swa02); // left ankle
  //---- stance leg
  state2->setDesiredJointPosition("j_shin_right", -stk02); // right knee
  //state2->setDesiredJointPosition("r_leg_aky", -sta02); // right ankle
  //---- arm
  //state2->setDesiredJointPosition("l_arm_shy", DART_RADIAN * +10.00); // left arm
  //state2->setDesiredJointPosition("r_arm_shy", DART_RADIAN * -20.00); // right arm
  //state2->setDesiredJointPosition("l_arm_shx", DART_RADIAN * -80.00); // left arm
  //state2->setDesiredJointPosition("r_arm_shx", DART_RADIAN * +80.00); // right arm
  //---- feedback gain for hip joints
  //state2->setFeedbackCoronalCOMDistance(mCoronalLeftHip,  -cd);  // coronal left hip
  //state2->setFeedbackCoronalCOMVelocity(mCoronalLeftHip,  -cv);  // coronal left hip
  //state2->setFeedbackCoronalCOMDistance(mCoronalRightHip, -cd);  // coronal right hip
  //state2->setFeedbackCoronalCOMVelocity(mCoronalRightHip, -cv);  // coronal right hip
  state2->setFeedbackSagitalCOMDistance(mSagitalLeftHip,  -cd02);  // sagital left hip
  state2->setFeedbackSagitalCOMVelocity(mSagitalLeftHip,  -cv02);  // sagital left hip
  state2->setFeedbackSagitalCOMDistance(mSagitalRightHip, -cd02);  // sagital right hip
  state2->setFeedbackSagitalCOMVelocity(mSagitalRightHip, -cv02);  // sagital right hip

  //-- State 3
  //---- pelvis
  state3->setDesiredJointPosition("j_pelvis_rot", -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state3->setDesiredJointPosition("j_thigh_right", -swh13); // right hip
  state3->setDesiredJointPosition("j_shin_right", -swk13); // right knee
  //state3->setDesiredJointPosition("r_leg_aky", -swa13); // right ankle
  //---- stance leg
  state3->setDesiredJointPosition("j_shin_left", -stk13); // left knee
  //state3->setDesiredJointPosition("l_leg_aky", -sta13); // left ankle
  //---- arm
  //state3->setDesiredJointPosition("l_arm_shy", DART_RADIAN * -20.00); // left arm
  //state3->setDesiredJointPosition("r_arm_shy", DART_RADIAN * +10.00); // right arm
  //state3->setDesiredJointPosition("l_arm_shx", DART_RADIAN * -80.00); // left arm
  //state3->setDesiredJointPosition("r_arm_shx", DART_RADIAN * +80.00); // right arm
  //---- feedback gain for hip joints
  //state3->setFeedbackCoronalCOMDistance(mCoronalLeftHip,  -cd);  // coronal left hip
  //state3->setFeedbackCoronalCOMVelocity(mCoronalLeftHip,  -cv);  // coronal left hip
  //state3->setFeedbackCoronalCOMDistance(mCoronalRightHip, -cd);  // coronal right hip
  //state3->setFeedbackCoronalCOMVelocity(mCoronalRightHip, -cv);  // coronal right hip
  state3->setFeedbackSagitalCOMDistance(mSagitalLeftHip,  -cd13);  // sagital left hip
  state3->setFeedbackSagitalCOMVelocity(mSagitalLeftHip,  -cv13);  // sagital left hip
  state3->setFeedbackSagitalCOMDistance(mSagitalRightHip, -cd13);  // sagital right hip
  state3->setFeedbackSagitalCOMVelocity(mSagitalRightHip, -cv13);  // sagital right hip

  sm->addState(state0);
  sm->addState(state1);
  sm->addState(state2);
  sm->addState(state3);

  sm->setInitialState(state1);

  return sm;
}

/*
void Controller::setTargetPositions(const Eigen::VectorXd& pose)
{
	mTargetPositions = pose;
}
*/

} //namespace nonlinear

