/*************************************************************************
    > File Name: Controller.h
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Wed 16 Mar 2016 01:08:59 PM EDT
 ************************************************************************/

#ifndef CONTROLLER_H
#define CONTROLLER_H 

#include "dart/dart.h"
#include "../ControlPBP/ControlPBP.h"


namespace toyexample{

using namespace dart::common;
using namespace dart::utils;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;
using namespace dart::gui;


class Controller
{
public:
	Controller(SkeletonPtr cube, SkeletonPtr world_setup, dart::collision::CollisionDetector* detector);
	void setCubeVelocity();
	bool collisionEvent();
	bool collisionEvent_detector();
protected:
	SkeletonPtr mCube;
	SkeletonPtr mWorld_Setup;	
	dart::collision::CollisionDetector* mDetector;
	double mSpeed;
	double mCollisionThre;
};




} // namespace toyexample


#endif
