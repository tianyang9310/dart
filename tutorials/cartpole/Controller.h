#ifndef CONTROLLER_H
#define CONTROLLER_H 

#include "dart/dart.h"
#include "DDP.h"

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace dart::common;
using namespace dart::math;
using namespace dart::gui;


class Controller
{
public:
	Controller(SkeletonPtr mCartPole, double delta_t, WorldPtr mDDPWorld);
//------------------------------------------------------------------------------------------------
	SkeletonPtr mCartPole;
	std::unique_ptr<DDP> mDDP;
};

#endif
