/*************************************************************************
    > File Name: Controller.h
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Fri 20 May 2016 12:42:04 PM EDT
 ************************************************************************/

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
	Controller(SkeletonPtr mCartPole, double delta_t);
//------------------------------------------------------------------------------------------------
	SkeletonPtr mCartPole;
	std::unique_ptr<DDP> mDDP;
};

#endif
