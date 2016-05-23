/*************************************************************************
    > File Name: Controller.cpp
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Fri 20 May 2016 12:44:11 PM EDT
 ************************************************************************/

#include "Controller.h"
#include "DDP.h"
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace dart::common;
using namespace dart::math;
using namespace dart::gui;

Controller::Controller(SkeletonPtr mCartPole):mCartPole(mCartPole)
{
	mDDP = std::unique_ptr<DDP>(new DDP(5000));
}
