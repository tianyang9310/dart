/*************************************************************************
    > File Name: WorldSetup.h
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Tue Apr  5 10:10:57 2016
 ************************************************************************/

#ifndef WORLD_SETUP_H
#define WORLD_SETUP_H 

#include "dart/dart.h"
    
namespace nonlinear{

using namespace dart::common;
using namespace dart::utils;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;
using namespace dart::gui;

SkeletonPtr createFloor();

SkeletonPtr loadBiped();

} // namespace nonlinear
#endif
