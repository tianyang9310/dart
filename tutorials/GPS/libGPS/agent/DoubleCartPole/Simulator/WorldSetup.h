#ifndef worldsetup_h
#define worldsetup_h


#include "dart/dart.h"


using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace dart::common;
using namespace dart::math;
using namespace dart::gui;

SkeletonPtr addDoubleCartPole();
void WorldSetup(WorldPtr mWorld);
#endif
