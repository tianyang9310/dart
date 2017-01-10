#ifndef CNTCTLRNIN_ADDSKEL_H
#define CNTCTLRNIN_ADDSKEL_H

#include "dart/dart.h"
#include "MyWindow.h"

using namespace dart::simulation;
using namespace dart::dynamics;

void AddSkel(dart::simulation::WorldPtr world);
SkeletonPtr AddBox();
SkeletonPtr AddGround();
SkeletonPtr AddPlatform();

#endif  // CNTCTLRNIN_ADDSKEL_H
