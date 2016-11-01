#ifndef CNTCTLRNIN_ADDSKEL_H
#define CNTCTLRNIN_ADDSKEL_H

#include "dart/dart.h"

using namespace dart::simulation;
using namespace dart::dynamics;

void AddSkel(dart::simulation::WorldPtr world);
SkeletonPtr AddBox();
SkeletonPtr AddGround();

#endif // CNTCTLRNIN_ADDSKEL_H
