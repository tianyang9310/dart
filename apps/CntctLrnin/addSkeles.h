#ifndef CNTCTLRNIN_ADDSKEL_H
#define CNTCTLRNIN_ADDSKEL_H

#include "dart/dart.h"
#include "MyWindow.h"

#define rsttn_cff 0.0
#define frcton_cff  0.0
#define jnt_dmpin  0.0

using namespace dart::simulation;
using namespace dart::dynamics;

void AddSkel(dart::simulation::WorldPtr world);
SkeletonPtr AddBox();
SkeletonPtr AddGround();
SkeletonPtr AddPlatform();

#endif  // CNTCTLRNIN_ADDSKEL_H
