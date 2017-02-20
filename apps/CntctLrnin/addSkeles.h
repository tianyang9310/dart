#ifndef CNTCTLRNIN_ADDSKEL_H
#define CNTCTLRNIN_ADDSKEL_H

#include <string>
#include "MyWindow.h"
#include "dart/dart.h"

#define rsttn_cff 0.0
#define frcton_cff 0.0
#define jnt_dmpin 0.0

using namespace dart::simulation;
using namespace dart::dynamics;

void AddSkel(dart::simulation::WorldPtr world);
SkeletonPtr AddBox(int numCubes, const Eigen::Vector3d& init_pos_offset);
SkeletonPtr AddGround();
SkeletonPtr AddPlatform();

#endif  // CNTCTLRNIN_ADDSKEL_H
