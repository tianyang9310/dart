#ifndef CNTCTLRNIN_ADDSKEL_H
#define CNTCTLRNIN_ADDSKEL_H

#include <string>
#include "parameter.h"
#include "dart/dart.h"

#define rsttn_cff 1.0
#define jnt_dmpin 0.0

namespace CntctLrnin {

using namespace dart::simulation;
using namespace dart::dynamics;

/// Body node shape
enum class mShapeType {cube, ball};

/// 
void addSkel(WorldPtr world);

/// 
SkeletonPtr addBox(int numCubes, const Eigen::Vector3d& init_pos_offset);

/// 
SkeletonPtr addPlatform();
}

#endif  // CNTCTLRNIN_ADDSKEL_H
