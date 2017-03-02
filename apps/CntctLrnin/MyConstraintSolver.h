#ifndef MYCONSTRAINTSOLVER_H
#define MYCONSTRAINTSOLVER_H

#include "MyContactConstraint.h"
#include "dart/constraint/ConstraintSolver.h"
// #include "dart/constraint/SoftContactConstraint.h"

namespace dart {

namespace dynamics {
class Skeleton;
}

namespace constraint {

class ConstrainedGroup;
class ConstraintBase;
class ClosedLoopConstraint;
class ContactConstraint;
class SoftContactConstraint;
class JointLimitConstraint;
class JointCoulombFrictionConstraint;
class JointConstraint;
class LCPSolver;
}
}

namespace CntctLrnin {

using namespace dart;
using namespace dart::constraint;

class MyConstraintSolver : public ConstraintSolver {
  public:
  MyConstraintSolver(double _timeStep);
  virtual ~MyConstraintSolver();

  void updateConstraints();
};

}

#endif  // MYCONSTRAINTSOLVER
