#ifndef MYCONTACTCONSTRAINT_H
#define MYCONTACTCONSTRAINT_H

#include "dart/constraint/ContactConstraint.h"
#include "parameter.h"
#include "utils.h"

namespace dart {

namespace dynamics {
class BodyNode;
class Skeleton;
}  // namespace dynamics
}  // namespace dart

namespace CntctLrnin {

using namespace dart;
using namespace dart::constraint;

class MyContactConstraint : public ContactConstraint {
  public:
  /// Constructor
  MyContactConstraint(collision::Contact& _contact, double _timeStep);

  /// Destructor
  virtual ~MyContactConstraint();

  //----------------------------------------------------------------------------
  // Member Function
  //----------------------------------------------------------------------------
  void getInformation(ConstraintInfo* _info) override;
  void getRelVelocity(double* _relVel) override;
  void applyUnitImpulse(size_t _idx) override;
  void MyapplyImpulse(double fn, const Eigen::VectorXd& fd, bool impulse_flag);
  void applyImpulse(double* _lambda) override;
  void getVelocityChange(double* _vel, bool _withCfm) override;

  //----------------------------------------------------------------------------
  // Member Function
  //----------------------------------------------------------------------------
  int numBasis;
  int mPrecision;
};

}  // namespace CntctLrnin

#endif  // MyContactConstraint
