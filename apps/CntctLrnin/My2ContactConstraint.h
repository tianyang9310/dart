#ifndef MY2CONTACTCONSTRAINT
#define MY2CONTACTCONSTRAINT

#include "dart/constraint/ContactConstraint.h"
#include "utils.h"

namespace dart {

namespace dynamics {
class BodyNode;
class Skeleton;
}  // namespace dynamics

namespace constraint {

class My2ContactConstraint : public ContactConstraint {
  public:
  /// Constructor
  My2ContactConstraint(collision::Contact& _contact, double _timeStep);

  /// Destructor
  virtual ~My2ContactConstraint();

  //----------------------------------------------------------------------------
  // Member Function
  //----------------------------------------------------------------------------
  void getInformation(ConstraintInfo* _info);
  void getRelVelocity(double* _relVel) override;
  void applyUnitImpulse(size_t _idx);
  void getVelocityChange(double* _vel, bool _withCfm);

  //----------------------------------------------------------------------------
  // Member Function
  //----------------------------------------------------------------------------
  int numBasis;
};

} // namespace constraint
} // namespace dart

#endif  // MYCONTACTCONSTRAINT
