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
  My2ContactConstraint(collision::Contact& _contact, double _timeStep);
  virtual ~My2ContactConstraint();

  int numBasis;
};
}
}

#endif  // MYCONTACTCONSTRAINT
