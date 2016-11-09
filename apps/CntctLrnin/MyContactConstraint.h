#ifndef MYCONTACTCONSTRAINT 
#define MYCONTACTCONSTRAINT 

#include "dart/constraint/ContactConstraint.h"

namespace dart{
namespace constraint{

class MyContactConstraint : public ContactConstraint
{
public:
    MyContactConstraint(collision::Contact& _contact, double _timeStep);
    virtual ~MyContactConstraint();

    // Here applyImpulse is not a virtual function but a function overload
    void applyImpulse(Eigen::VectorXd _lambda);

};

}
}

#endif  // MYCONTACTCONSTRAINT 
