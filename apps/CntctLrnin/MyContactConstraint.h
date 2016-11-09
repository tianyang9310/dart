#ifndef MYCONTACTCONSTRAINT 
#define MYCONTACTCONSTRAINT 

#include "dart/constraint/ContactConstraint.h"
#include "utils.h"

namespace dart{

namespace dynamics {
class BodyNode;
class Skeleton;
}  // namespace dynamics

namespace constraint{

class MyContactConstraint : public ContactConstraint
{
public:
    MyContactConstraint(collision::Contact& _contact, double _timeStep);
    virtual ~MyContactConstraint();

    // Here applyImpulse is not a virtual function but a function overload
    void MyapplyImpulse(double fn, const Eigen::VectorXd & fd, const Eigen::VectorXd &  N, const Eigen::MatrixXd & B, int BodyNode1_dim, int BodyNode2_dim);
    virtual void applyImpulse(double* _lambda);
    int numBasis;

};

}
}

#endif  // MYCONTACTCONSTRAINT 
