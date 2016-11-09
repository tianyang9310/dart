#include "MyContactConstraint.h"

namespace dart{
namespace constraint{

MyContactConstraint::MyContactConstraint(collision::Contact& _contact,  double _timestep):ContactConstraint(_contact, _timestep)
{

}

MyContactConstraint::~MyContactConstraint()
{

}

void MyContactConstraint::applyImpulse(Eigen::VectorXd _lambda)
{

}

}
}
