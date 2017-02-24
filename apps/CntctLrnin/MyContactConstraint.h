#ifndef MYCONTACTCONSTRAINT
#define MYCONTACTCONSTRAINT

#include "dart/constraint/ContactConstraint.h"
#include "utils.h"

namespace dart {

namespace dynamics {
class BodyNode;
class Skeleton;
}  // namespace dynamics
}

namespace CntctLrnin {

using namespace dart;
using namespace dart::constraint;

class MyContactConstraint : public dart::constraint::ContactConstraint {
  public:
  MyContactConstraint(collision::Contact& _contact, double _timeStep);
  virtual ~MyContactConstraint();

  // Here applyImpulse is not a virtual function but a function overload
  void MyapplyImpulse(double fn, const Eigen::VectorXd& fd,
                      const Eigen::VectorXd& N, const Eigen::MatrixXd& B,
                      int BodyNode1_dim, int BodyNode2_dim, bool impulse_flag);
  void My2LemkeapplyImpulse(double fn, const Eigen::VectorXd& fd,
                            const Eigen::VectorXd& N, const Eigen::MatrixXd& B,
                            int BodyNode1_dim, int BodyNode2_dim,
                            bool impulse_flag, Eigen::Vector3d& allForce,
                            Eigen::Vector3d& allTorque,Eigen::VectorXd& GeneralizedForces, std::shared_ptr<std::fstream> Lemke_FILE);
  void My2ODEapplyImpulse(double* _lambda, Eigen::Vector3d& allForce,
                          Eigen::Vector3d& allTorque,Eigen::VectorXd& GeneralizedForces, std::shared_ptr<std::fstream> ODE_FILE);
  void getInformation(ConstraintInfo* _info);
  int numBasis;
};

}

#endif  // MYCONTACTCONSTRAINT
