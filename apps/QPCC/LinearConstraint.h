#ifndef APPS_QPCC_LINEARCONSTRAINT_H
#define APPS_QPCC_LINEARCONSTRAINT_H

#include "Constraint.h"

namespace qpcc {
class LinearConstraint : public Constraint {
  public:
  LinearConstraint(std::vector<Var*>& _var, const Eigen::MatrixXd& A,
                   const Eigen::VectorXd& b, const int _Equality = -1);
  virtual ~LinearConstraint();
  virtual Eigen::VectorXd evalCon();
  //    virtual void fillJac(VVD, int);
  virtual void fillJac(VVD, VVB, int);
  //  virtual void fillObjGrad(std::vector<double>&);
  virtual void updateParams(int _index);

  protected:
  Eigen::MatrixXd mA;
  Eigen::VectorXd mB;
};
}

#endif  // APPS_QPCC_LINEARCONSTRAINT_H
