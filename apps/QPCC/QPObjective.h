#ifndef APPS_QPCC_QPOBJ_H
#define APPS_QPCC_QPOBJ_H

#include "Constraint.h"

namespace qpcc {

class QPObjective : public Constraint {
public:
  QPObjective(std::vector<Var*>& _var, const Eigen::MatrixXd& A,
              const Eigen::VectorXd& b);
  virtual ~QPObjective();
  virtual Eigen::VectorXd evalCon();
  virtual double evalObj();
  //    virtual void fillJac(VVD, int);
  virtual void fillJac(VVD, VVB, int);
  virtual void fillObjGrad(std::vector<double>&);
  virtual void updateParams(int _index);

  protected:
  Eigen::MatrixXd mA;
  Eigen::VectorXd mB;
};
}
#endif  // APPS_QPCC_L2NORMCONSTRAINT_H
