#ifndef APPS_QPCC_L2NORMCONSTRAINT_H
#define APPS_QPCC_L2NORMCONSTRAINT_H

#include "Constraint.h"

namespace qpcc {

class L2NormConstraint : public Constraint
{
public:
  L2NormConstraint(std::vector<Var *>& _var);
  virtual ~L2NormConstraint();
  virtual Eigen::VectorXd evalCon();
  virtual double evalObj();
  //    virtual void fillJac(VVD, int);
  virtual void fillJac(VVD, VVB, int);
  virtual void fillObjGrad(std::vector<double>&);
  virtual void updateParams(int _index);
  
protected:
};
}
#endif // APPS_QPCC_L2NORMCONSTRAINT_H
