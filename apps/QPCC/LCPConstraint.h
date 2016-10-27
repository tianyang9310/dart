#ifndef APPS_QPCC_LCPCONSTRAINT_H
#define APPS_QPCC_LCPCONSTRAINT_H

#include <iostream>
#include "Constraint.h"
#include "dart/dynamics/Skeleton.h"

namespace qpcc {
class LCPConstraint : public Constraint {
public:
  LCPConstraint(std::vector<Var *>& _var,
                const Eigen::VectorXd& a,
                double b,
                int rowIndex);
  virtual ~LCPConstraint();
  virtual Eigen::VectorXd evalCon();
  //    virtual void fillJac(VVD, int);
  virtual void fillJac(VVD, VVB, int);
  //virtual void fillObjGrad(std::vector<double>&);
  virtual void updateParams(int _index);
  
protected:
  Eigen::VectorXd mA;
  double mB;
  int mRowIndex;
};
}
#endif // APPS_QPCC_LCPCONSTRAINT_H
