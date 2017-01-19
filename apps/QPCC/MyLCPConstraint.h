#ifndef APPS_QPCC_MYLCPCONSTRAINT_H
#define APPS_QPCC_MYLCPCONSTRAINT_H

#include <iostream>
#include "Constraint.h"
#include "dart/dynamics/Skeleton.h"

namespace qpcc {
class MyLCPConstraint : public Constraint {
public:
  MyLCPConstraint(std::vector<Var *>& _var,
                const Eigen::MatrixXd& a,
                const Eigen::VectorXd& b);
  virtual ~MyLCPConstraint();
  virtual Eigen::VectorXd evalCon();
  //    virtual void fillJac(VVD, int);
  virtual void fillJac(VVD, VVB, int);
  //virtual void fillObjGrad(std::vector<double>&);
  virtual void updateParams(int _index);
  
protected:
  Eigen::MatrixXd mA;
  Eigen::VectorXd mB;
};
}
#endif // APPS_QPCC_LCPCONSTRAINT_H
