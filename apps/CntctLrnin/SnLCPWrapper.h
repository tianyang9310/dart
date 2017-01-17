#ifndef SNLCPWRAPPER_H
#define SNLCPWRAPPER_H

#include <iostream>
#include "apps/QPCC/QPCCProblem.h"
#include "apps/QPCC/SnoptSolver.h"
#include "apps/QPCC/Var.h"
#include "dart/dart.h"

using namespace qpcc;

class SnLCP {
  public:
  SnLCP(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);
  virtual ~SnLCP(){};
  void solve(Eigen::VectorXd& z);

  protected:
  Eigen::MatrixXd mA;
  Eigen::VectorXd mb;
  int dim_cnst;
  int dim_var;
};

#endif
