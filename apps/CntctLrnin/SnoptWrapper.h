#ifndef SNOPTWRAPPER_H
#define SNOPTWRAPPER_H

#include <iostream>
#include "SnoptLPproblem.h"
#include "SnoptQPproblem.h"
#include "apps/QPCC/QPCCProblem.h"
#include "apps/QPCC/SnoptSolver.h"
#include "apps/QPCC/Var.h"
#include "dart/dart.h"

using namespace qpcc;

class SnoptWrapper {
  public:
  SnoptWrapper(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);
  virtual ~SnoptWrapper(){};
  void solveLCP(Eigen::VectorXd& z);
  void solveLP(Eigen::VectorXd& z);
  void solveQP(const Eigen::VectorXd& z0, Eigen::VectorXd& z);

  private:
  Eigen::MatrixXd mA;
  Eigen::VectorXd mb;
  int dim_cnst;
  int dim_var;
};

#endif
