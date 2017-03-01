#ifndef LCPLPPROBLEM_H
#define LCPLPPROBLEM_H

#include "apps/QPCC/Constraint.h"
#include "apps/QPCC/ConstraintBox.h"
#include "apps/QPCC/L2NormConstraint.h"
#include "apps/QPCC/LCPConstraint.h"
#include "apps/QPCC/LinearConstraint.h"
#include "apps/QPCC/MyLCPConstraint.h"
#include "apps/QPCC/ObjectiveBox.h"
#include "apps/QPCC/Problem.h"
#include "apps/QPCC/QPCCProblem.h"
#include "apps/QPCC/QPObjective.h"
#include "apps/QPCC/SnoptSolver.h"
#include "apps/QPCC/Var.h"
#include "dart/dart.h"
#include "utils.h"

using namespace std;
using namespace qpcc;

class LCPLPproblem : public Problem {
  public:
  LCPLPproblem(size_t dim_var, size_t dim_cnst, const Eigen::MatrixXd& A,
               const Eigen::VectorXd& b);
  virtual ~LCPLPproblem();
  virtual void update(double* _coefs){};
};

/*
 * /// Here using LP completely to solve the problem
 * /// where lambda = 0, fn > 0, and fd >=0
 * bool LCPLP(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
 *            Eigen::VectorXd& z);
 */

#endif
