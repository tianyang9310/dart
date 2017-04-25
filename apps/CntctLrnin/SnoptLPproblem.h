#ifndef SnoptLPproblem_H
#define SnoptLPproblem_H

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

/// Use Snopt to solve a LP problem, where A*x = b, x>=0
class SnoptLPproblem : public Problem {
  public:
  SnoptLPproblem(size_t dim_var, size_t dim_cnst, const Eigen::MatrixXd& A,
                 const Eigen::VectorXd& b, const Eigen::VectorXd* z0 = nullptr);
  virtual ~SnoptLPproblem();

  virtual void update(double* _coefs){};
};

#endif
