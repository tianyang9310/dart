#ifndef SnoptQPproblem_H
#define SnoptQPproblem_H

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

class SnoptQPproblem : public Problem {
  public:
  SnoptQPproblem(size_t dim_var, size_t dim_cnst, const Eigen::MatrixXd& A,
                 const Eigen::VectorXd& b, const Eigen::VectorXd& initGuess);
  virtual ~SnoptQPproblem();

  virtual void update(double* _coefs);
};

#endif
