#ifndef LCPQP_H
#define LCPQP_H

#include "SnoptLPproblem.h"
#include "SnoptWrapper.h"
#include "dart/dart.h"
#include "parameter.h"
#include "LCPLS.h"
#include "apps/lemkeFix/myLemke.h"

#define LCPQP_ZERO 1e-6

class LCPQP {
  public:
  LCPQP(const Eigen::MatrixXd& _A, const Eigen::VectorXd& _b,
        const std::vector<int> _value);
  virtual ~LCPQP(){};
  void solve(bool useInit = true);
  Eigen::VectorXd getSolution();

  protected:
  int dim_cnst;
  int dim_var;
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  Eigen::VectorXd z;
  std::vector<int> value;
  std::shared_ptr<SnoptSolver> solver;
};

#endif
