#ifndef LCPLS_H
#define LCPLS_H

#include "SnoptLPproblem.h"
#include "SnoptWrapper.h"
#include "apps/lemkeFix/myLemke.h"
#include "dart/dart.h"
#include "parameter.h"

#define LCPLS_ZERO 1e-6

class LCPLS {
  public:
  LCPLS(const Eigen::MatrixXd& _A, const Eigen::VectorXd& _b,
        const std::vector<int> _value);
  virtual ~LCPLS(){};
  void solve();
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

Eigen::VectorXd value2ub_index(const std::vector<int> value);

#endif
