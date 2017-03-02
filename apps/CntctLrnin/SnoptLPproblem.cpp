#include "SnoptLPproblem.h"


SnoptLPproblem::SnoptLPproblem(size_t dim_var, size_t dim_cnst,
                         const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
  double initVal[dim_var];
  for (size_t i = 0; i < dim_var; i++) {
    initVal[i] = 0;
  }

  Eigen::MatrixXd AMat(dim_cnst, dim_var);
  Eigen::VectorXd bVec(dim_cnst);
  AMat = A;
  bVec = b;

  for (size_t i = 0; i < dim_var; i++) {
    // addVariable(initVal[i], -1e9, 1e9);
    addVariable(initVal[i], 0.0, 1e9);
  }

  createBoxes();

  qpcc::LinearConstraint* equlinear =
      new qpcc::LinearConstraint(this->vars(), AMat, bVec, 0);
  conBox()->add(equlinear);
}

SnoptLPproblem::~SnoptLPproblem() {
  for (int i = 0; i < vars().size(); i++) delete vars()[i];
  vars().clear();
  conBox()->clear();
  objBox()->clear();
}
