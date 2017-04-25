#include "SnoptQPproblem.h"

SnoptQPproblem::SnoptQPproblem(size_t dim_var, size_t dim_cnst,
                               const Eigen::MatrixXd& A,
                               const Eigen::VectorXd& b,
                               const Eigen::VectorXd& initGuess) {
  // double initVal[dim_var];
  // for (size_t i = 0; i < dim_var; i++) {
  //   initVal[i] = 0;
  // }

  Eigen::MatrixXd AMat(dim_cnst, dim_var);
  Eigen::VectorXd bVec(dim_cnst);

  AMat = A;
  bVec = b;

  for (size_t i = 0; i < dim_var; i++)
    // addVariable(initVal[i], -1e9, 1e9);
    addVariable(initGuess[i], 0.0, 1e9);

  // Create constraint and objective boxes
  createBoxes();

  //   Add constraints or objectives
  qpcc::LinearConstraint* linear =
      new qpcc::LinearConstraint(this->vars(), AMat, -bVec, 1);
  conBox()->add(linear);

  //  for (size_t i = 0; i < dim; i++)
  //  {
  //    LCPConstraint* lcp =
  //      new LCPConstraint(this->vars(), AMat.row(i), bVec[i], i);
  //    conBox()->add(lcp);
  //  }

  //   L2NormConstraint* l2 = new L2NormConstraint(this->vars());
  //   objBox()->add(l2);

  QPObjective* qp = new QPObjective(this->vars(), 2 * AMat, bVec);
  objBox()->add(qp);
}

SnoptQPproblem::~SnoptQPproblem() {
  for (int i = 0; i < vars().size(); i++) delete vars()[i];
  vars().clear();
  conBox()->clear();
  objBox()->clear();
}

void SnoptQPproblem::update(double* _coefs) {}
