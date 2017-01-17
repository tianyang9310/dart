#include "QPCCProblem.h"
#include "Constraint.h"
#include "ConstraintBox.h"
#include "L2NormConstraint.h"
#include "LCPConstraint.h"
#include "LinearConstraint.h"
#include "MyLCPConstraint.h"
#include "ObjectiveBox.h"
#include "QPObjective.h"
#include "Var.h"

namespace qpcc {
QPCCProblem::QPCCProblem(size_t dim_var, size_t dim_cnst, double* A,
                         double* b) {
  double initVal[dim_var];
  for (size_t i = 0; i < dim_var; i++) {
    initVal[i] = 0;
  }
  Eigen::MatrixXd AMat(dim_cnst, dim_var);
  Eigen::VectorXd bVec(dim_cnst);
  for (size_t i = 0; i < dim_cnst; i++) {
    bVec[i] = b[i];
    for (size_t j = 0; j < dim_var; j++) AMat(i, j) = A[i * dim_var + j];
  }

  for (size_t i = 0; i < dim_var; i++)
    // addVariable(initVal[i], -1e9, 1e9);
    addVariable(initVal[i], 0.0, 1e9);

  // Create constraint and objective boxes
  createBoxes();

  //   Add constraints or objectives
  // qpcc::LinearConstraint* linear = new qpcc::LinearConstraint(this->vars(),
  // AMat, bVec);
  // conBox()->add(linear);

  // for (size_t i = 0; i < dim_cnst; i++)
  // {
  //   LCPConstraint* lcp =
  //     new LCPConstraint(this->vars(), AMat.row(i), bVec[i], i);
  //   conBox()->add(lcp);
  // }

  qpcc::LinearConstraint* linear =
      new qpcc::LinearConstraint(this->vars(), AMat, bVec, 1);
  conBox()->add(linear);
  MyLCPConstraint* lcp = new MyLCPConstraint(this->vars(), AMat, bVec);
  conBox()->add(lcp);

  //   L2NormConstraint* l2 = new L2NormConstraint(this->vars());
  //   objBox()->add(l2);

  // Eigen::MatrixXd H(dim_var,dim_var);
  // H << 1,0,0,0;
  // Eigen::VectorXd f(dim_var);
  // f << 3,4;
  // QPObjective* qp = new QPObjective(this->vars(), H, f);
  // objBox()->add(qp);
}

QPCCProblem::QPCCProblem(size_t dim_var, size_t dim_cnst,
                         const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
  double initVal[dim_var];
  for (size_t i = 0; i < dim_var; i++) {
    initVal[i] = 0;
  }
  Eigen::MatrixXd AMat(dim_cnst, dim_var);
  Eigen::VectorXd bVec(dim_cnst);
  /*
   * for (size_t i = 0; i < dim_cnst; i++) {
   *   bVec[i] = b[i];
   *   for (size_t j = 0; j < dim_var; j++) AMat(i, j) = A[i * dim_var + j];
   * }
   */
  AMat = A;
  bVec = b;

  for (size_t i = 0; i < dim_var; i++)
    // addVariable(initVal[i], -1e9, 1e9);
    addVariable(initVal[i], 0.0, 1e9);

  // Create constraint and objective boxes
  createBoxes();

  //   Add constraints or objectives
  // qpcc::LinearConstraint* linear = new qpcc::LinearConstraint(this->vars(),
  // AMat, bVec);
  // conBox()->add(linear);

  // for (size_t i = 0; i < dim_cnst; i++)
  // {
  //   LCPConstraint* lcp =
  //     new LCPConstraint(this->vars(), AMat.row(i), bVec[i], i);
  //   conBox()->add(lcp);
  // }

  qpcc::LinearConstraint* linear =
      new qpcc::LinearConstraint(this->vars(), AMat, bVec, 1);
  conBox()->add(linear);
  MyLCPConstraint* lcp = new MyLCPConstraint(this->vars(), AMat, bVec);
  conBox()->add(lcp);

  //   L2NormConstraint* l2 = new L2NormConstraint(this->vars());
  //   objBox()->add(l2);

  // Eigen::MatrixXd H(dim_var,dim_var);
  // H << 1,0,0,0;
  // Eigen::VectorXd f(dim_var);
  // f << 3,4;
  // QPObjective* qp = new QPObjective(this->vars(), H, f);
  // objBox()->add(qp);
}

QPCCProblem::~QPCCProblem() {
  for (int i = 0; i < vars().size(); i++) delete vars()[i];
  vars().clear();
  conBox()->clear();
  objBox()->clear();
}

void QPCCProblem::update(double* _coefs) {}
}
