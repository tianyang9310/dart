#include "LCPLPproblem.h"

/*
 * bool LCPLP(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
 *            Eigen::VectorXd& z) {
 *   int dim_var = mA.cols();
 *   int dim_cnst = mA.rows();
 *   assert(dim_var==dim_cnst);
 * }
 */

LCPLPproblem::LCPLPproblem(size_t dim_var, size_t dim_cnst,
                         const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
  double initVal[dim_var];
  for (size_t i = 0; i < dim_var; i++) {
    initVal[i] = 0;
  }
  Eigen::MatrixXd AMat(dim_cnst, dim_var);
  Eigen::VectorXd bVec(dim_cnst);
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

  qpcc::LinearConstraint* equlinear =
      new qpcc::LinearConstraint(this->vars(), AMat, bVec, 0);
  conBox()->add(equlinear);

  // qpcc::LinearConstraint* inequlinear =
  //     new qpcc::LinearConstraint(this->vars(), AMat.bottomLeftCorner(4,36), Eigen::VectorXd::Zero(4), 1);
  // conBox()->add(inequlinear);
  
  // MyLCPConstraint* lcp = new MyLCPConstraint(this->vars(), AMat, bVec);
  // conBox()->add(lcp);

  //   L2NormConstraint* l2 = new L2NormConstraint(this->vars());
  //   objBox()->add(l2);

  // Eigen::MatrixXd H(dim_var,dim_var);
  // H << 1,0,0,0;
  // Eigen::VectorXd f(dim_var);
  // f << 3,4;
  // QPObjective* qp = new QPObjective(this->vars(), H, f);
  // objBox()->add(qp);
}

LCPLPproblem::~LCPLPproblem() {
  for (int i = 0; i < vars().size(); i++) delete vars()[i];
  vars().clear();
  conBox()->clear();
  objBox()->clear();
}
