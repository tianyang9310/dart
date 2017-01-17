#include "QPCCProblem.h"
#include "ConstraintBox.h"
#include "ObjectiveBox.h"
#include "Var.h"
#include "Constraint.h"
#include "LinearConstraint.h"
#include "LCPConstraint.h"
#include "L2NormConstraint.h"
#include "QPObjective.h"

namespace qpcc {
QPCCProblem::QPCCProblem(size_t dim_var, size_t dim_cnst, double* A, double* b)
{
//  double initVal[] = {4.35613, -0.575495, -0.125088, 4.32588, -0.760761, -0.0918233, 1.39549, -0.629149, -0.18671, 1.3659, -0.70944, -0.138637, 4.3259, -0.575683, -0.12527, 4.3561, -0.760558, -0.0920227, 1.36589, -0.62892, -0.186495, 1.3955, -0.709698, -0.1384};
  double initVal[] = {0, 0};
  Eigen::MatrixXd AMat(dim_cnst, dim_var);
  Eigen::VectorXd bVec(dim_cnst);
  for (size_t i = 0; i < dim_cnst; i++)
  {
    bVec[i] = b[i];
    for (size_t j = 0; j < dim_var; j++)
      AMat(i, j) = A[i * dim_var + j];
  }
  
 for (size_t i = 0; i < dim_var; i++)
   addVariable(initVal[i], 0, 1e9);
   // addVariable(1e3, 0.0, 1e9);
  
  // Create constraint and objective boxes
  createBoxes();

//   Add constraints or objectives
   qpcc::LinearConstraint* linear = new qpcc::LinearConstraint(this->vars(), AMat, bVec);
   conBox()->add(linear);

//  for (size_t i = 0; i < dim; i++)
//  {
//    LCPConstraint* lcp =
//      new LCPConstraint(this->vars(), AMat.row(i), bVec[i], i);
//    conBox()->add(lcp);
//  }
  
//   L2NormConstraint* l2 = new L2NormConstraint(this->vars());
//   objBox()->add(l2);
  Eigen::MatrixXd H(2,2);
  H << 1, -1, -1, 2;
  Eigen::VectorXd f(2);
  f << -2, -6;
  QPObjective* qp = new QPObjective(this->vars(), H, f);
  objBox()->add(qp);
}

QPCCProblem::~QPCCProblem() {
  for (int i = 0; i < vars().size(); i++)
    delete vars()[i];
  vars().clear();
  conBox()->clear();
  objBox()->clear();  
}

void QPCCProblem::update(double* _coefs) {
}
  
}

