#include "SnLCPWrapper.h"

SnLCP::SnLCP(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
    : mA(A), mb(b) {
  dim_var = mA.cols();
  dim_cnst = mA.rows();
  assert(dim_var==dim_cnst);

  // negate b
  for (size_t i = 0; i < dim_var; i++) {
    mb(i) = -mb(i);
  }
}

void SnLCP::solve(Eigen::VectorXd& z) {
  z.resize(dim_var);

  QPCCProblem problem(dim_var, dim_cnst, mA, mb);
  SnoptSolver solver(&problem);
  solver.solve();

  for (size_t i = 0; i < dim_var; i++) {
    z(i) = problem.vars()[i]->mVal;
  }
  std::cout << z.transpose() << std::endl;
}
