#include "SnoptWrapper.h"

SnoptWrapper::SnoptWrapper(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
    : mA(A), mb(b) {
  dim_var = mA.cols();
  dim_cnst = mA.rows();
  assert(dim_var == dim_cnst);
}

void SnoptWrapper::solveLCP(Eigen::VectorXd& z) {
  // negate b because ```ret = ((mA * x - mB).array() * x.array()).matrix();```
  for (size_t i = 0; i < dim_var; i++) {
    mb(i) = -mb(i);
  }
  z.resize(dim_var);
  z.setZero();

  // Use SnoptLP to solve a LP problem, where A*z+b=w, z>=0, w>=0, z^T*w=0
  QPCCProblem problem(dim_var, dim_cnst, mA, mb);
  SnoptSolver solver(&problem);
  solver.solve();

  for (size_t i = 0; i < dim_var; i++) {
    z(i) = problem.vars()[i]->mVal;
  }
  // std::cout << z.transpose() << std::endl;
}

void SnoptWrapper::solveLP(Eigen::VectorXd& z) {
  // No need to negate because ```return mA * x - mB;```
  z.resize(dim_var);
  z.setZero();

  // Use SnoptLP to solve a LP problem, where A*x = b, x>=0
  SnoptLPproblem problem(dim_var, dim_cnst, mA, mb);
  SnoptSolver solver(&problem);

  solver.solve();

  for (size_t i = 0; i < dim_var; i++) {
    z(i) = problem.vars()[i]->mVal;
  }
  // std::cout << z.transpose() << std::endl;
}
