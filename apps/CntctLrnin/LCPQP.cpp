#include "LCPQP.h"

LCPQP::LCPQP(const Eigen::MatrixXd& _A, const Eigen::VectorXd& _b,
             const std::vector<int> _value)
    : A(_A), b(_b), value(_value) {
  dim_cnst = A.rows();
  dim_var = A.cols();
  assert(dim_var == dim_cnst);
  z.resize(dim_var);
  z.setZero();
}

void LCPQP::solve(bool useInit) {
  if (b.minCoeff() >= -LCPQP_ZERO) {
    z.setZero();
    return;
  }

  std::vector<int> bas;
  std::vector<int> nonbas;
  bas.clear();
  nonbas.clear();

  Eigen::VectorXd z0 = value2ub_index(value);

  for (int idx = 0; idx < dim_var; ++idx) {
    if (z0(idx) > 0) {
      bas.push_back(idx);
    } else {
      nonbas.push_back(idx);
    }
  }

  Eigen::MatrixXd B = -Eigen::MatrixXd::Identity(dim_var, dim_var);

  Eigen::MatrixXd B_copy = B;
  for (size_t i = 0; i < bas.size(); ++i) {
    B.col(i) = A.col(bas[i]);
  }
  for (size_t i = 0; i < nonbas.size(); ++i) {
    B.col(bas.size() + i) = B_copy.col(nonbas[i]);
  }

  assert(dim_cnst == (bas.size() + nonbas.size()));

  SnoptWrapper mSnoptLPSolver(B, -b);
  Eigen::VectorXd __z;
  mSnoptLPSolver.solveLP(__z);

  if (__z.minCoeff() > -LCPQP_ZERO) {
    Eigen::VectorXd _z = Eigen::VectorXd::Zero(2 * dim_var);
    for (size_t i = 0; i < bas.size(); i++) {
      _z(bas[i]) = __z(i);
    }
    z = _z.head(dim_var);
  } else {
    // z.setZero();
  }

  if (!dart::lcpsolver::YT::validate(A, b, z)) {
    // First check: if fail, try to use LP solution as initial guess
    // for snopt's QP
    // std::cout << "Using Linear Programming fails, instead using qudratic
    // Programming..." << std::endl;
    Eigen::VectorXd z0 = z;
    z.setZero();
    SnoptWrapper mSnoptLPSolver4QP(A, b);
    if (useInit) {
      mSnoptLPSolver4QP.solveQP(z0, z);
    } else {
      mSnoptLPSolver4QP.solveQP(z, z);
    }

    if (!dart::lcpsolver::YT::validate(A, b, z)) {
      z.setZero();
    }
  }
}

Eigen::VectorXd LCPQP::getSolution() { return z; }
