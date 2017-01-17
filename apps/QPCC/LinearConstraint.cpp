#include "LinearConstraint.h"
#include "Var.h"

namespace qpcc {

LinearConstraint::LinearConstraint(std::vector<Var*>& _var,
                                   const Eigen::MatrixXd& A,
                                   const Eigen::VectorXd& b,
                                   const int _Equality)
    : Constraint(_var), mA(A), mB(b) {
  mNumRows = b.size();
  mConstTerm.resize(mNumRows);
  mConstTerm.setZero();
  mWeight.resize(mNumRows);
  mWeight.setOnes();
  mEquality = _Equality;
  // mIsNonlinear = false;
}

LinearConstraint::~LinearConstraint() {}

Eigen::VectorXd LinearConstraint::evalCon() {
  int dim_var = mA.cols();
  int dim_cnst = mA.rows();
  Eigen::VectorXd x(dim_var);
  for (size_t i = 0; i < dim_var; i++) x[i] = mVariables[i]->mVal;

  std::cout << "Linear Con " << mA * x-mB << std::endl;
  return mA * x - mB;
}

void LinearConstraint::fillJac(VVD _jacobian, VVB _map, int _index) {
  int dim_var = mA.cols();
  int dim_cnst = mA.rows();
  for (size_t i = 0; i < dim_cnst; i++) {
    for (size_t j = 0; j < dim_var; j++) {
      _jacobian->at(_index + i)->at(j) = mA(i, j);
      _map->at(_index + i)->at(j) = true;
    }
  }
}

void LinearConstraint::updateParams(int _index) {}

// void LinearConstraint::fillObjGrad(std::vector<double>& _dG)
//{
//  Eigen::VectorXd diff = evalCon();
//  int nBody = mSkeleton->getNumBodyNodes();
//  for (int i = 0; i < nBody; i++)
//    _dG[i] += diff[0];
//}
}
