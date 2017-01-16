#include "QPObjective.h"
#include "Var.h"

namespace qpcc {

QPObjective::QPObjective(std::vector<Var*>& _var, const Eigen::MatrixXd& A,
            const Eigen::VectorXd& b)
    : Constraint(_var), mA(A), mB(b) {
  mNumRows = 1;
  mConstTerm.resize(mNumRows);
  mConstTerm.setZero();
  mWeight.resize(mNumRows);
  mWeight.setOnes();
}

QPObjective::~QPObjective() {}

Eigen::VectorXd QPObjective::evalCon() {
  size_t dim = mVariables.size();
  Eigen::VectorXd x(dim);
  for (size_t i = 0; i < dim; i++) x[i] = mVariables[i]->mVal;

  Eigen::VectorXd ret(1);
  ret = 0.5 * x.transpose() * mA * x + mB.transpose() * x;
  return ret;
}

double QPObjective::evalObj() {
  size_t dim = mVariables.size();
  Eigen::VectorXd x(dim);
  for (size_t i = 0; i < dim; i++) x[i] = mVariables[i]->mVal;

  Eigen::VectorXd ret(1);
  ret = (0.5 * x.transpose() * mA * x + mB.transpose() * x).eval();
  return ret[0] * mWeight[0];
}

void QPObjective::fillJac(VVD _jacobian, VVB _map, int _index) {
  size_t dim = mVariables.size();
  Eigen::VectorXd x(dim);
  for (size_t i = 0; i < dim; i++) x[i] = mVariables[i]->mVal;

  Eigen::VectorXd _tmpJacobian(dim);
  _tmpJacobian = 0.5 * (mA + mA.transpose()) * x + mB;

  for (size_t i = 0; i < dim; i++) {
    _jacobian->at(_index)->at(i) = _tmpJacobian(i);
    _map->at(_index)->at(i) = true;
  }
}

void QPObjective::updateParams(int _index) {}

void QPObjective::fillObjGrad(std::vector<double>& _dG) {
  size_t dim = mVariables.size();
  Eigen::VectorXd x(dim);
  for (size_t i = 0; i < dim; i++) x[i] = mVariables[i]->mVal;

  Eigen::VectorXd _tmpJacobian(dim);
  _tmpJacobian = 0.5 * (mA + mA.transpose()) * x + mB;


  for (size_t i = 0; i < dim; i++)
    _dG[i] += mWeight[0] * _tmpJacobian(i); 
}
}
