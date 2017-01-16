#include "L2NormConstraint.h"
#include "Var.h"

namespace qpcc {

L2NormConstraint::L2NormConstraint(std::vector<Var *>& _var) : Constraint(_var)
{
  mNumRows = 1;
  mConstTerm.resize(mNumRows);
  mConstTerm.setZero();
  mWeight.resize(mNumRows);
  mWeight.setOnes();
}

L2NormConstraint::~L2NormConstraint()
{
}

Eigen::VectorXd L2NormConstraint::evalCon()
{
  size_t dim = mVariables.size();
  double sum = 0.0;
  for (size_t i = 0; i < dim; i++)
    sum += mVariables[i]->mVal * mVariables[i]->mVal;
  
  Eigen::VectorXd ret(1);
  ret[0] = sum;
  return ret;
}

double L2NormConstraint::evalObj()
{
  size_t dim = mVariables.size();
  double sum = 0.0;
  for (size_t i = 0; i < dim; i++)
    sum += mVariables[i]->mVal * mVariables[i]->mVal;
  
  return mWeight[0] * sum;
}

void L2NormConstraint::fillJac(VVD _jacobian, VVB _map, int _index)
{
  size_t dim = mVariables.size();
  for (size_t i = 0; i < dim; i++)
  {
    _jacobian->at(_index)->at(i) = 2.0 * mVariables[i]->mVal;
    _map->at(_index)->at(i) = true;
  }
}

void L2NormConstraint::updateParams(int _index)
{
}

void L2NormConstraint::fillObjGrad(std::vector<double>& _dG)
{
  size_t dim = mVariables.size();
  for (size_t i = 0; i < dim; i++)
    _dG[i] += mWeight[0] * 2.0 * mVariables[i]->mVal;
}
}
