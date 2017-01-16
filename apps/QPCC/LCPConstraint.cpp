#include "LCPConstraint.h"
#include "Var.h"

namespace qpcc {

LCPConstraint::LCPConstraint(std::vector<Var *>& _var,
                             const Eigen::VectorXd& a,
                             double b,
                             int rowIndex) :
  Constraint(_var),
  mA(a),
  mB(b),
  mRowIndex(rowIndex)

{
  mNumRows = 1;
  mConstTerm.resize(1);
  mConstTerm[0] = 0.0001;
  mWeight.resize(1);
  mWeight << 1.0;
  mEquality = 2;
}

LCPConstraint::~LCPConstraint() {
}

Eigen::VectorXd LCPConstraint::evalCon()
{
  int dim = mA.size();
  Eigen::VectorXd x(dim);
  for (size_t i = 0; i < dim; i++)
    x[i] = mVariables[i]->mVal;
  Eigen::VectorXd ret(1);
  ret[0] = x[mRowIndex] * (mA.dot(x) - mB);
  std::cout << "LCP Constraint " << mRowIndex << ": " << ret << std::endl;
  return ret;
}

void LCPConstraint::fillJac(VVD _jacobian, VVB _map, int _index)
{
  int dim = mA.size();
  double x = mVariables[mRowIndex]->mVal;
  double dotProduct = 0.0;
  for (size_t i = 0; i < dim; i++)
  {
    dotProduct += mA[i] * mVariables[i]->mVal;
    _jacobian->at(_index)->at(i) = mA[i] * x;
    _map->at(_index)->at(i) = true;
  }
  _jacobian->at(_index)->at(mRowIndex) =
    dotProduct + mA[mRowIndex] * x - mB;
}

void LCPConstraint::updateParams(int _index) {
}

//void LCPConstraint::fillObjGrad(std::vector<double>& _dG) {
//  Eigen::VectorXd diff = evalCon();
//  int nBody = mSkeleton->getNumBodyNodes();
//  for (int i = 0; i < nBody; i++)
//    _dG[i] += diff[0];
//}
}
