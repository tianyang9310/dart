#include "MyLCPConstraint.h"
#include "Var.h"

namespace qpcc {

MyLCPConstraint::MyLCPConstraint(std::vector<Var *>& _var,
                             const Eigen::MatrixXd& a,
                             const Eigen::VectorXd& b) :
  Constraint(_var),
  mA(a),
  mB(b)
{
  mNumRows = b.size();

  mConstTerm.resize(mNumRows);
  // mConstTerm = Eigen::VectorXd::Constant(mNumRows,0.0001);
  mConstTerm.setZero();

  mWeight.resize(mNumRows);
  mWeight.setOnes();

  mEquality = 2;
  // mEquality = 0;
}

MyLCPConstraint::~MyLCPConstraint() {
}

Eigen::VectorXd MyLCPConstraint::evalCon()
{
  int dim_var = mA.cols();
  int dim_cnst = mA.rows();
  Eigen::VectorXd x(dim_var);
  for (size_t i = 0; i < dim_var; i++)
    x[i] = mVariables[i]->mVal;

  Eigen::VectorXd ret(dim_cnst);
  ret = ((mA * x - mB).array() * x.array()).matrix();

  std::cout << "LCP Constraint: " << ret.transpose() << std::endl;
  return ret;
}

void MyLCPConstraint::fillJac(VVD _jacobian, VVB _map, int _index)
{
  int dim_var = mA.cols();
  int dim_cnst = mA.rows();
  assert(dim_var == dim_cnst);
  Eigen::VectorXd x(dim_var);
  for (size_t i = 0; i < dim_var; i++)
    x[i] = mVariables[i]->mVal;

  Eigen::MatrixXd diag = (mA*x-mB).asDiagonal();
  Eigen::MatrixXd tmpJ(dim_cnst, dim_var);
  // for (size_t i = 0; i < dim_var; i++) 
  //   // TODO: using colwise to optimize code
  //   tmpJ.col(i) = (mA.col(i).array() * x.array()).matrix();
  // }
  tmpJ = (mA.array().colwise() * x.array()).matrix();
  tmpJ = tmpJ + diag;
  // std::cout << "Diag: " << diag << std::endl;
  // std::cout << "Jacobian: " << tmpJ << std::endl;
  for (size_t i = 0; i < dim_cnst; i++) {
    for (size_t j = 0; j < dim_var; j++) {
      _jacobian->at(_index+i)->at(j) = tmpJ(i,j);
      _map->at(_index + i)->at(j) = true;
    }
  }
}

void MyLCPConstraint::updateParams(int _index) {
}

//void MyLCPConstraint::fillObjGrad(std::vector<double>& _dG) {
//  Eigen::VectorXd diff = evalCon();
//  int nBody = mSkeleton->getNumBodyNodes();
//  for (int i = 0; i < nBody; i++)
//    _dG[i] += diff[0];
//}
}
