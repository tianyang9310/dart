#ifndef CAFFE_LP_SOLVER
#define CAFFE_LP_SOLVER

#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include "CaffeClassifier.h"
#include "LCPLS.h"
#include "apps/lemkeFix/myLemke.h"
#include "parameter.h"

class CaffeLPSolver {
  public:
  CaffeLPSolver(int _numContactsToLearn);
  virtual ~CaffeLPSolver(){};
  void solve(int idxContact, const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
             Eigen::VectorXd& z);

  private:
  Eigen::VectorXd Aandb2input(const Eigen::MatrixXd& A,
                              const Eigen::VectorXd& b, int idxContact);
  std::vector<std::shared_ptr<Classifier>> mCaffeClassifiers;
  int numContactsToLearn;
  int numBasis;
};

#endif
