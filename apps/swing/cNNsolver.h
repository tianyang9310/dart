#ifndef DART_NNSOLVER
#define DART_NNSOLVER

#include <caffe/caffe.hpp>
#include <caffe/sgd_solvers.hpp>
#include "cNeuralNet.h"

template <typename tSolverType>
class cCaffeSolver : public tSolverType {
  public:
  cCaffeSolver(const caffe::SolverParameter& param) : tSolverType(param){};
  virtual ~cCaffeSolver(){};

  virtual boost::shared_ptr<caffe::Net<cNeuralNet::tNNData>> GetNet();
  virtual void ApplySteps(int iters);
  virtual cNeuralNet::tNNData ForwardBackward();
};

template <typename tSolverType>
boost::shared_ptr<caffe::Net<cNeuralNet::tNNData>>
cCaffeSolver<tSolverType>::GetNet() {
  return tSolverType::net();
}

template <typename tSolverType>
void cCaffeSolver<tSolverType>::ApplySteps(int steps) {
  tSolverType::Step(steps);
};

template <typename tSolverType>
cNeuralNet::tNNData cCaffeSolver<tSolverType>::ForwardBackward() {
  cNeuralNet::tNNData loss = 0;
  this->GetNet()->ClearParamDiffs();
  loss = this->GetNet()->ForwardBackward();
  return loss;
};

typedef cCaffeSolver<caffe::SGDSolver<cNeuralNet::tNNData>> cSGDSolver;
typedef cCaffeSolver<caffe::NesterovSolver<cNeuralNet::tNNData>>cNesterovSolver;
typedef cCaffeSolver<caffe::AdaGradSolver<cNeuralNet::tNNData>> cAdaGradSolver;
typedef cCaffeSolver<caffe::RMSPropSolver<cNeuralNet::tNNData>> cRMSPropSolver;
typedef cCaffeSolver<caffe::AdaDeltaSolver<cNeuralNet::tNNData>> cAdaDeltaSolver;
typedef cCaffeSolver<caffe::AdamSolver<cNeuralNet::tNNData>> cAdamSolver;


class cNNSolver {
  public:
  // member function
  cNNSolver();
  virtual ~cNNSolver();
  void buildSolver(const std::string& solver_file);

  // member variable
  caffe::Net<cNeuralNet::tNNData>* mTrainNet;
  caffe::Net<cNeuralNet::tNNData>* mTestNet;
  caffe::Solver<cNeuralNet::tNNData>* mSolver;
};
#endif
