#include "cNNsolver.h"

cNNSolver::cNNSolver() {
  mSolver = NULL;
}

cNNSolver::~cNNSolver() {}

void cNNSolver::buildSolver(const std::string& solver_file) {
  caffe::SolverParameter param;
  caffe::ReadProtoFromTextFileOrDie(solver_file, &param);
  caffe::Caffe::set_mode(caffe::Caffe::CPU);
  caffe::SolverParameter_SolverType type = param.solver_type();

  switch (type) {
    case caffe::SolverParameter_SolverType_SGD:
      mSolver = new cSGDSolver(param);
      break;
    case caffe::SolverParameter_SolverType_NESTEROV:
      mSolver = new cNesterovSolver(param);
      break;
    case caffe::SolverParameter_SolverType_ADAGRAD:
      mSolver = new cAdaGradSolver(param);
      break;
    case caffe::SolverParameter_SolverType_RMSPROP:
      mSolver = new cRMSPropSolver(param);
      break;
    case caffe::SolverParameter_SolverType_ADADELTA:
      mSolver = new cAdaDeltaSolver(param);
      break;
    case caffe::SolverParameter_SolverType_ADAM:
      mSolver = new cAdamSolver(param);
      break;
    default:
      LOG(FATAL) << "Unknown SolverType: " << type;
  }

  mTrainNet = mSolver->net();
}
