#ifndef DART_NEURALNET
#define DART_NEURALNET

#include <caffe/caffe.hpp>

class cNeuralNet {
  public:
  typedef double tNNData;

  // member function
  cNeuralNet();
  virtual ~cNeuralNet();
  void loadNet(const std::string& net_file, caffe::Phase phase);
  std::shared_ptr<caffe::Net<tNNData>> getNet();

  // member variable
  std::shared_ptr<caffe::Net<tNNData>> mNet;
};

#endif
