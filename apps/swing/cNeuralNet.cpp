#include "cNeuralNet.h"

cNeuralNet::cNeuralNet() {
  mNet = NULL;
}

cNeuralNet::~cNeuralNet() {}

void cNeuralNet::loadNet(const std::string& net_file, caffe::Phase phase) {
  if (net_file != "") {
    mNet = std::unique_ptr<caffe::Net<tNNData>>(
        new caffe::Net<tNNData>(net_file, phase));
  }
}

std::shared_ptr<caffe::Net<cNeuralNet::tNNData>> cNeuralNet::getNet() {
  return mNet;  
}
