#ifndef CAFFE_classifier
#define CAFFE_classifier

#include <caffe/caffe.hpp>
#include <memory>
#include "dart/dart.h"
#include "parameter.h"

using namespace caffe;


class Classifier {
  public:
  Classifier(const string& model_file, const string& trained_file,
             const string& preprocessing_file,
             int _numContactsToLearn = 1);

  void Eval(const Eigen::VectorXd& in_x, Eigen::VectorXd& out_y);
  int getNumChannels(){
    return num_channels_;
  }

  private:
  void mSetPreprocessing(const string& preprocessing_file);

  void NormalizeWhiten(Eigen::VectorXd& in_x);

  private:
  std::shared_ptr<Net<double>> net_;

  // input data size
  int num_channels_;
  // Normalization to [-1,1]
  Eigen::VectorXd mLowerBound;
  Eigen::VectorXd mUpperBound;
  Eigen::VectorXd mMean;
  // Whitening
  Eigen::MatrixXd mU;  // EigenVector
  Eigen::VectorXd mS;  // EigenValues

  // net id
  int numContactsToLearn;
  int numBasis;
};

#endif
