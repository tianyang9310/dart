#ifndef CAFFE_classifier
#define CAFFE_classifier

#include "dart/dart.h"
#include <caffe/caffe.hpp>
#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace caffe;

/* Pair (label, confidence) representing a prediction. */
typedef std::pair<string, float> Prediction;

class Classifier {
 public:
  Classifier(const string& model_file,
             const string& trained_file,
             const string& mean_file,
             const string& label_file,
             int numContactsToLearn = 1);

  std::vector<Prediction> Classify(const cv::Mat& img, int N = 5);

 private:
  void SetMean(const string& mean_file);

  void mSetPreprocessing(const string& preprocessing_file);

  std::vector<float> Predict(const cv::Mat& img);

  void WrapInputLayer(std::vector<cv::Mat>* input_channels);

  void Preprocess(const cv::Mat& img,
                  std::vector<cv::Mat>* input_channels);

 private:
  std::shared_ptr<Net<float> > net_;
  cv::Size input_geometry_;
  cv::Mat mean_;

  // input data size
  int num_channels_;
  // Normalization to [-1,1]
  Eigen::VectorXd mLowerBound;
  Eigen::VectorXd mUpperBound;
  Eigen::VectorXd mMean;
  // Whitening
  Eigen::MatrixXd mU; // EigenVector
  Eigen::VectorXd mS; // EigenValues

  std::vector<string> labels_;
};

#endif
