#include "CaffeClassifier.h"
#include <fstream>
#include "JsonUtil.h"
#include "dist-json/json/json.h"

Classifier::Classifier(const string& model_file, const string& trained_file,
                       const string& preprocessing_file,
                       int numContactsToLearn) {
#ifdef CPU_ONLY
  Caffe::set_mode(Caffe::CPU);
#else
  Caffe::set_mode(Caffe::GPU);
#endif

  /* Load the network. */
  net_.reset(new Net<double>(model_file, TEST));
  net_->CopyTrainedLayersFrom(trained_file);

  // Input and output blob numbers.
  // std::cerr << "Net num input: " << net_->num_inputs() << std::endl;
  // std::cerr << "Net num output: " << net_->num_outputs() << std::endl;
  CHECK_EQ(net_->num_inputs(), 1) << "Network should have exactly one input.";
  CHECK_EQ(net_->num_outputs(), 1) << "Network should have exactly one output.";

  Blob<double>* input_layer = net_->input_blobs()[0];
  num_channels_ = input_layer->channels();
  // std::cerr << "Net input layer num channels: " << num_channels_ << std::endl;

  int numBasis = 4;
  int ASize = numContactsToLearn * (numBasis + 1);
  int inputSize = ASize + (ASize + 1) * ASize / 2 + numContactsToLearn;

  CHECK(num_channels_ == inputSize) << "Input layer channels should be "
                                    << inputSize;
  mSetPreprocessing(preprocessing_file);
}

void Classifier::Eval(const Eigen::VectorXd& in_x, Eigen::VectorXd& out_y) {
  int inputSize = in_x.size();
  assert(num_channels_ == inputSize);

  const std::vector<int> shape = {1, inputSize};
  caffe::Blob<double> blob(shape);
  double* blob_data = blob.mutable_cpu_data();
  // std::cout << "Net count: " << blob.count() << std::endl;

  Eigen::VectorXd normWhite_x = in_x;
  NormalizeWhiten(normWhite_x);

  for (int i = 0; i < blob.count(); ++i) {
    blob_data[i] = normWhite_x[i];
  }

  const std::vector<caffe::Blob<double>*>& input_blobs = net_->input_blobs();
  input_blobs[0]->CopyFrom(blob);
  const std::vector<caffe::Blob<double>*>& result_arr = net_->Forward();

  // Fetch output
  const caffe::Blob<double>* result = result_arr[0];
  const double* result_data = result->cpu_data();

  assert(result->count() == net_->num_outputs());
  int outputSize = result->count();
  out_y.resize(outputSize);

  for (int i = 0; i < outputSize; ++i) {
    out_y[i] = result_data[i];
  }
}

void Classifier::NormalizeWhiten(Eigen::VectorXd& in_x) {
  // std::cout << "Before normalization: " << std::endl << in_x << std::endl;
  assert(num_channels_ == in_x.size());

  // normlize to [-1, 1]
  in_x =
      2 * (in_x - mLowerBound).array() / (mUpperBound - mLowerBound).array() -
      1;
  // std::cout << "After normalization: " << std::endl << in_x << std::endl;

  // Use train data to whitening data
  in_x = in_x - mMean;
  in_x = (in_x.transpose() * mU).eval().transpose();
  in_x = in_x.array() / (mS.array() + 1e-5).sqrt();
  // std::cout << "After whitening: " << std::endl << in_x << std::endl;
}

void Classifier::mSetPreprocessing(const string& preprocessing_file) {
  const std::string gLowerBoundKey = "lower bound";
  const std::string gUpperBoundKey = "upper bound";
  const std::string gMeanKey = "mean";
  const std::string gUKey = "U";
  const std::string gSKey = "S";

  // std::cout << "Net preprocessing_file: " << preprocessing_file << std::endl;
  std::ifstream f_stream(preprocessing_file);
  Json::Reader reader;
  Json::Value root;
  bool succ = reader.parse(f_stream, root);
  f_stream.close();

  if (succ && !root[gLowerBoundKey].isNull()) {
    Eigen::VectorXd mLowerBound_;
    succ &= cJsonUtil::ReadVectorJson(root[gLowerBoundKey], mLowerBound_);

    int mLowerBound_size = static_cast<int>(mLowerBound_.size());
    if (mLowerBound_size == num_channels_) {
      mLowerBound = mLowerBound_;
    } else {
      std::cout << "Invalid lower bound size" << std::endl;
    }
  }

  if (succ && !root[gUpperBoundKey].isNull()) {
    Eigen::VectorXd mUpperBound_;
    succ &= cJsonUtil::ReadVectorJson(root[gUpperBoundKey], mUpperBound_);

    int mUpperBound_size = static_cast<int>(mUpperBound_.size());
    if (mUpperBound_size == num_channels_) {
      mUpperBound = mUpperBound_;
    } else {
      std::cout << "Invalid upper bound size" << std::endl;
    }
  }

  if (succ && !root[gMeanKey].isNull()) {
    Eigen::VectorXd mMean_;
    succ &= cJsonUtil::ReadVectorJson(root[gMeanKey], mMean_);

    int mMean_size = static_cast<int>(mMean_.size());
    if (mMean_size == num_channels_) {
      mMean = mMean_;
    } else {
      std::cout << "Invalid mean size" << std::endl;
    }
  }

  if (succ && !root[gSKey].isNull()) {
    Eigen::VectorXd mS_;
    succ &= cJsonUtil::ReadVectorJson(root[gSKey], mS_);

    int mS_size = static_cast<int>(mS_.size());
    if (mS_size == num_channels_) {
      mS = mS_;
    } else {
      std::cout << "Invalid S size" << std::endl;
    }
  }

  if (succ && !root[gUKey].isNull()) {
    Eigen::MatrixXd mU_;
    succ &= cJsonUtil::ReadMatrixJson(root[gUKey], mU_);

    int mU_row = static_cast<int>(mU_.rows());
    int mU_col = static_cast<int>(mU_.cols());
    if (mU_row == num_channels_ && mU_col == num_channels_) {
      mU = mU_;
    } else {
      std::cout << "Invalid U size" << std::endl;
    }
  }

  // std::cout << "mLowerBound: " << std::endl << std::setprecision(20) <<
  // mLowerBound << std::endl;
  // std::cout << "mUpperBound: " << std::endl << std::setprecision(20) <<
  // mUpperBound << std::endl;
  // std::cout << "mMean: " << std::endl << std::setprecision(20) << mMean <<
  // std::endl;
  // std::cout << "mS: " << std::endl << std::setprecision(20) << mS <<
  // std::endl;
  // std::cout << "mU: " << std::endl << std::setprecision(20) << mU <<
  // std::endl;
  // std::cin.get();
}
