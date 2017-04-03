#include "CaffeClassifier.h"
#include "dist-json/json/json.h"
#include <fstream>
#include "JsonUtil.h"

Classifier::Classifier(const string& model_file,
                       const string& trained_file,
                       const string& mean_file,
                       const string& label_file,
                       int numContactsToLearn) {
#ifdef CPU_ONLY
  Caffe::set_mode(Caffe::CPU);
#else
  Caffe::set_mode(Caffe::GPU);
#endif

  /* Load the network. */
  net_.reset(new Net<float>(model_file, TEST));
  net_->CopyTrainedLayersFrom(trained_file);

  // Input and output blob numbers. 
  std::cerr << "Net num input: " << net_->num_inputs() << std::endl;
  std::cerr << "Net num output: " << net_->num_outputs() << std::endl;
  CHECK_EQ(net_->num_inputs(), 1) << "Network should have exactly one input.";
  CHECK_EQ(net_->num_outputs(), 1) << "Network should have exactly one output.";

  Blob<float>* input_layer = net_->input_blobs()[0];
  num_channels_ = input_layer->channels();
  std::cerr << "Net input layer num channels: " << num_channels_ << std::endl;

  int numBasis = 4;
  int ASize = numContactsToLearn * (numBasis + 1);
  int inputSize = ASize + (ASize + 1) * ASize / 2 + numContactsToLearn;

  CHECK(num_channels_ == inputSize)
     << "Input layer channels should be " << inputSize;
  // input_geometry_ = cv::Size(input_layer->width(), input_layer->height());

  // /* Load the binaryproto mean file. */
  // SetMean(mean_file);
  std::string preprocessing_file = DART_ROOT_PATH"apps/CntctLrnin/CaffeNet/numContactToLearn_1/PreprocessingData.json";
  mSetPreprocessing(preprocessing_file);

  /* Load labels. */
  std::ifstream labels(label_file.c_str());
  CHECK(labels) << "Unable to open labels file " << label_file;
  string line;
  while (std::getline(labels, line))
    labels_.push_back(string(line));

  Blob<float>* output_layer = net_->output_blobs()[0];
  CHECK_EQ(labels_.size(), output_layer->channels())
    << "Number of labels is different from the output layer dimension.";
}

static bool PairCompare(const std::pair<float, int>& lhs,
                        const std::pair<float, int>& rhs) {
  return lhs.first > rhs.first;
}

/* Return the indices of the top N values of vector v. */
static std::vector<int> Argmax(const std::vector<float>& v, int N) {
  std::vector<std::pair<float, int> > pairs;
  for (size_t i = 0; i < v.size(); ++i)
    pairs.push_back(std::make_pair(v[i], i));
  std::partial_sort(pairs.begin(), pairs.begin() + N, pairs.end(), PairCompare);

  std::vector<int> result;
  for (int i = 0; i < N; ++i)
    result.push_back(pairs[i].second);
  return result;
}

/* Return the top N predictions. */
std::vector<Prediction> Classifier::Classify(const cv::Mat& img, int N) {
  std::vector<float> output = Predict(img);

  N = std::min<int>(labels_.size(), N);
  std::vector<int> maxN = Argmax(output, N);
  std::vector<Prediction> predictions;
  for (int i = 0; i < N; ++i) {
    int idx = maxN[i];
    predictions.push_back(std::make_pair(labels_[idx], output[idx]));
  }

  return predictions;
}

/* Load the mean file in binaryproto format. */
void Classifier::SetMean(const string& mean_file) {
  BlobProto blob_proto;
  ReadProtoFromBinaryFileOrDie(mean_file.c_str(), &blob_proto);

  /* Convert from BlobProto to Blob<float> */
  Blob<float> mean_blob;
  mean_blob.FromProto(blob_proto);
  CHECK_EQ(mean_blob.channels(), num_channels_)
    << "Number of channels of mean file doesn't match input layer.";

  /* The format of the mean file is planar 32-bit float BGR or grayscale. */
  std::vector<cv::Mat> channels;
  float* data = mean_blob.mutable_cpu_data();
  for (int i = 0; i < num_channels_; ++i) {
    /* Extract an individual channel. */
    cv::Mat channel(mean_blob.height(), mean_blob.width(), CV_32FC1, data);
    channels.push_back(channel);
    data += mean_blob.height() * mean_blob.width();
  }

  /* Merge the separate channels into a single image. */
  cv::Mat mean;
  cv::merge(channels, mean);

  /* Compute the global mean pixel value and create a mean image
   * filled with this value. */
  cv::Scalar channel_mean = cv::mean(mean);
  mean_ = cv::Mat(input_geometry_, mean.type(), channel_mean);
}

void Classifier::mSetPreprocessing(const string& preprocessing_file) {
  const std::string gLowerBoundKey = "lower bound";
  const std::string gUpperBoundKey = "upper bound";
  const std::string gMeanKey = "mean";
  const std::string gUKey = "U";
  const std::string gSKey = "S";

  std::cout << "Net preprocessing_file: " << preprocessing_file << std::endl;
  std::ifstream f_stream(preprocessing_file);
  Json::Reader reader;
  Json::Value root;
  bool succ = reader.parse(f_stream, root);
  f_stream.close();

  if (succ && !root[gLowerBoundKey].isNull()) {
    Eigen::VectorXd mLowerBound_;
    succ &=cJsonUtil::ReadVectorJson(root[gLowerBoundKey], mLowerBound_);

    int mLowerBound_size = static_cast<int>(mLowerBound_.size());
    if (mLowerBound_size == num_channels_) {
      mLowerBound = mLowerBound_;
    } else {
      std::cout << "Invalid lower bound size" << std::endl;
    }
  }

  if (succ && !root[gUpperBoundKey].isNull()) {
    Eigen::VectorXd mUpperBound_;
    succ &=cJsonUtil::ReadVectorJson(root[gUpperBoundKey], mUpperBound_);

    int mUpperBound_size = static_cast<int>(mUpperBound_.size());
    if (mUpperBound_size == num_channels_) {
      mUpperBound = mUpperBound_;
    } else {
      std::cout << "Invalid upper bound size" << std::endl;
    }
  }

  if (succ && !root[gMeanKey].isNull()) {
    Eigen::VectorXd mMean_;
    succ &=cJsonUtil::ReadVectorJson(root[gMeanKey], mMean_);

    int mMean_size = static_cast<int>(mMean_.size());
    if (mMean_size == num_channels_) {
      mMean = mMean_;
    } else {
      std::cout << "Invalid mean size" << std::endl;
    }
  }

  if (succ && !root[gSKey].isNull()) {
    Eigen::VectorXd mS_;
    succ &=cJsonUtil::ReadVectorJson(root[gSKey], mS_);

    int mS_size = static_cast<int>(mS_.size());
    if (mS_size == num_channels_) {
      mS = mS_;
    } else {
      std::cout << "Invalid S size" << std::endl;
    }
  }

  if (succ && !root[gUKey].isNull()) {
    Eigen::MatrixXd mU_;
    succ &=cJsonUtil::ReadMatrixJson(root[gUKey], mU_);

    int mU_row = static_cast<int>(mU_.rows());
    int mU_col = static_cast<int>(mU_.cols());
    if (mU_row == num_channels_ && mU_col == num_channels_) {
      mU = mU_;
    } else {
      std::cout << "Invalid U size" << std::endl;
    }
  }

  std::cout << "mLowerBound: " << std::endl << std::setprecision(20) << mLowerBound << std::endl;
  std::cout << "mUpperBound: " << std::endl << std::setprecision(20) << mUpperBound << std::endl;
  std::cout << "mMean: " << std::endl << std::setprecision(20) << mMean << std::endl;
  std::cout << "mS: " << std::endl << std::setprecision(20) << mS << std::endl;
  std::cout << "mU: " << std::endl << std::setprecision(20) << mU << std::endl;
  std::cin.get();
}

std::vector<float> Classifier::Predict(const cv::Mat& img) {
  Blob<float>* input_layer = net_->input_blobs()[0];
  input_layer->Reshape(1, num_channels_,
                       input_geometry_.height, input_geometry_.width);
  /* Forward dimension change to all layers. */
  net_->Reshape();

  std::vector<cv::Mat> input_channels;
  WrapInputLayer(&input_channels);

  Preprocess(img, &input_channels);

  net_->Forward();

  /* Copy the output layer to a std::vector */
  Blob<float>* output_layer = net_->output_blobs()[0];
  const float* begin = output_layer->cpu_data();
  const float* end = begin + output_layer->channels();
  return std::vector<float>(begin, end);
}

/* Wrap the input layer of the network in separate cv::Mat objects
 * (one per channel). This way we save one memcpy operation and we
 * don't need to rely on cudaMemcpy2D. The last preprocessing
 * operation will write the separate channels directly to the input
 * layer. */
void Classifier::WrapInputLayer(std::vector<cv::Mat>* input_channels) {
  Blob<float>* input_layer = net_->input_blobs()[0];

  int width = input_layer->width();
  int height = input_layer->height();
  float* input_data = input_layer->mutable_cpu_data();
  for (int i = 0; i < input_layer->channels(); ++i) {
    cv::Mat channel(height, width, CV_32FC1, input_data);
    input_channels->push_back(channel);
    input_data += width * height;
  }
}

void Classifier::Preprocess(const cv::Mat& img,
                            std::vector<cv::Mat>* input_channels) {
  /* Convert the input image to the input image format of the network. */
  cv::Mat sample;
  if (img.channels() == 3 && num_channels_ == 1)
    cv::cvtColor(img, sample, cv::COLOR_BGR2GRAY);
  else if (img.channels() == 4 && num_channels_ == 1)
    cv::cvtColor(img, sample, cv::COLOR_BGRA2GRAY);
  else if (img.channels() == 4 && num_channels_ == 3)
    cv::cvtColor(img, sample, cv::COLOR_BGRA2BGR);
  else if (img.channels() == 1 && num_channels_ == 3)
    cv::cvtColor(img, sample, cv::COLOR_GRAY2BGR);
  else
    sample = img;

  cv::Mat sample_resized;
  if (sample.size() != input_geometry_)
    cv::resize(sample, sample_resized, input_geometry_);
  else
    sample_resized = sample;

  cv::Mat sample_float;
  if (num_channels_ == 3)
    sample_resized.convertTo(sample_float, CV_32FC3);
  else
    sample_resized.convertTo(sample_float, CV_32FC1);

  cv::Mat sample_normalized;
  cv::subtract(sample_float, mean_, sample_normalized);

  /* This operation will write the separate BGR planes directly to the
   * input layer of the network because it is wrapped by the cv::Mat
   * objects in input_channels. */
  cv::split(sample_normalized, *input_channels);

  CHECK(reinterpret_cast<float*>(input_channels->at(0).data)
        == net_->input_blobs()[0]->cpu_data())
    << "Input channels are not wrapping the input layer of the network.";
}
