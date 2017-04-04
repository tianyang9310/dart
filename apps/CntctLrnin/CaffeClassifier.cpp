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
  net_.reset(new Net<double>(model_file, TEST));
  net_->CopyTrainedLayersFrom(trained_file);

  // Input and output blob numbers. 
  std::cerr << "Net num input: " << net_->num_inputs() << std::endl;
  std::cerr << "Net num output: " << net_->num_outputs() << std::endl;
  CHECK_EQ(net_->num_inputs(), 1) << "Network should have exactly one input.";
  CHECK_EQ(net_->num_outputs(), 1) << "Network should have exactly one output.";

  Blob<double>* input_layer = net_->input_blobs()[0];
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

  /*
  // Load labels. 
  std::ifstream labels(label_file.c_str());
  CHECK(labels) << "Unable to open labels file " << label_file;
  string line;
  while (std::getline(labels, line))
    labels_.push_back(string(line));

  Blob<double>* output_layer = net_->output_blobs()[0];
  CHECK_EQ(labels_.size(), output_layer->channels())
    << "Number of labels is different from the output layer dimension.";
  */

  Eigen::VectorXd in_x(num_channels_);
  Eigen::VectorXd out_y;
  in_x << 

// 3.923412243511365638,-1.5085911704039944858,-1.509981828925615055,1.5085911704039938197,1.5099818289256157211,3.9818032711416648795,-1.4813398109331630259,-3.9818032711416657676,1.4813398109331610275,3.3510809592022456904,1.4813398109331645802,-3.3510809592022452463,3.9818032711416670999,-1.4813398109331630259,3.3510809592022443582,4,-0.70709686801706383985,0.25904608006187823666,0.21929640051061866091,-0.25904608006187818114,-0.21929640051061877193;
// 2.9806112140825526957,-1.4075385708881347213,-0.62179593721841508458,1.4075385708881342772,0.62179593721841575071,3.0216994157190795178,0.091682884316219015464,-3.0216994157190795178,-0.091682884316220375487,0.52989818954128131701,-0.091682884316218626886,-0.52989818954128131701,3.0216994157190790737,0.09168288431622004242,0.52989818954128131701,4,-0.11618878782038308262,0.042989677640659348934,0.027653012666520745966,-0.042989677640659335056,-0.027653012666520759844;
// 2.9780021410199708498,-1.407757761321262846,-0.62100512852707423495,1.4077577613212624019,0.62100512852707490108,3.0246667155958366635,0.09131812801176580896,-3.0246667155958371076,-0.091318128011767141228,0.52966136059359925081,-0.091318128011765642427,-0.52966136059359925081,3.0246667155958366635,0.091318128011766891428,0.52966136059359913979,4,-0.0097604743720599001361,0.0047985828077742459508,0.00053287250192112813052,-0.0047985828077742390119,-0.00053287250192113333469;
// 3.9126564135760024676,-1.1061866854421411599,-1.8296461885101926725,1.1061866854421402717,1.8296461885101931166,4.6240842030441191568,-1.1442790353078973187,-4.624084203044120045,1.1442790353078953203,2.7415798179373598309,1.144279035307898873,-2.7415798179373593868,4.624084203044120045,-1.1442790353078970966,2.7415798179373589427,4,-0.1135785290260624969,-0.039184618817683800063,0.092303390597551021601,0.039184618817683848635,-0.092303390597550993846;
// 3.0338723343957743062,1.4261530924057916891,-0.58383665687543551126,-1.4261530924057921332,0.58383665687543484513,3.0670211818598414055,-0.095576062052524474177,-3.0670211818598414055,0.095576062052523058643,0.51277876904483621345,0.095576062052524848878,-0.51277876904483621345,3.0670211818598414055,-0.09557606205252339171,0.51277876904483621345,4,-0.0098072405211791606472,-0.0027941907612658248988,-0.0027830219826181501475,0.0027941907612658240315,0.0027830219826181536169;
// 2.9692294125306046837,1.514138983982103781,0.33162908531514256882,-1.514138983982103781,-0.33162908531514317945,2.2342253367179076839,0.92063365116151996137,-2.2342253367179072399,-0.92063365116152073853,1.0548401786221410781,-0.92063365116151973933,-1.0548401786221413001,2.2342253367179076839,0.92063365116152051648,1.0548401786221415222,4,-0.051877432360729870686,-0.042655093157888429589,0.0034303461041772865051,0.042655093157888443467,-0.0034303461041772665557;
// 2.9698856560102115942,1.5139113518995488405,0.33166201578810428074,-1.5139113518995483965,-0.3316620157881050579,2.2334943430159079902,0.92058818272160181095,-2.2334943430159071021,-0.92058818272160269913,1.0549807606154155692,-0.92058818272160136686,-1.0549807606154160133,2.233494343015906658,0.92058818272160214402,1.0549807606154164574,4,-0.0098573075719641765902,-0.0065588224757692926287,0.001519309819675162751,0.0065588224757692995676,-0.0015193098196751610163;
// 2.9708423341957588271,1.5136306245458275299,0.33176911327995728396,-1.5136306245458273079,-0.33176911327995789458,2.2324479903801059955,0.92048234593102717938,-2.2324479903801055514,-0.92048234593102840062,1.0551183081465163038,-0.92048234593102706835,-1.0551183081465167479,2.2324479903801051073,0.92048234593102784551,1.05511830814651697,4,-0.0098516023278101272354,0.0010554619249290228544,-0.0060997436805796259943,-0.0010554619249290228544,0.0060997436805796259943;
// 3.0337139503414198849,-1.5361488716757809581,-0.10651887068534721115,1.5361488716757809581,0.10651887068534790504,2.862799835336783616,-0.7087797303108425151,-2.8627998353367845041,0.70877973031084118283,0.84358233537042304206,0.70877973031084284816,-0.84358233537042282002,2.8627998353367845041,-0.70877973031084151589,0.84358233537042237593,4,-0.08084743323197622622,0.14790806739618639343,-0.061587200849811382652,-0.14790806739618642118,0.061587200849811306325;
// 3.0305008385824758754,-1.5377119366858091887,-0.071549347843358382382,1.5377119366858091887,0.071549347843359104027,2.832636284441131469,-0.74386993035054294054,-2.8326362844411319131,0.74386993035054160828,0.90188799644771644459,0.74386993035054327361,-0.9018879964477160005,2.8326362844411323572,-0.74386993035054205237,0.90188799644771577846,4,-0.0098024834590404764934,-0.00096482864036470314395,-0.00097616600340301711708,0.00096482864036470314395,0.0009761660034030166834;
// 3.9531634727547011465,-2.0876859147404531747,0.39978874329057811909,2.0876859147404531747,-0.39978874329057745296,2.3058637105935573253,0.80555073509376939889,-2.3058637105935568812,-0.80555073509377039809,5.1154983080425155961,-0.80555073509376773355,-5.1154983080425164843,2.3058637105935568812,0.80555073509376851071,5.1154983080425173725,4,-0.0097664425184112776263,0.0089303180219458613842,-0.0097176219922708306775,-0.0089303180219458683231,0.0097176219922708167998;
// 3.9493326434423545734,-2.0886448726122970321,0.40016613638698167454,2.0886448726122970321,-0.40016613638698067534,2.309635248343259839,0.80534010392930521771,-2.309635248343259839,-0.80534010392930643896,5.1155866454401017762,-0.80534010392930321931,-5.1155866454401026644,2.3096352483432593949,0.80534010392930444056,5.1155866454401026644,4,-0.0097799026076226119875,0.0099892754657597213064,-0.010771419244545089866,-0.0099892754657597282453,0.010771419244545075988;
// 2.9797127619774088636,-0.41487869574928670779,-1.480371009466252108,0.41487869574928598615,1.48037100946625233,1.3651950981926987971,0.87850456572735513561,-1.365195098192698353,-0.87850456572735580174,1.7466473902645502925,-0.87850456572735424743,-1.7466473902645507366,1.365195098192697909,0.87850456572735513561,1.7466473902645511806,4,-0.14121117375266784011,0.05001161414054841603,0.13803184210387017816,-0.050011614140548346641,-0.13803184210387020592;


  Eval(in_x, out_y);
  std::cout << "in_x: " << std::endl << in_x.transpose() << std::endl;
  std::cout << "out_y: " << std::endl << out_y.transpose() << std::endl;
}

static bool PairCompare(const std::pair<double, int>& lhs,
                        const std::pair<double, int>& rhs) {
  return lhs.first > rhs.first;
}

/* Return the indices of the top N values of vector v. */
static std::vector<int> Argmax(const std::vector<double>& v, int N) {
  std::vector<std::pair<double, int> > pairs;
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
  std::vector<double> output = Predict(img);

  N = std::min<int>(labels_.size(), N);
  std::vector<int> maxN = Argmax(output, N);
  std::vector<Prediction> predictions;
  for (int i = 0; i < N; ++i) {
    int idx = maxN[i];
    predictions.push_back(std::make_pair(labels_[idx], output[idx]));
  }

  return predictions;
}

void Classifier::Eval(const Eigen::VectorXd& in_x, Eigen::VectorXd& out_y) {
  int inputSize = in_x.size();
  assert(num_channels_ == inputSize);

  const std::vector<int> shape = {1,inputSize};
  caffe::Blob<double> blob(shape);
  double* blob_data = blob.mutable_cpu_data();
  std::cout << "Net count: " << blob.count() << std::endl;

  Eigen::VectorXd normWhite_x = in_x;
  NormalizeWhiten(normWhite_x);

  for (int i = 0; i < blob.count(); ++i)
  {
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

  for (int i = 0; i < outputSize; ++i)
  {
    out_y[i] = result_data[i];
  }
}

void Classifier::NormalizeWhiten(Eigen::VectorXd& in_x) {
  // std::cout << "Before normalization: " << std::endl << in_x << std::endl;
  assert(num_channels_ == in_x.size());

  // normlize to [-1, 1]
  in_x = 2 * (in_x - mLowerBound).array() / (mUpperBound - mLowerBound).array() - 1;
  // std::cout << "After normalization: " << std::endl << in_x << std::endl;

  // Use train data to whitening data
  in_x = in_x - mMean;
  in_x = (in_x.transpose() * mU).eval().transpose();
  in_x = in_x.array() / (mS.array() + 1e-5).sqrt();
  // std::cout << "After whitening: " << std::endl << in_x << std::endl;
}

/* Load the mean file in binaryproto format. */
void Classifier::SetMean(const string& mean_file) {
  BlobProto blob_proto;
  ReadProtoFromBinaryFileOrDie(mean_file.c_str(), &blob_proto);

  /* Convert from BlobProto to Blob<double> */
  Blob<double> mean_blob;
  mean_blob.FromProto(blob_proto);
  CHECK_EQ(mean_blob.channels(), num_channels_)
    << "Number of channels of mean file doesn't match input layer.";

  /* The format of the mean file is planar 32-bit double BGR or grayscale. */
  std::vector<cv::Mat> channels;
  double* data = mean_blob.mutable_cpu_data();
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

  // std::cout << "mLowerBound: " << std::endl << std::setprecision(20) << mLowerBound << std::endl;
  // std::cout << "mUpperBound: " << std::endl << std::setprecision(20) << mUpperBound << std::endl;
  // std::cout << "mMean: " << std::endl << std::setprecision(20) << mMean << std::endl;
  // std::cout << "mS: " << std::endl << std::setprecision(20) << mS << std::endl;
  // std::cout << "mU: " << std::endl << std::setprecision(20) << mU << std::endl;
  // std::cin.get();
}

std::vector<double> Classifier::Predict(const cv::Mat& img) {
  Blob<double>* input_layer = net_->input_blobs()[0];
  input_layer->Reshape(1, num_channels_,
                       input_geometry_.height, input_geometry_.width);
  /* Forward dimension change to all layers. */
  net_->Reshape();

  std::vector<cv::Mat> input_channels;
  WrapInputLayer(&input_channels);

  Preprocess(img, &input_channels);

  net_->Forward();

  /* Copy the output layer to a std::vector */
  Blob<double>* output_layer = net_->output_blobs()[0];
  const double* begin = output_layer->cpu_data();
  const double* end = begin + output_layer->channels();
  return std::vector<double>(begin, end);
}

/* Wrap the input layer of the network in separate cv::Mat objects
 * (one per channel). This way we save one memcpy operation and we
 * don't need to rely on cudaMemcpy2D. The last preprocessing
 * operation will write the separate channels directly to the input
 * layer. */
void Classifier::WrapInputLayer(std::vector<cv::Mat>* input_channels) {
  Blob<double>* input_layer = net_->input_blobs()[0];

  int width = input_layer->width();
  int height = input_layer->height();
  double* input_data = input_layer->mutable_cpu_data();
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

  cv::Mat sample_double;
  if (num_channels_ == 3)
    sample_resized.convertTo(sample_double, CV_32FC3);
  else
    sample_resized.convertTo(sample_double, CV_32FC1);

  cv::Mat sample_normalized;
  cv::subtract(sample_double, mean_, sample_normalized);

  /* This operation will write the separate BGR planes directly to the
   * input layer of the network because it is wrapped by the cv::Mat
   * objects in input_channels. */
  cv::split(sample_normalized, *input_channels);

  CHECK(reinterpret_cast<double*>(input_channels->at(0).data)
        == net_->input_blobs()[0]->cpu_data())
    << "Input channels are not wrapping the input layer of the network.";
}
