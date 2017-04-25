#include "CaffeLPSolver.h"

inline bool mExist(const std::string& name) {
  std::ifstream file(name);
  if (file) {
    std::cout << name << " exists..." << std::endl;
  }
  return bool(file);
}

void permuteAandBforTest(Eigen::MatrixXd& newA, Eigen::VectorXd& newb,
                         const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                         int idx0, int idx1, int numContactsCallBack,
                         int numBasis) {
  assert(newA.size() == A.size());
  assert(newb.size() == b.size());
  assert(newb.size() == (numBasis + 2) * numContactsCallBack);
  assert(idx0 >= 0 && idx0 < numContactsCallBack);
  assert(idx1 >= 0 && idx1 < numContactsCallBack);

  int mDim = numBasis + 1;
  Eigen::MatrixXd T(mDim * numContactsCallBack, mDim * numContactsCallBack);
  T.setIdentity();

  T.row(idx0).swap(T.row(idx1));
  T.block(numContactsCallBack + idx0 * numBasis, 0, numBasis,
          mDim * numContactsCallBack)
      .swap(T.block(numContactsCallBack + idx1 * numBasis, 0, numBasis,
                    mDim * numContactsCallBack));
  assert(T * T.transpose() == Eigen::MatrixXd::Identity(T.rows(), T.cols()));
  newA.topLeftCorner(T.rows(), T.cols()) =
      T * A.topLeftCorner(T.rows(), T.cols()) * T.transpose();
  newb.head(T.rows()) = T * b.head(T.rows());

  // permute friction coefficients
  T.resize(numContactsCallBack, numContactsCallBack);
  T.setIdentity();
  T.row(idx0).swap(T.row(idx1));
  assert(T * T.transpose() == Eigen::MatrixXd::Identity(T.rows(), T.cols()));
  newA.bottomLeftCorner(numContactsCallBack, numContactsCallBack) =
      T * A.bottomLeftCorner(numContactsCallBack, numContactsCallBack) *
      T.transpose();
}

CaffeLPSolver::CaffeLPSolver(int _numContactsToLearn)
    : numContactsToLearn(_numContactsToLearn) {
  numBasis = NUMBASIS;

  // load all CaffeClassifiers
  for (int i = 0; i < numContactsToLearn; ++i) {
    std::string mDART_ROOT_PATH(DART_ROOT_PATH);

    std::string model_file = mDART_ROOT_PATH +
                             "apps/CntctLrnin/CaffeNet/numContactToLearn_" +
                             std::to_string(i + 1) + "/deploy.prototxt";
    std::string trained_file =
        mDART_ROOT_PATH + "apps/CntctLrnin/CaffeNet/numContactToLearn_" +
        std::to_string(i + 1) + "/train_iter_100000.caffemodel";
    std::string preprocessing_file =
        mDART_ROOT_PATH + "apps/CntctLrnin/CaffeNet/numContactToLearn_" +
        std::to_string(i + 1) + "/PreprocessingData.json";

    if (mExist(model_file) && mExist(trained_file) &&
        mExist(preprocessing_file)) {
      std::cout << "Loading caffe classifer " << i + 1 << std::endl;
      std::shared_ptr<Classifier> classifier = std::make_shared<Classifier>(
          model_file, trained_file, preprocessing_file, i + 1);
      mCaffeClassifiers.push_back(classifier);
    } else {
      std::cerr << "Some files don't exsit, so not all files can be loaded..."
                << std::endl;
      std::cerr << "model file: " << std::boolalpha << mExist(model_file)
                << std::endl;
      std::cerr << "trained file: " << std::boolalpha << mExist(trained_file)
                << std::endl;
      std::cerr << "preprocessing file: " << std::boolalpha
                << mExist(preprocessing_file) << std::endl;
      std::cin.get();
    }
  }
}

void CaffeLPSolver::solve(int idxContact, const Eigen::MatrixXd& A,
                          const Eigen::VectorXd& b, Eigen::VectorXd& z) {
  std::vector<int> value;
  assert(idxContact * (numBasis + 2) == b.size());
  z = Eigen::VectorXd::Zero(b.rows());

  for (int mIdxCnstrnt = 0; mIdxCnstrnt < idxContact; ++mIdxCnstrnt) {
    Eigen::MatrixXd newA = A;
    Eigen::VectorXd newb = b;
    if (mIdxCnstrnt > 0) {
      permuteAandBforTest(newA, newb, A, b, 0, mIdxCnstrnt, idxContact,
                          numBasis);
    }

    Eigen::VectorXd in_x = Aandb2input(newA, newb, idxContact);
    Eigen::VectorXd out_y;

    mCaffeClassifiers[idxContact - 1]->Eval(in_x, out_y);

    assert(out_y.size() == 1);
    value.push_back(static_cast<int>(out_y(0)));
  }

  LCPLS mlcpls(A, b, value);
  mlcpls.solve();
  z = mlcpls.getSolution();

  if (dart::lcpsolver::YT::validate(A, b, z)) {
    // pass
  } else {
    // std::cout << "Caffe + LP fails...." << std::endl;
    // std::cin.get();
  }
}

Eigen::VectorXd CaffeLPSolver::Aandb2input(const Eigen::MatrixXd& A,
                                           const Eigen::VectorXd& b,
                                           int idxContact) {
  assert(A.rows() == A.cols());
  assert(A.rows() == b.rows());
  assert(A.rows() == idxContact * (numBasis + 2));

  int nSize = b.rows();
  int ASize = idxContact * (numBasis + 1);
  int inputSize = ASize + (ASize + 1) * ASize / 2 + idxContact;

  std::vector<double> inputVector;
  for (int i = 0; i < nSize - idxContact; i++) {
    for (int j = i; j < nSize - idxContact; j++) {
      inputVector.push_back(A(i, j));
    }
  }

  for (int i = 0; i < idxContact; i++) {
    inputVector.push_back(A(nSize - idxContact + i, i));
  }

  for (int i = 0; i < nSize - idxContact; i++) {
    inputVector.push_back(b(i));
  }

  assert(inputSize == inputVector.size());

  Eigen::VectorXd input =
      Eigen::Map<Eigen::VectorXd>(inputVector.data(), inputSize);
  return input;
}
