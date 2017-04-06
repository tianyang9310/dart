#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>

#include "CaffeClassifier.h"
#include "DepthFirstSearchLCP.h"
#include "JsonUtil.h"
#include "LCPLS.h"
#include "LemkeLCPSolver.h"
#include "MyWindow.h"
#include "MyWorld.h"
#include "SnoptWrapper.h"
#include "addSkeles.h"
#include "apps/lemkeFix/myLemke.h"
#include "csvParser.h"
#include "dart/dart.h"
#include "dist-json/json/json.h"
#include "CaffeLPSolver.h"
#include <glog/logging.h>
#include <ctime>

#define runGUI

using namespace CntctLrnin;

void testSnoptLP() {
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  A = load_csv<Eigen::MatrixXd>(DART_ROOT_PATH
                                "apps/CntctLrnin/testData/testSnoptLP_A.csv");
  b = load_csv<Eigen::MatrixXd>(DART_ROOT_PATH
                                "apps/CntctLrnin/testData/testSnoptLP_b.csv");
  std::cout << "Matrix A: " << std::endl << A << std::endl;
  std::cout << "Vector b: " << std::endl << b.transpose() << std::endl;

  SnoptWrapper mSnoptLPSolver(A, -b);
  Eigen::VectorXd z;
  mSnoptLPSolver.solveLP(z);

  std::cout << "Vector z:" << std::endl << z.transpose() << std::endl;
  std::cout << "A*z+b ? 0" << std::endl << (A * z + b).transpose() << std::endl;

  std::cout << "Validation: " << std::boolalpha
            << ((A * z + b).array().abs() < 1e-6).any() << std::endl;

  std::cin.get();
}

void testSnoptLCP() {
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  A = load_csv<Eigen::MatrixXd>(DART_ROOT_PATH
                                "apps/CntctLrnin/testData/testSnoptLCP_A.csv");
  b = load_csv<Eigen::MatrixXd>(DART_ROOT_PATH
                                "apps/CntctLrnin/testData/testSnoptLCP_b.csv");
  std::cout << "Matrix A: " << std::endl << A << std::endl;
  std::cout << "Vector b: " << std::endl << b.transpose() << std::endl;

  SnoptWrapper mSnoptLCPSolver(A, b);
  Eigen::VectorXd z;
  mSnoptLCPSolver.solveLCP(z);

  std::cout << "Vector z:" << std::endl << z.transpose() << std::endl;
  std::cout << "Validation: " << std::boolalpha
            << dart::lcpsolver::YT::validate(A, b, z);

  std::cin.get();
}

void testDFSLCP() {
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  A = load_csv<Eigen::MatrixXd>(
      DART_ROOT_PATH "apps/CntctLrnin/testData/testSnoptLCPDFS_A.csv");
  b = load_csv<Eigen::MatrixXd>(
      DART_ROOT_PATH "apps/CntctLrnin/testData/testSnoptLCPDFS_b.csv");
  std::cout << "Matrix A: " << std::endl << A << std::endl;
  std::cout << "Vector b: " << std::endl << b.transpose() << std::endl;

  Eigen::VectorXd z(6);
  std::vector<Eigen::VectorXd> ret_list;
  DFS(z, 0, A, b, ret_list);
  for (size_t i = 0; i < ret_list.size(); i++) {
    std::cout << "ret_list: " << std::endl
              << ret_list[i].transpose() << std::endl;
  }
  z = ret_list[0];
  std::cout << "Vector z:" << std::endl << z.transpose() << std::endl;
  std::cout << "Validation: " << std::boolalpha
            << dart::lcpsolver::YT::validate(A, b, z);
  std::cin.get();
}

void testCaffe() {
  std::string model_file = DART_ROOT_PATH
      "apps/CntctLrnin/CaffeNet/numContactToLearn_1/deploy.prototxt";
  std::string trained_file = DART_ROOT_PATH
      "apps/CntctLrnin/CaffeNet/numContactToLearn_1/train_iter_100000.caffemodel";

  std::string preprocessing_file = DART_ROOT_PATH
      "apps/CntctLrnin/CaffeNet/numContactToLearn_1/PreprocessingData.json";
  Classifier classifier(model_file, trained_file, preprocessing_file);

  Eigen::MatrixXd in_x_mat;
  in_x_mat = load_csv<Eigen::MatrixXd>(
      DART_ROOT_PATH "apps/CntctLrnin/testData/testCaffe.csv_");
  Eigen::VectorXd out_y;
  std::cout << "Ground Truth: " << std::endl
            << "8  8  8  8  2  7  7  3  4  1  3  3  6" << std::endl;
  for (int i = 0; i < 13; ++i) {
    classifier.Eval(in_x_mat.row(i).transpose(), out_y);
    std::cout << out_y.transpose() << " ";
  }

  std::cin.get();
}

void loadct(const string& ctJson, std::vector<Eigen::MatrixXd>& A,
            std::vector<Eigen::MatrixXd>& b,
            std::vector<Eigen::MatrixXd>& value) {
  std::ifstream f_stream(ctJson);
  Json::Reader reader;
  Json::Value root;
  bool succ = reader.parse(f_stream, root);
  f_stream.close();

  A.clear();
  b.clear();
  value.clear();

  for (int i = 0; i < 12; ++i) {
    Eigen::MatrixXd eachA;
    Eigen::MatrixXd eachb;
    Eigen::MatrixXd eachValue;
    succ &= cJsonUtil::ReadMatrixJson(root[i][0], eachA);
    succ &= cJsonUtil::ReadMatrixJson(root[i][1], eachb);
    succ &= cJsonUtil::ReadMatrixJson(root[i][2], eachValue);
    if (!succ) {
      std::cout << "Error occurs when reading json..." << std::endl;
      break;
    }
    A.push_back(eachA);
    b.push_back(eachb);
    value.push_back(eachValue);
  }
  std::cout << "Finish loading ct data..." << std::endl;
}

void testLCPLS(int whichIdx = -1) {
  // load ct from Json
  string ctJson = DART_ROOT_PATH "apps/CntctLrnin/testData/ct.json";
  std::vector<Eigen::MatrixXd> A;
  std::vector<Eigen::MatrixXd> b;
  std::vector<Eigen::MatrixXd> value;
  loadct(ctJson, A, b, value);

  int MaxIter = 10000;
  int count = 0;

  time_t tstart, tend;
  tstart = time(0);
  for (int iter = 0; iter < MaxIter; ++iter) {
    int idxContact = whichIdx == -1 ? rand()%12: whichIdx;

    int poolSize = b[idxContact].rows();  

    int idxChoice = rand()%poolSize;  // choose from poolSize

    int matSize = b[idxContact].cols();  
    assert(static_cast<int>(matSize / 6) == idxContact + 1);

    Eigen::VectorXd eachValueEigen =
        value[idxContact].row(idxChoice).transpose();
    std::vector<int> eachValue;
    eachValue.clear();
    for (int i = 0; i < idxContact + 1; ++i) {
      eachValue.push_back(eachValueEigen[i]);
    }
    Eigen::MatrixXd eachA =
        A[idxContact].block(idxChoice * matSize, 0, matSize, matSize);
    Eigen::VectorXd eachb = b[idxContact].row(idxChoice).transpose();

    // std::cout << "Matrix A: " << std::endl << eachA << std::endl;
    // std::cout << "Vector b: " << std::endl << eachb.transpose() << std::endl;
    // sstd::cout << "Value: " << std::endl << eachValueEigen.transpose() <<
    // std::endl;

    LCPLS mlcpls(eachA, eachb, eachValue);
    mlcpls.solve();
    Eigen::VectorXd eachZ = mlcpls.getSolution();

    // std::cout << "Vector z:" << std::endl << eachZ.transpose() << std::endl;
    // std::cout << "Validation: " << std::boolalpha
    //          << dart::lcpsolver::YT::validate(eachA, eachb, eachZ);

    if (dart::lcpsolver::YT::validate(eachA, eachb, eachZ)) {
      count++;
    } else {
      // std::cout << "idxChoice is " << idxChoice << std::endl;
      // Eigen::IOFormat CSVFmt(Eigen::FullPrecision, Eigen::DontAlignCols, ",\t");
      // std::cout << std::setprecision(20);
      // std::cout << "idxContact: " << idxContact + 1 << " idxChoice: " << idxChoice + 1 << std::endl;
      // std::cout << "Matrix A: " << std::endl << eachA.format(CSVFmt) << std::endl;
      // std::cout << "Vector b: " << std::endl << eachb.transpose().format(CSVFmt) << std::endl;
      // std::cout << "Value: " << std::endl << eachValueEigen.transpose() << std::endl;
      // std::cout << "Vector z:" << std::endl << eachZ.transpose() << std::endl;
      // std::cin.get();
    }
  }
  // Can acheive 99%+ solved ratio. Some not solvable due to incorrect value which may dates
  // back to different implementation of cpp and matlab and machine error epsilon
  std::cout << "Solved Ratio: " << double(count)/MaxIter << std::endl;
  tend = time(0); 
  std::cout << "Average Time: " << difftime(tend, tstart) / MaxIter << " seconds. " << std::endl;
  // std::cin.get();
}

void testLemke(int whichIdx = -1) {
  // load ct from Json
  string ctJson = DART_ROOT_PATH "apps/CntctLrnin/testData/ct.json";
  std::vector<Eigen::MatrixXd> A;
  std::vector<Eigen::MatrixXd> b;
  std::vector<Eigen::MatrixXd> value;
  loadct(ctJson, A, b, value);

  int MaxIter = 10000;
  int count = 0;

  time_t tstart, tend;
  tstart = time(0);
  for (int iter = 0; iter < MaxIter; ++iter) {
    int idxContact = whichIdx == -1 ? rand()%12: whichIdx;

    int poolSize = b[idxContact].rows();  

    int idxChoice = rand()%poolSize;  // choose from poolSize

    int matSize = b[idxContact].cols();  
    assert(static_cast<int>(matSize / 6) == idxContact + 1);

    Eigen::VectorXd eachValueEigen =
        value[idxContact].row(idxChoice).transpose();
    std::vector<int> eachValue;
    eachValue.clear();
    for (int i = 0; i < idxContact + 1; ++i) {
      eachValue.push_back(eachValueEigen[i]);
    }
    Eigen::MatrixXd eachA =
        A[idxContact].block(idxChoice * matSize, 0, matSize, matSize);
    Eigen::VectorXd eachb = b[idxContact].row(idxChoice).transpose();

    // std::cout << "Matrix A: " << std::endl << eachA << std::endl;
    // std::cout << "Vector b: " << std::endl << eachb.transpose() << std::endl;
    // sstd::cout << "Value: " << std::endl << eachValueEigen.transpose() <<
    // std::endl;

    Eigen::VectorXd* eachZ = new Eigen::VectorXd(matSize);
    int err = dart::lcpsolver::YT::Lemke(eachA, eachb, eachZ);

    // std::cout << "Vector z:" << std::endl << eachZ.transpose() << std::endl;
    // std::cout << "Validation: " << std::boolalpha
    //          << dart::lcpsolver::YT::validate(eachA, eachb, eachZ);

    if (dart::lcpsolver::YT::validate(eachA, eachb, (*eachZ))) {
      // std::cout << "Validation: True" << std::endl;
      count++;
    } else {
      // std::cout << "Validation: False" << std::endl;
      // Eigen::IOFormat CSVFmt(Eigen::FullPrecision, Eigen::DontAlignCols, ",\t");
      // std::cout << std::setprecision(20);
      // std::cout << "idxContact: " << idxContact + 1 << " idxChoice: " << idxChoice + 1 << std::endl;
      // std::cout << "Matrix A: " << std::endl << eachA.format(CSVFmt) << std::endl;
      // std::cout << "Vector b: " << std::endl << eachb.transpose().format(CSVFmt) << std::endl;
      // std::cout << "Value: " << std::endl << eachValueEigen.transpose() << std::endl;
      // std::cout << "Vector z:" << std::endl << eachZ.transpose() << std::endl;
      // std::cin.get();
    }
  }
  // Can acheive 99%+ solved ratio. Some not solvable due to incorrect value which may dates
  // back to different implementation of cpp and matlab and machine error epsilon
  std::cout << "Solved Ratio: " << double(count)/MaxIter << std::endl;
  tend = time(0); 
  std::cout << "Average Time: " << difftime(tend, tstart) / MaxIter << " seconds. " << std::endl;
  // std::cin.get();
}

void testCaffeLPSolver() {
  CaffeLPSolver caffelpsolver(1);
}

void compareLemkevsLP() {
  // solve time between Lemke and LP
  for (int i = 0; i < 12; ++i)
  {
    dtmsg << "For " << i + 1 << " contact points: " << std::endl;
    testLCPLS(i);
    testLemke(i);
  }
  std::cin.get();
}

int main(int argc, char* argv[]) {
  std::srand(
      (unsigned)(std::chrono::system_clock::now().time_since_epoch().count()));
  
  FLAGS_log_dir = DART_ROOT_PATH"/build/log";
  google::InitGoogleLogging(argv[0]);

  std::shared_ptr<MyWorld> mWorld = std::make_shared<MyWorld>();
  assert(mWorld != nullptr);
  addSkel(mWorld);

  // testSnoptLP();
  // testSnoptLCP();
  // testDFSLCP();
  // testCaffe();
  // std::cout << "Specifically for ct 12..." << std::endl;
  // testLCPLS(11);
  // testCaffeLPSolver();
  compareLemkevsLP();

  Eigen::Vector3d gravity(0.0, -9.81, 0.0);
  mWorld->setGravity(gravity);

  // using Bullet Collision Detector
  mWorld->getConstraintSolver()->setCollisionDetector(
      // new dart::collision::BulletCollisionDetector());
      // new dart::collision::FCLMeshCollisionDetector());
      new dart::collision::DARTCollisionDetector());

  // create a window and link it to the world
  MyWindow window(mWorld);
  MyWindow* mWindow = &window;

#ifdef LEMKE_SOLVER
  // Using LemkeLCPSolver
  mWorld->getConstraintSolver()->setLCPSolver(
      new LemkeLCPSolver(mWorld->getTimeStep(), mWindow));
  if (NUMBASIS != 2) {
    dtmsg << "Using LemkeLCPSolver..." << std::endl;
  }
#endif

#ifdef runGUI
  glutInit(&argc, argv);
  window.initWindow(640, 480, "Balance");
  glutMainLoop();
#else
  while (true) {
    window.timeStepping();
  }
#endif

  return 0;
}
