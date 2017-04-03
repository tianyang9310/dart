#include <chrono>
#include <ctime>
#include <iostream>

#include "DepthFirstSearchLCP.h"
#include "LemkeLCPSolver.h"
#include "MyWindow.h"
#include "MyWorld.h"
#include "SnoptWrapper.h"
#include "addSkeles.h"
#include "apps/lemkeFix/myLemke.h"
#include "csvParser.h"
#include "dart/dart.h"
#include "CaffeClassifier.h"

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
  // if (argc != 6) {
  //   std::cerr << "Usage: " << argv[0]
  //             << " deploy.prototxt network.caffemodel"
  //             << " mean.binaryproto labels.txt img.jpg" << std::endl;
  //   return 1;
  // }

  // ::google::InitGoogleLogging(argv[0]);

  string model_file   = "/Users/Yang/Material/Research/caffe/models/bvlc_reference_caffenet/deploy.prototxt";
  string trained_file = "/Users/Yang/Material/Research/caffe/models/bvlc_reference_caffenet/bvlc_reference_caffenet.caffemodel";
  string mean_file    = "/Users/Yang/Material/Research/caffe/data/ilsvrc12/imagenet_mean.binaryproto";
  string label_file   = "/Users/Yang/Material/Research/caffe/data/ilsvrc12/synset_words.txt";
  Classifier classifier(model_file, trained_file, mean_file, label_file);

  string file = "/Users/Yang/Material/Research/caffe/examples/images/cat.jpg";

  std::cout << "---------- Prediction for "
            << file << " ----------" << std::endl;

  cv::Mat img = cv::imread(file, -1);
  CHECK(!img.empty()) << "Unable to decode image " << file;
  std::vector<Prediction> predictions = classifier.Classify(img);

  /* Print the top N predictions. */
  for (size_t i = 0; i < predictions.size(); ++i) {
    Prediction p = predictions[i];
    std::cout << std::fixed << std::setprecision(4) << p.second << " - \""
              << p.first << "\"" << std::endl;
  }
  std::cin.get();
}

int main(int argc, char* argv[]) {
  std::srand(
      (unsigned)(std::chrono::system_clock::now().time_since_epoch().count()));

  std::shared_ptr<MyWorld> mWorld = std::make_shared<MyWorld>();
  assert(mWorld != nullptr);
  addSkel(mWorld);

  // testSnoptLP();
  // testSnoptLCP();
  // testDFSLCP();
  testCaffe();

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
