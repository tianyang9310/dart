#include <chrono>
#include <ctime>
#include <iostream>

#include "DepthFirstSearchLCP.h"
#include "LemkeLCPSolver.h"
#include "MyWindow.h"
#include "MyWorld.h"
#include "SnoptWrapper.h"
#include "addSkeles.h"
#include "csvParser.h"
#include "dart/dart.h"

#define runGUI

using namespace CntctLrnin;

void testSnoptLP() {
  Eigen::MatrixXd A(40, 40);
  Eigen::VectorXd b(40);
  A = load_csv<Eigen::MatrixXd>("testData/testSnoptLP_A.csv");
  b = load_csv<Eigen::MatrixXd>("testData/testSnoptLP_b.csv");
  std::cout << "Matrix A: " << std::endl << A << std::endl;
  std::cout << "Vector b: " << std::endl << b.transpose() << std::endl;

  SnoptWrapper mSnoptLPSolver(A, -b);
  Eigen::VectorXd z(40);
  mSnoptLPSolver.solveLP(z);

  std::cout << "Vector z:" << std::endl << z.transpose() << std::endl;
  std::cout << "A*z+b ? 0" << std::endl << (A * z + b).transpose() << std::endl;

  std::cin.get();
}

void testSnoptLCP() {
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  A = load_csv<Eigen::MatrixXd>("testData/testSnoptLCP_A.csv");
  b = load_csv<Eigen::MatrixXd>("testData/testSnoptLCP_b.csv");
  std::cout << "Matrix A: " << std::endl << A << std::endl;
  std::cout << "Vector b: " << std::endl << b.transpose() << std::endl;

  SnoptWrapper mSnoptLCPSolver(A,b);
  Eigen::VectorXd z(40);
  mSnoptLCPSolver.solveLCP(z);

  std::cout << "Vector z:" << std::endl << z.transpose() << std::endl;

  std::cin.get();
}

void testDFSLCP() {
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  A = load_csv<Eigen::MatrixXd>("testData/testSnoptLCP_A.csv");
  b = load_csv<Eigen::MatrixXd>("testData/testSnoptLCP_b.csv");
  std::cout << "Matrix A: " << std::endl << A << std::endl;
  std::cout << "Vector b: " << std::endl << b.transpose() << std::endl;

  Eigen::VectorXd z(40);
  std::vector<Eigen::VectorXd> ret_list;
  DFS(z, 0, A, b, ret_list);
  // DFS(z,0);
  for (size_t i = 0; i < ret_list.size(); i++) {
    std::cout << "ret_list: " << ret_list[i] << std::endl;
  }

  std::cin.get();
}

int main(int argc, char* argv[]) {
  // std::srand((unsigned)(std::chrono::system_clock::now().time_since_epoch().count()));

  std::shared_ptr<MyWorld> mWorld = std::make_shared<MyWorld>();
  assert(mWorld != nullptr);
  addSkel(mWorld);

  // testSnoptLP();
  // testSnoptLCP();
  // testDFSLCP();

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

#ifndef ODE_VANILLA
#ifndef FORK_LEMKE
#else
  // Using LemkeLCPSolver
  mWorld->getConstraintSolver()->setLCPSolver(
      new LemkeLCPSolver(mWorld->getTimeStep(), mWindow));
#endif
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
