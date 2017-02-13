#include <chrono>
#include <ctime>
#include <iostream>

#include "LCPLinEqu.h"
#include "My2DantzigLCPSolver.h"
#include "MyDantzigLCPSolver.h"
#include "MyWindow.h"
#include "MyWorld.h"
#include "addSkeles.h"
#include "dart/dart.h"

// #define RUN_GUI

int main(int argc, char* argv[]) {
  std::srand(
      (unsigned)(std::chrono::system_clock::now().time_since_epoch().count()));
  // create and initialize the world
  std::shared_ptr<dart::simulation::MyWorld> mWorld =
      std::make_shared<dart::simulation::MyWorld>();
  assert(mWorld != nullptr);
  AddSkel(mWorld);

  /*
   *   // -----------------------------------------------------------------
   *   // test
   *     Eigen::MatrixXd A;
   *     Eigen::VectorXd b;
   *
   *     A.resize(6,6);
   *     A <<
   *             3.1360,   -2.0370,   0.9723,   0.1096,  -2.0370,   0.9723,
   *            -2.0370,    3.7820,   0.8302,  -0.0257,   2.4730,   0.0105,
   *             0.9723,    0.8302,   5.1250,  -2.2390,  -1.9120,   3.4080,
   *             0.1096,   -0.0257,  -2.2390,   3.1010,  -0.0257,  -2.2390,
   *            -2.0370,    2.4730,  -1.9120,  -0.0257,   5.4870,  -0.0242,
   *             0.9723,    0.0105,   3.4080,  -2.2390,  -0.0242,   3.3860;
   *
   *     b.resize(6);
   *     b <<
   *             0.1649,
   *            -0.0025,
   *            -0.0904,
   *            -0.0093,
   *            -0.0000,
   *            -0.0889;
   *     // SnLCP mSnoptLCPSolver(A,b);
   *     Eigen::VectorXd z(6);
   *     z.setZero();
   *
   *     std::vector<Eigen::VectorXd> ret_list;
   *     DFS(z,0,A,b,ret_list);
   *     // DFS(z,0);
   *     for (size_t i = 0; i<ret_list.size(); i++) {
   *       std::cout << "ret_list: " << ret_list[i] << std::endl;
   *     }
   *
   *     // z << 0,1,0,1,1,1;
   *     // LCPLinEqu(A,b,z);
   *     // mSnoptLCPSolver.solve(z);
   *     std::cout << "z: " << z.transpose() << std::endl;
   *       // ---------------------------------------------------------------
   */

  Eigen::Vector3d gravity(0.0, -9.81, 0.0);
  mWorld->setGravity(gravity);

  // using Bullet Collision Detector
  mWorld->getConstraintSolver()->setCollisionDetector(
      // new dart::collision::BulletCollisionDetector());
      // new dart::collision::FCLMeshCollisionDetector());
      new dart::collision::DARTCollisionDetector());

  // using MyDantzigLCPSolver
  int totalDOF = 0;
  // totalDOF += mWorld->getSkeleton("ground skeleton")->getNumDofs();
  totalDOF += mWorld->getSkeleton("mBox")->getNumDofs();
  // std::cout << "Ground Skeleton: "
  // << mWorld->getSkeleton("ground skeleton")->isMobile() << std::endl;
  // std::cout << "mBox: " << mWorld->getSkeleton("mBox")->isMobile() <<
  // std::endl;

  // create a window and link it to the world
  MyWindow window(mWorld);
  MyWindow* mWindow = &window;

#ifndef ODE_VANILLA
#ifndef FORK_LEMKE
  // Using MyDantzigLCPSolver
  mWorld->getConstraintSolver()->setLCPSolver(
      new MyDantzigLCPSolver(mWorld->getTimeStep(), totalDOF, mWindow));
#else
  // Using My2DantzigLCPSolver
  mWorld->getConstraintSolver()->setLCPSolver(
      new My2DantzigLCPSolver(mWorld->getTimeStep(), mWindow));
#endif
#endif

#ifdef RUN_GUI
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
