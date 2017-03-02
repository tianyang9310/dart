#include <chrono>
#include <ctime>
#include <iostream>

#include "LCPLinEqu.h"
#include "LCPLPproblem.h"
#include "csvParser.h"
#include "My2DantzigLCPSolver.h"
#include "MyWindow.h"
#include "MyWorld.h"
#include "addSkeles.h"
#include "dart/dart.h"

#define RUN_GUI

using namespace CntctLrnin;

int main(int argc, char* argv[]) {
  std::srand(
      (unsigned)(std::chrono::system_clock::now().time_since_epoch().count()));
  // create and initialize the world
  std::shared_ptr<MyWorld> mWorld =
      std::make_shared<MyWorld>();
  assert(mWorld != nullptr);
  addSkel(mWorld);

/*
 *   // -----------------------------------------------------------------
 *   // test snopt LP
 *   Eigen::MatrixXd A(40,40);
 *   Eigen::VectorXd b(40);
 *   A = load_csv<Eigen::MatrixXd>("/tmp/A.csv");
 *   b = load_csv<Eigen::MatrixXd>("/tmp/b.csv");
 *   std::cout << "Matrix A: " << std::endl << A << std::endl;
 *   std::cout << "Vector b: " << std::endl << b.transpose() << std::endl;
 * 
 *   LCPLPproblem problem(40,40,A,-b);
 *   SnoptSolver solver(&problem);
 *   solver.solve();
 *   Eigen::VectorXd z(40);
 *   z.setZero();
 *   for (size_t i = 0; i < 40; i++) {
 *     z(i) = problem.vars()[i]->mVal;
 *   }
 *   std::cout << "Vector z:" << std::endl << z.transpose() << std::endl;
 *   std::cout << "A*z" << std::endl << (A*z+b).transpose() << std::endl;
 * 
 *   std::cin.get();
 */

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

  // create a window and link it to the world
  MyWindow window(mWorld);
  MyWindow* mWindow = &window;

#ifndef ODE_VANILLA
#ifndef FORK_LEMKE
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
