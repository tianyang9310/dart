#include <chrono>
#include <ctime>
#include <iostream>

#include "MyDantzigLCPSolver.h"
#include "My2DantzigLCPSolver.h"
#include "MyWindow.h"
#include "MyWorld.h"
#include "addSkeles.h"
#include "dart/dart.h"

int main(int argc, char* argv[]) {
  std::srand(
      (unsigned)(std::chrono::system_clock::now().time_since_epoch().count()));
  // create and initialize the world
  std::shared_ptr<dart::simulation::MyWorld> mWorld =
      std::make_shared<dart::simulation::MyWorld>();
  assert(mWorld != nullptr);
  AddSkel(mWorld);

  Eigen::Vector3d gravity(0.0, -9.81, 0.0);
  mWorld->setGravity(gravity);

  // using Bullet Collision Detector
  mWorld->getConstraintSolver()->setCollisionDetector(
      new dart::collision::BulletCollisionDetector());

  // using MyDantzigLCPSolver
  int totalDOF = 0;
  totalDOF += mWorld->getSkeleton("ground skeleton")->getNumDofs();
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
      new My2DantzigLCPSolver(mWorld->getTimeStep()));
#endif
#endif

  glutInit(&argc, argv);
  window.initWindow(640, 480, "Balance");
  glutMainLoop();

  return 0;
}
