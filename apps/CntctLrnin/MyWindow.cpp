#include "MyWindow.h"

using namespace std;

MyWindow::MyWindow(dart::simulation::WorldPtr world) {
  setWorld(world);
  mCollisionDetector = std::unique_ptr<dart::collision::CollisionDetector>(
      mWorld->getConstraintSolver()->getCollisionDetector());
  counter = 0;
}

MyWindow::~MyWindow() {}

void MyWindow::timeStepping() {

#ifndef ODE_VANILLA
  // lock Skeleton Velocities before stepping function
  dynamic_cast<MyDantzigLCPSolver*>(
      mWorld->getConstraintSolver()->getLCPSolver())
      ->pushVelocities(mWorld->getSkeleton("mBox"),
                       mWorld->getSkeleton("mBox")->getVelocities());
  dynamic_cast<MyDantzigLCPSolver*>(
      mWorld->getConstraintSolver()->getLCPSolver())
      ->pushVelocities(mWorld->getSkeleton("ground skeleton"),
                       mWorld->getSkeleton("ground skeleton")->getVelocities());
#endif

  dart::gui::SimWindow::timeStepping();

#ifndef ODE_VANILLA
  dynamic_cast<MyDantzigLCPSolver*>(
      mWorld->getConstraintSolver()->getLCPSolver())
      ->getSkeletonVelocitiesLock()
      .clear();
  // std::cout<<"Clear velocities lock
  // "<<dynamic_cast<MyDantzigLCPSolver*>(mWorld->getConstraintSolver()->getLCPSolver())->getSkeletonVelocitiesLock().size()<<std::endl;
#endif

  // regularization the positive of cube
  double range = 12.5;
  if ((std::abs(mWorld->getSkeleton("mBox")
                    ->getBodyNode("BodyNode_1")
                    ->getTransform()
                    .translation()(0)) > range)  // x direction
      || (std::abs(mWorld->getSkeleton("mBox")
                       ->getBodyNode("BodyNode_1")
                       ->getTransform()
                       .translation()(2)) > range))  // z direction
  {
    dterr << "ERROR: runnng out of range, need regularization!!!" << std::endl;

    mWorld->getSkeleton("mBox")->setPositions(Eigen::VectorXd::Zero(6));
    mWorld->getSkeleton("mBox")->setVelocities(Eigen::VectorXd::Zero(6));
  }

  counter = (counter + 1) % 200;

  if (counter == 50) {
    addExtForce();
  } else if (counter == 150) {
    addExtTorque();
  }
}

void MyWindow::addExtForce() {
  // dtmsg<<"Add external force"<<std::endl;
  // mWorld->getSkeleton("mBox")->getBodyNode("BodyNode_1")->addExtTorque(Eigen::Vector3d::Random()*100);

  Eigen::Vector3d extForce = Eigen::Vector3d::Random() * 5e2;
  extForce[1] = 0.0;
  mWorld->getSkeleton("mBox")->getBodyNode("BodyNode_1")->addExtForce(extForce);
  // mWorld->getSkeleton("mBox")->getBodyNode("BodyNode_1")->addExtForce(Eigen::Vector3d::Random()*3);
}

void MyWindow::addExtTorque() {
  // dtmsg<<"Add external torque"<<std::endl;
  mWorld->getSkeleton("mBox")
      ->getBodyNode("BodyNode_1")
      ->addExtTorque(Eigen::Vector3d::Random() * 100);
}

void MyWindow::keyboard(unsigned char _key, int _x, int _y) {
  switch (_key) {
    case 'y':
      mSimulating = !mSimulating;
      break;
    case 'a':
      addExtForce();
      break;
    case 'b':
      addExtTorque();
      break;
    default:
      dart::gui::SimWindow::keyboard(_key, _x, _y);
  }
}

void MyWindow::drawSkels() {
  // Make sure lighting is turned on and that polygons get filled in
  glEnable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  dart::gui::SimWindow::drawSkels();
}
