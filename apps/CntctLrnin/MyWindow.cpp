#include "MyWindow.h"

using namespace std;

namespace CntctLrnin {

MyWindow::MyWindow(dart::simulation::WorldPtr world) : SimWindow() {
  setWorld(world);
  mCollisionDetector = std::unique_ptr<dart::collision::CollisionDetector>(
      mWorld->getConstraintSolver()->getCollisionDetector());

  counter = 0;
  episodeLength = 6500;
  setPlatform();

  for (int i = 0; i < NUMBODYNODES; ++i) {
    extForce.push_back(Eigen::Vector3d::Zero());
    extTorque.push_back(Eigen::Vector3d::Zero());
  }
  extForceDuration = -1;
  extTorqueDuration = -1;

  /*
   * for (int i=0; i<NUMBODYNODES; i++){
   *   resetCubeOrientation(i,0);
   * }
   */
}

MyWindow::~MyWindow() {}

void MyWindow::timeStepping() {
  // --------------------------------------------------------------------------
  /*
   * for (int i = 0; i < NUMBODYNODES; i++) {
   *   std::cout << "================================================="
   *             << std::endl;
   *   std::cout << "mBox Position: "
   *             << mWorld->getSkeleton("mBox")->getPositions().transpose()
   *             << std::endl;
   *   std::cout << "mBox Velocity: "
   *             << mWorld->getSkeleton("mBox")->getVelocities().transpose()
   *             << std::endl
   *             << std::endl;
   * }
   */

  dart::gui::SimWindow::timeStepping();

#ifndef UNIT_TEST
  // 20 is the period/
  int mPeriod = PERIOD;
  int mDutyCycle = RANDOM_DURATION;
  counter = (counter + 1) % mPeriod;

  if (counter < mDutyCycle) {
    addExtForce();
  } else if (counter > mPeriod - mDutyCycle - 1) {
    addExtTorque();
  }
  restart();
#endif

#ifdef UNIT_TEST
#ifdef STRAIGHT_PUSH
  addExtForce();
  movingCamera();
  if (mWorld->getSimFrames() == episodeLength) {
    std::cout << "Time is " << mWorld->getTime() << std::endl;
    std::cout << mWorld->getSkeleton("mBox")->getPositions().transpose()
              << std::endl;
    std::cin.get();
  }
#endif

#endif
}

void MyWindow::addExtForce() {
  if (extForceDuration < 0) {
    extForceDuration = RANDOM_DURATION;
    for (int i = 0; i < NUMBODYNODES; i++) {
#ifndef UNIT_TEST
      extForce[i] = Eigen::Vector3d::Zero();
      int dir = (mWorld->getSimFrames() / 800) % 8;
      double mag = std::rand() % 7;
      double dev = double(std::rand()) / RAND_MAX * 10 - 5;
      extForce[i](0) = mag * std::sin((dir * 45.0 + dev) / 180 * DART_PI);
      extForce[i](2) = mag * std::cos((dir * 45.0 + dev) / 180 * DART_PI);
#endif

#ifdef UNIT_TEST
#ifdef STRAIGHT_PUSH
      extForce[i] = Eigen::Vector3d::Zero();
      extForce[i](0) = 15;
#endif
#endif
    }
  } else {
    extForceDuration--;
  }

  for (int i = 0; i < NUMBODYNODES; i++) {
    mWorld->getSkeleton("mBox")
        ->getBodyNode(idx2string(i))
        ->addExtForce(extForce[i]);
  }
}

void MyWindow::addExtTorque() {
  // dtmsg<<"Add external torque"<<std::endl;

  /// update extTorque every extTorqueDuration
  if (extTorqueDuration < 0) {
    extTorqueDuration = RANDOM_DURATION;
    for (int i = 0; i < NUMBODYNODES; ++i) {
      extTorque[i] = Eigen::Vector3d::Random() * 2.5;
    }
  } else {
    extTorqueDuration--;
  }

  for (int i = 0; i < NUMBODYNODES; i++) {
    mWorld->getSkeleton("mBox")
        ->getBodyNode(idx2string(i))
        ->addExtTorque(extTorque[i]);
  }
}

void MyWindow::keyboard(unsigned char _key, int _x, int _y) {
  switch (_key) {
    case 'e':
      movingCamera();
      break;
    default:
      dart::gui::SimWindow::keyboard(_key, _x, _y);
  }
}

void MyWindow::drawSkels() {
  glEnable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  /*
   * for (int i = 0; i<NUMBODYNODES; i++) {
   * // draw Ext force arrow
   * Eigen::Vector3d poa =
   * mWorld->getSkeleton("mBox")->getBodyNode(idx2string(i))->getCOM();
   * double len = extForce.norm() / 100.0;
   * // dart::gui::drawArrow3D(poa, extForce, len, 0.005, 0.01);
   * }
   */

  dart::gui::SimWindow::drawSkels();
}

void MyWindow::restart() {
  // restart if cubes are out of range
  double range = 500;
  for (int i = 0; i < NUMBODYNODES; i++) {
    if ((std::abs(mWorld->getSkeleton("mBox")
                      ->getBodyNode(idx2string(i))
                      ->getTransform()
                      .translation()(0)) > range)  // x direction
        || (std::abs(mWorld->getSkeleton("mBox")
                         ->getBodyNode(idx2string(i))
                         ->getTransform()
                         .translation()(2)) > range))  // z direction
    {
      // std::cout << "translation: " <<
      // mWorld->getSkeleton("mBox")->getBodyNode(idx2string(i))->getTransform().translation().transpose()
      // << std::endl;
      // std::cout << std::boolalpha;
      // std::cout << "translation x: " <<(std::abs(mWorld->getSkeleton("mBox")
      //                 ->getBodyNode(idx2string(i))
      //                 ->getTransform()
      //                 .translation()(0)) > range)<< std::endl;

      // std::cout << "translation z: " <<(std::abs(mWorld->getSkeleton("mBox")
      //                 ->getBodyNode(idx2string(i))
      //                 ->getTransform()
      //                 .translation()(2)) > range)<< std::endl;

      //    dterr << "ERROR: runnng out of range, need regularization!!!" <<
      // std::endl;

      // int dir = (mWorld->getSimFrames()/800)%8;
      // resetCubeOrientation(i,dir);
      int totalDof = mWorld->getSkeleton("mBox")->getNumDofs();
      mWorld->getSkeleton("mBox")->setPositions(
          Eigen::VectorXd::Zero(totalDof));
      mWorld->getSkeleton("mBox")->setVelocities(
          Eigen::VectorXd::Zero(totalDof));

      // std::cin.get();
    }
  }
}

/*
 * void MyWindow::resetCubeOrientation(int idxmBox, int dir) {
 *   std::cout << "reset orientation" << std::endl;
 *   Eigen::Vector3d eulerYZX;
 *   eulerYZX << dir*45.0, std::rand()%10-5+45, 0.0;
 *   eulerYZX = eulerYZX /180.0 * DART_PI;
 *   Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
 *   tf.linear() = dart::math::eulerYZXToMatrix(eulerYZX);
 *
 *   Eigen::VectorXd mResetPos(6);
 *   mResetPos  =
 *   dynamic_cast<dart::dynamics::FreeJoint*>(
 *     mWorld->getSkeleton(idx2string(idxmBox))->getJoint("Joint_1"))->convertToPositions(tf);
 *   mWorld->getSkeleton(idx2string(idxmBox))->setPositions(mResetPos);
 *   mWorld->getSkeleton(idx2string(idxmBox))->setVelocities(Eigen::VectorXd::Zero(6));
 * }
 */

void MyWindow::movingCamera() {
#ifdef UNIT_TEST
#ifdef STRAIGHT_PUSH
  dtmsg << "Update camear position..." << std::endl;
  //  std::cout << mTrans.transpose() << std::endl;
  mTrans = -mWorld->getSkeleton("mBox")->getPositions().segment(3, 3) * 1000;
//  mTrans[1] = 150.0f;
//  std::cout << mTrans.transpose() << std::endl;
#endif
#endif
}

void MyWindow::setPlatform() {
#ifdef UNIT_TEST
#ifdef STATIC_SLOPE
  double raw_angle = 45.0 / 180 * DART_PI;

  // tilt platform
  mWorld->getSkeleton("mPlatform")->getDof(0)->setPosition(raw_angle);

  // tilt cube
  mWorld->getSkeleton("mBox")->getDof(2)->setPosition(raw_angle);
  mWorld->getSkeleton("mBox")->getDof(4)->setPosition(
      0.055 / std::cos(raw_angle) - 0.055);

// keyboard('y',0,0);
// std::cin.get();
#endif
#endif
}
}
