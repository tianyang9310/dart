#include "MyWindow.h"

using namespace std;

namespace CntctLrnin {

MyWindow::MyWindow(dart::simulation::WorldPtr world) : SimWindow() {
  setWorld(world);
  mCollisionDetector = std::unique_ptr<dart::collision::CollisionDetector>(
      mWorld->getConstraintSolver()->getCollisionDetector());
  counter = 0;
  randFCounter = 50;
  episodeLength = 6500;
  mColor.push_back(Eigen::Vector3d(0.8, 0.2, 0.2));  // red
  mColor.push_back(Eigen::Vector3d(0.2, 0.8, 0.2));  // green
  mColor.push_back(Eigen::Vector3d(0.2, 0.2, 0.8));  // blue
  mColor.push_back(Eigen::Vector3d(0.2, 0.2, 0.2));  // black
  srand(0);

  mZoom = 0.50f;
  mTrans[1] = 15.0f;
  Eigen::Quaterniond initTrackBallQuat;
  initTrackBallQuat.w() = 0.895342;
  initTrackBallQuat.vec() = Eigen::Vector3d(0.180079, 0.406869, 0.0198165);
  mTrackBall.setQuaternion(initTrackBallQuat);

  extForce.setZero();
  /*
   * for (int i=0; i<NUMBODYNODES; i++){
   *   resetCubeOrientation(i,0);
   * }
   */

  offset << -0.05, 0.05, 0;
#ifdef LEMKE_SOLVER
    dtmsg << "Using Matrix A and Vector b from ODE and Lemke to solve LCP"
          << std::endl;
#else
    dtmsg << "Using ODE to solve LCP" << std::endl;
#endif
}

MyWindow::~MyWindow() {}

void MyWindow::timeStepping() {
  // addExtForce();
  int numContacts = mCollisionDetector->getNumContacts();
// for (int i=0; i<NUMBODYNODES; i++){
// std::cout << "=================================================" <<
// std::endl;
// std::cout << "mBox Position: "
//           << mWorld->getSkeleton("mBox")->getPositions().transpose()
//           << std::endl;
// std::cout << "mBox Velocity: "
//           << mWorld->getSkeleton("mBox")->getVelocities().transpose()
//           << std::endl
//           << std::endl;
// }

/*
 *std::cout << "num of contact points is: " << numContacts << std::endl;
 *std::cout << std::endl;
 */

  dart::gui::SimWindow::timeStepping();

  // regularization the positive of cube
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
      //    dterr << "ERROR: runnng out of range, need regularization!!!" <<
      // std::endl;

      // int dir = (mWorld->getSimFrames()/800)%8;
      // resetCubeOrientation(i,dir);
      int totalDof = mWorld->getSkeleton("mBox")->getNumDofs();
      mWorld->getSkeleton("mBox")->setPositions(
          Eigen::VectorXd::Zero(totalDof));
      mWorld->getSkeleton("mBox")->setVelocities(
          Eigen::VectorXd::Zero(totalDof));
    }
  }

  // 20 is the period/
  int mPeriod = 10;
  int mDutyCycle = 3;
  counter = (counter + 1) % mPeriod;

  if (counter < mDutyCycle) {
    addExtForce();
    // assert(NUMBODYNODES==1);
  } else if (counter > mPeriod - mDutyCycle - 1) {
    addExtTorque();
  }
  // addExtTorque();
  if (mWorld->getSimFrames() == episodeLength) {
    std::cout << "Time is " << mWorld->getTime() << std::endl;
    std::cout << mWorld->getSkeleton("mBox")->getPositions().transpose()
              << std::endl;
#ifdef LEMKE_SOLVER
    std::cout << "Lemke fail ratio: "
              << dynamic_cast<LemkeLCPSolver*>(
                     mWorld->getConstraintSolver()->getLCPSolver())
                         ->getLCPFail() /
                     double(episodeLength)
              << std::endl;
#endif
    // keyboard('y', 0, 0);
  }
// mCollisionDetector->detectCollision(true, true);

  // keyboard('y', 0, 0);
}

void MyWindow::addExtForce() {
  // ---------------------------------------------------------------------------
  for (int i = 0; i < NUMBODYNODES; i++) {
    // Apply force to COM
    // add constant external forces
    extForce.setZero();
    int dir = (mWorld->getSimFrames() / 800) % 8;
    double mag = std::rand() % 15;
    double dev = double(std::rand()) / RAND_MAX * 10 - 5;
    extForce(0) = mag * std::sin((dir * 45.0 + dev) / 180 * DART_PI);
    extForce(2) = mag * std::cos((dir * 45.0 + dev) / 180 * DART_PI);

    // randFCounter--;
    // if (randFCounter < 0) {
    // add random external forces
    // extForce = Eigen::Vector3d::Random() * 5;
    //   randFCounter = 50;
    // }
    // dterr << "Add external force: " << extForce.transpose() << std::endl;

    // Apply force to COM
    mWorld->getSkeleton("mBox")
        ->getBodyNode(idx2string(i))
        ->addExtForce(extForce);
  }
  // ---------------------------------------------------------------------------

  // // Apply force to margin
  // double theta = -mWorld->getSkeleton("mBox")->getDof(2)->getPosition();
  // while (theta < 0) {
  //   theta += 2*DART_PI;
  // }
  // theta = DART_PI_HALF / 2.0 -theta;

  // extForce.setZero();
  // extForce(0) = -(mWorld->getSkeleton("mBox")->getMass() *
  // mWorld->getGravity()
  //   * std::tan(theta) / 2)(1);
  // extForce(0) = extForce(0) * 3.0;

  // std::cout << "Theta: " << theta << std::endl;
  // if (theta>0) { // && mWorld->getSimFrames() < 250){
  //   mWorld->getSkeleton("mBox")->getBodyNode("BodyNode_1")->addExtForce(extForce,
  //   offset);
  // } else {
  //   std::cout << "Psi: " <<
  //   -mWorld->getSkeleton("mBox")->getDof(2)->getPosition() << std::endl;
  //   if (-mWorld->getSkeleton("mBox")->getDof(2)->getPosition()>=3.13) {
  //     // std::cin.get();
  //     // keyboard('y',0,0);
  //   }
  // }
}

void MyWindow::addExtTorque() {
  // dtmsg<<"Add external torque"<<std::endl;
  // Eigen::Vector3d extTorque = Eigen::Vector3d::Identity() * 0.8;
  // mWorld->getSkeleton("mBox")
  //     ->getBodyNode("BodyNode_1")
  //     ->addExtTorque(extTorque);

  for (int i = 0; i < NUMBODYNODES; i++) {
    mWorld->getSkeleton("mBox")
        ->getBodyNode(idx2string(i))
        ->addExtTorque(Eigen::Vector3d::Random() * 5);
  }
}

void MyWindow::keyboard(unsigned char _key, int _x, int _y) {
  switch (_key) {
    case 'y':
      mSimulating = !mSimulating;
      break;
    case 'o':
      mPlay = !mPlay;
      break;
    case 'a':
      addExtForce();
      break;
    case 'b':
      addExtTorque();
      break;
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
      break;
    case 'e':
      break;
    default:
      dart::gui::SimWindow::keyboard(_key, _x, _y);
  }
}

void MyWindow::drawSkels() {
  // Make sure lighting is turned on and that polygons get filled in
  glEnable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  /*
   * for (int i = 0; i<NUMBODYNODES; i++) {
   * // draw Ext force arrow
   * Eigen::Vector3d poa = mWorld->getSkeleton(idx2string(i))->getCOM();
   * double len = extForce.norm() / 100.0;
   * // dart::gui::drawArrow3D(poa, extForce, len, 0.005, 0.01);
   * }
   */

  dart::gui::SimWindow::drawSkels();
}

void MyWindow::draw() {
  glDisable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  if (!mSimulating) {
    if (mPlay) {
      if (mPlayFrame < mWorld->getRecording()->getNumFrames()) {
        size_t nSkels = mWorld->getNumSkeletons();
        for (size_t i = 0; i < nSkels; i++) {
          // size_t start = mWorld->getIndex(i);
          // size_t size = mWorld->getSkeleton(i)->getNumDofs();
          mWorld->getSkeleton(i)->setPositions(
              mWorld->getRecording()->getConfig(mPlayFrame, i));
        }
        if (mShowMarkers) {
          // size_t sumDofs = mWorld->getIndex(nSkels);
          int nContact = mWorld->getRecording()->getNumContacts(mPlayFrame);
          for (int i = 0; i < nContact; i++) {
            Eigen::Vector3d v =
                mWorld->getRecording()->getContactPoint(mPlayFrame, i);
            Eigen::Vector3d f =
                mWorld->getRecording()->getContactForce(mPlayFrame, i) / 10.0;

            mRI->setPenColor(mColor[i]);
            glBegin(GL_LINES);
            glVertex3f(v[0], v[1], v[2]);
            glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
            glEnd();
            mRI->pushMatrix();
            glTranslated(v[0], v[1], v[2]);
            mRI->drawEllipsoid(Eigen::Vector3d(0.02, 0.02, 0.02));
            mRI->popMatrix();
          }
        }
      }
    } else {
      if (mShowMarkers) {
        /*
         *dart::collision::CollisionDetector* cd =
         *    mWorld->getConstraintSolver()->getCollisionDetector();
         */
        for (size_t k = 0; k < mCollisionDetector->getNumContacts(); k++) {
          Eigen::Vector3d v = mCollisionDetector->getContact(k).point;
          Eigen::Vector3d f = mCollisionDetector->getContact(k).force / 10.0;
          mRI->setPenColor(mColor[k]);
          glBegin(GL_LINES);
          glVertex3f(v[0], v[1], v[2]);
          glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
          glEnd();
          mRI->pushMatrix();
          glTranslated(v[0], v[1], v[2]);
          mRI->drawEllipsoid(Eigen::Vector3d(0.02, 0.02, 0.02));
          mRI->popMatrix();
        }
      }
    }
  } else {
    if (mShowMarkers) {
      for (size_t k = 0; k < mCollisionDetector->getNumContacts(); k++) {
        Eigen::Vector3d v = mCollisionDetector->getContact(k).point;
        Eigen::Vector3d f = mCollisionDetector->getContact(k).force / 10.0;
        mRI->setPenColor(mColor[k]);
        glBegin(GL_LINES);
        glVertex3f(v[0], v[1], v[2]);
        glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
        glEnd();
        mRI->pushMatrix();
        glTranslated(v[0], v[1], v[2]);
        mRI->drawEllipsoid(Eigen::Vector3d(0.02, 0.02, 0.02));
        mRI->popMatrix();
      }
    }
  }
  drawSkels();
  drawEntities();

  // display the frame count in 2D text
  char buff[64];
  if (!mSimulating)
    if (!mPlay)
#ifdef _WIN32
      _snprintf(buff, sizeof(buff), "%d", mWorld->getSimFrames());
#else
      std::snprintf(buff, sizeof(buff), "%d", mWorld->getSimFrames());
#endif
    else
#ifdef _WIN32
      _snprintf(buff, sizeof(buff), "%d", mPlayFrame);
#else
      std::snprintf(buff, sizeof(buff), "%d", mPlayFrame);
#endif
  else
#ifdef _WIN32
    _snprintf(buff, sizeof(buff), "%d", mWorld->getSimFrames());
#else
    std::snprintf(buff, sizeof(buff), "%d", mWorld->getSimFrames());
#endif
  std::string frame(buff);
  glColor3f(0.0, 0.0, 0.0);
  dart::gui::drawStringOnScreen(0.02f, 0.02f, frame);
  glEnable(GL_LIGHTING);
}

void MyWindow::displayTimer(int _val) {
  int numIter = mDisplayTimeout / (mWorld->getTimeStep() * 1000);
  if (mPlay) {
    mPlayFrame += 1;
    if (mPlayFrame >= mWorld->getRecording()->getNumFrames()) {
      mPlayFrame = 0;
      mPlay = false;
    }
  } else if (mSimulating) {
    for (int i = 0; i < numIter; i++) {
      timeStepping();
      mWorld->bake();
      if (!mSimulating) {
        break;
      }
    }
  }

  /*
   *std::cout << "################################################" <<
   *std::endl;
   *if (mCollisionDetector->getNumContacts() > 0) {
   *  for (int idx_cnt = 0; idx_cnt < mCollisionDetector->getNumContacts();
   *       idx_cnt++) {
   *    std::cout << mCollisionDetector->getContact(idx_cnt).force.transpose()
   *              << std::endl;
   *  }
   *}
   *std::cout << "################################################" <<
   *std::endl;
   */

  glutPostRedisplay();
  glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
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
}
