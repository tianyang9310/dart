#include "MyWindow.h"

using namespace std;

MyWindow::MyWindow(dart::simulation::WorldPtr world):SimWindow() {
  setWorld(world);
  mCollisionDetector = std::unique_ptr<dart::collision::CollisionDetector>(
      mWorld->getConstraintSolver()->getCollisionDetector());
  counter = 0;
  episodeLength = 1500;
  mColor.push_back(Eigen::Vector3d(0.8, 0.2, 0.2));  // red
  mColor.push_back(Eigen::Vector3d(0.2, 0.8, 0.2));  // green
  mColor.push_back(Eigen::Vector3d(0.2, 0.2, 0.8));  // blue
  mColor.push_back(Eigen::Vector3d(0.2, 0.2, 0.2));  // black
  srand(0);

  mZoom = 0.50f;
  mTrans[1] = 15.0f; 
  Eigen::Quaterniond initTrackBallQuat;
  initTrackBallQuat.w() = 0.895342;
  initTrackBallQuat.vec() = Eigen::Vector3d(0.180079,0.406869,0.0198165);
  mTrackBall.setQuaternion(initTrackBallQuat);
}

MyWindow::~MyWindow() {}

void MyWindow::timeStepping() {
  int numContacts = mCollisionDetector->getNumContacts();
/*
 *std::cout << "num of contact points is: " << numContacts << std::endl;
 *std::cout << std::endl;
 */
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
// std::cout << "Clear velocities lock"
//           << dynamic_cast<MyDantzigLCPSolver*>(
//                  mWorld->getConstraintSolver()->getLCPSolver())
//                  ->getSkeletonVelocitiesLock()
//                  .size()
//           << std::endl;
#endif

  /*
   *  // regularization the positive of cube
   *  double range = 12.5;
   *  if ((std::abs(mWorld->getSkeleton("mBox")
   *                    ->getBodyNode("BodyNode_1")
   *                    ->getTransform()
   *                    .translation()(0)) > range)  // x direction
   *      || (std::abs(mWorld->getSkeleton("mBox")
   *                       ->getBodyNode("BodyNode_1")
   *                       ->getTransform()
   *                       .translation()(2)) > range))  // z direction
   *  {
   *    dterr << "ERROR: runnng out of range, need regularization!!!" <<
   * std::endl;
   *
   *    mWorld->getSkeleton("mBox")->setPositions(Eigen::VectorXd::Zero(6));
   *    mWorld->getSkeleton("mBox")->setVelocities(Eigen::VectorXd::Zero(6));
   *  }
   */

  counter = (counter + 1) % 200;

  if (counter == 50) {
    // addExtForce();
  } else if (counter == 150) {
    // addExtTorque();
  }
  addExtForce();
  // addExtTorque();
  if (mWorld->getSimFrames() == episodeLength) {
    std::cout << "Time is " << mWorld->getTime() << std::endl;
    std::cout << mWorld->getSkeleton("mBox")->getPositions().transpose()
              << std::endl;
#ifndef ODE_VANILLA
    std::cout << "Lemke fail ratio: "
              << dynamic_cast<MyDantzigLCPSolver*>(
                     mWorld->getConstraintSolver()->getLCPSolver())
                         ->getLemkeFailCounter() /
                     double(episodeLength)
              << std::endl;
#endif
    keyboard('y', 0, 0);
  }
// mCollisionDetector->detectCollision(true, true);
#ifndef ODE_VANILLA
  numContacts = dynamic_cast<MyDantzigLCPSolver*>(
                    mWorld->getConstraintSolver()->getLCPSolver())
                    ->numContactsCallBack;
#else
  numContacts = dynamic_cast<DantzigLCPSolver*>(
                    mWorld->getConstraintSolver()->getLCPSolver())
                    ->numContactsCallBack;
#endif

  std::cout << "Current frame is " << mWorld->getSimFrames() << std::endl;
  if (numContacts != 4) {
    dterr << "numContacts: " << numContacts
          << " current frame: " << mWorld->getSimFrames() << std::endl;
    dterr << mWorld->getSkeleton("mBox")->getPositions().transpose()
          << std::endl;
    // keyboard('y', 0, 0);
  }
  /*
   *std::cout<<"num of contact points is: "<<numContacts<<std::endl;
   *std::cout<<std::endl;
   */
}

void MyWindow::addExtForce() {
  // dtmsg<<"Add external force"<<std::endl;
  // mWorld->getSkeleton("mBox")
  //     ->getBodyNode("BodyNode_1")
  //     ->addExtForce(Eigen::Vector3d::Random() * 3);

  // Eigen::Vector3d extForce = Eigen::Vector3d::Random() * 1e2;
  Eigen::Vector3d extForce = Eigen::Vector3d::Identity() * 1.2e1;
  extForce[1] = 0.0;
  extForce[2] = 0.0;
  mWorld->getSkeleton("mBox")->getBodyNode("BodyNode_1")->addExtForce(extForce);
}

void MyWindow::addExtTorque() {
  // dtmsg<<"Add external torque"<<std::endl;
  Eigen::Vector3d extTorque = Eigen::Vector3d::Identity() * 0.8;
  mWorld->getSkeleton("mBox")
      ->getBodyNode("BodyNode_1")
      ->addExtTorque(extTorque);

  // mWorld->getSkeleton("mBox")
  //     ->getBodyNode("BodyNode_1")
  //     ->addExtTorque(Eigen::Vector3d::Random() * 100);
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

dart::simulation::WorldPtr MyWindow::getWorld() { return mWorld; }
