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

  mColor.clear();
  assert(NUMBODYNODES == 5);
  mColor.push_back(Eigen::Vector3d(0.9,0.1,0.1)); // red
  mColor.push_back(Eigen::Vector3d(0.1,0.9,0.1)); // green
  mColor.push_back(Eigen::Vector3d(0.1,0.1,0.9)); // blue
  mColor.push_back(Eigen::Vector3d(0.9,0.9,0.1)); // yellow
  mColor.push_back(Eigen::Vector3d(0.9,0.1,0.9)); // magenta

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
      double mag = std::rand() % 15;
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
      extTorque[i] = Eigen::Vector3d::Random() * 5;
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

void MyWindow::displayTimer(int _val) {
  // int numIter = 1;
  int numIter = mDisplayTimeout / (mWorld->getTimeStep() * 1000);
  if (mPlay) {
    mPlayFrame += 16;
    if (mPlayFrame >= mWorld->getRecording()->getNumFrames())
      mPlayFrame = 0;
  } else if (mSimulating) {
    for (int i = 0; i < numIter; i++) {
      timeStepping();
      mWorld->bake();
    }
  }
  glutPostRedisplay();
  glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

bool MyWindow::screenshot() {
  static int count = 0;
  char fileBase[32] = "frames/Capture";
  char fileName[64];
  // png
#ifdef _WIN32
  _snprintf(fileName, sizeof(fileName), "%s%.4d.png", fileBase, count++);
#else
  std::snprintf(fileName, sizeof(fileName), "%s%.4d.png", fileBase, count++);
#endif
  int tw = glutGet(GLUT_WINDOW_WIDTH);
  int th = glutGet(GLUT_WINDOW_HEIGHT);

  glReadPixels(0, 0,  tw, th, GL_RGBA, GL_UNSIGNED_BYTE, &mScreenshotTemp[0]);

  // reverse temp2 temp1
  for (int row = 0; row < th; row++) {
    memcpy(&mScreenshotTemp2[row * tw * 4],
           &mScreenshotTemp[(th - row - 1) * tw * 4], tw * 4);
  }

  unsigned result = lodepng::encode(fileName, mScreenshotTemp2, tw, th);

  // if there's an error, display it
  if (result) {
    std::cout << "lodepng error " << result << ": "
              << lodepng_error_text(result) << std::endl;
    return false;
  } else {
    std::cout << "wrote screenshot " << fileName << "\n";
    return true;
  }
}

void MyWindow::render() {
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(mPersp,
                 static_cast<double>(mWinWidth)/static_cast<double>(mWinHeight),
                 0.1, 10.0);
  gluLookAt(mEye[0], mEye[1], mEye[2], 0.0, 0.0, -1.0, mUp[0], mUp[1], mUp[2]);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  initGL();

  mTrackBall.applyGLRotation();

  // Draw world origin indicator
  if (!mCapture)
  {
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);
    glLineWidth(2.0);
    if (mRotate || mTranslate || mZooming) {
      glColor3f(1.0f, 0.0f, 0.0f);
      glBegin(GL_LINES);
      glVertex3f(-0.1f, 0.0f, -0.0f);
      glVertex3f(0.15f, 0.0f, -0.0f);
      glEnd();

      glColor3f(0.0f, 1.0f, 0.0f);
      glBegin(GL_LINES);
      glVertex3f(0.0f, -0.1f, 0.0f);
      glVertex3f(0.0f, 0.15f, 0.0f);
      glEnd();

      glColor3f(0.0f, 0.0f, 1.0f);
      glBegin(GL_LINES);
      glVertex3f(0.0f, 0.0f, -0.1f);
      glVertex3f(0.0f, 0.0f, 0.15f);
      glEnd();
    }
  }

  glScalef(mZoom, mZoom, mZoom);
  glTranslatef(mTrans[0]*0.001, mTrans[1]*0.001, mTrans[2]*0.001);

  initLights();
  draw();

  // Draw trackball indicator
  if (mRotate && !mCapture)
    mTrackBall.draw(mWinWidth, mWinHeight);

  glutSwapBuffers();

  if (mCapture)
    screenshot();
}

void MyWindow::draw() {
  glDisable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  if (!mSimulating) {
      if (mPlayFrame < mWorld->getRecording()->getNumFrames()) {
      size_t nSkels = mWorld->getNumSkeletons();
      for (size_t i = 0; i < nSkels; i++) {
        // size_t start = mWorld->getIndex(i);
        // size_t size = mWorld->getSkeleton(i)->getNumDofs();
        mWorld->getSkeleton(i)->setPositions(mWorld->getRecording()->getConfig(mPlayFrame, i));
      }
      if (mShowMarkers) {
        // size_t sumDofs = mWorld->getIndex(nSkels);
        int nContact = mWorld->getRecording()->getNumContacts(mPlayFrame);
        for (int i = 0; i < nContact; i++) {
            Eigen::Vector3d v = mWorld->getRecording()->getContactPoint(mPlayFrame, i);
            Eigen::Vector3d f = mWorld->getRecording()->getContactForce(mPlayFrame, i);

          glBegin(GL_LINES);
          glVertex3f(v[0], v[1], v[2]);
          glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
          glEnd();

          mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
          mRI->pushMatrix();
          glTranslated(v[0], v[1], v[2]);
          mRI->drawEllipsoid(Eigen::Vector3d(0.02, 0.02, 0.02));
          mRI->popMatrix();
        }
      }
    }
  } else {
    if (mShowMarkers) {
      collision::CollisionDetector* cd =
          mWorld->getConstraintSolver()->getCollisionDetector();
      for (size_t k = 0; k < cd->getNumContacts(); k++) {

        Eigen::Vector3d ctColor;
        if (cd->getContact(k).bodyNode1.lock()->getSkeleton()->getName() == "mBox") {
          ctColor = mColor[cd->getContact(k).bodyNode1.lock()->getIndexInSkeleton()];
        } else {
          ctColor = mColor[cd->getContact(k).bodyNode2.lock()->getIndexInSkeleton()];
        }

        mRI->setPenColor(ctColor);

        Eigen::Vector3d v = cd->getContact(k).point;
        Eigen::Vector3d f = cd->getContact(k).force / 10.0;
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
  gui::drawStringOnScreen(0.02f, 0.02f, frame);
  glEnable(GL_LIGHTING);
}

}
