#ifndef CNTCTLRNIN_MYWINDOW_H
#define CNTCTLRNIN_MYWINDOW_H

#include <cmath>
#include <random>
#include "LemkeLCPSolver.h"
#include "dart/dart.h"
#include "parameter.h"

namespace CntctLrnin {

class MyWindow : public dart::gui::SimWindow {
  //----------------------------------------------------------------------------
  // Member Function
  //----------------------------------------------------------------------------
  public:
  MyWindow(dart::simulation::WorldPtr world);
  virtual ~MyWindow();

  void timeStepping() override;
  void keyboard(unsigned char _key, int _x, int _y) override;
  void drawSkels() override;
  void displayTimer(int _val);
  void draw();
  void render();
  bool screenshot();

  void addExtForce();
  void addExtTorque();
  // void resetCubeOrientation(int idxmBox, int dir = 0);

  void restart();
  void movingCamera();
  void setPlatform();

  private:
  //----------------------------------------------------------------------------
  // Member Variable
  //----------------------------------------------------------------------------

  /// Collision Detector
  std::unique_ptr<dart::collision::CollisionDetector> mCollisionDetector;

  /// Counter for when to apply extForces
  int counter;

  /// Register for external forces and torques
  std::vector<Eigen::Vector3d> extForce;
  std::vector<Eigen::Vector3d> extTorque;
  int extForceDuration;
  int extTorqueDuration;

  /// Episode length
  int episodeLength;

  std::vector<Eigen::Vector3d> mColor;
};
}
#endif  // CNTCTLRNIN_MYWINDOW_H
