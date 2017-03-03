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
  void draw() override;
  void displayTimer(int _val);

  void addExtForce();
  void addExtTorque();
  // void resetCubeOrientation(int idxmBox, int dir = 0);

  void movingCamera();

  private:
  //----------------------------------------------------------------------------
  // Member Variable
  //----------------------------------------------------------------------------

  /// Collision Detector
  std::unique_ptr<dart::collision::CollisionDetector> mCollisionDetector;

  /// Different color for different contact poitns
  std::vector<Eigen::Vector3d> mColor;

  /// Counter for when to apply extForces
  int counter;

  /// Counter for when to update random Forces
  int randFCounter;

  /// Episode length
  int episodeLength;

  /// Force application point offset
  Eigen::Vector3d offset;

  /// Register for external forces
  Eigen::Vector3d extForce;
};
}
#endif  // CNTCTLRNIN_MYWINDOW_H
