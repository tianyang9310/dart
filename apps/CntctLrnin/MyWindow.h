#ifndef CNTCTLRNIN_MYWINDOW_H
#define CNTCTLRNIN_MYWINDOW_H

#include <random>
#include "Controller.h"
#include "MyDantzigLCPSolver.h"
#include "dart/dart.h"

// #define ODE_VANILLA  // decide to use MyDantizig or Danzig
#define FORK_LEMKE

#define NUMBASIS 8
#define PRECISION 20

// 0: cube
// 1: ball
// 2: cylinder
#define SHAPE 0

class MyWindow : public dart::gui::SimWindow {
  //----------------------------------------------------------------------------
  // Member Function
  //----------------------------------------------------------------------------
  public:
  MyWindow(dart::simulation::WorldPtr world);
  virtual ~MyWindow();

  void addExtForce();
  void addExtTorque();

  void timeStepping() override;
  void keyboard(unsigned char _key, int _x, int _y) override;
  void drawSkels() override;
  void draw() override;
  void displayTimer(int _val); 
  dart::simulation::WorldPtr getWorld();
  void updateViewer();
  void tiltPlatform();
  void setPlatform();

  private:
  std::unique_ptr<Controller> mController;
  std::unique_ptr<dart::collision::CollisionDetector> mCollisionDetector;
  std::vector<Eigen::Vector3d> mColor;

  //----------------------------------------------------------------------------
  // Member Variable
  //----------------------------------------------------------------------------
  /// counter for when to apply extForces
  int counter;

  /// counter for when to update random Forces
  int randFCounter;

  /// total length for episode
  int episodeLength;

  /// COM traj
  std::vector<Eigen::Vector3d> COMtraj;
  Eigen::Vector3d extForce;
  bool alwaysUpdateViewer;
};

#endif  // CNTCTLRNIN_MYWINDOW_H
