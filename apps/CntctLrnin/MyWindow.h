#ifndef CNTCTLRNIN_MYWINDOW_H
#define CNTCTLRNIN_MYWINDOW_H

#include <random>
#include "Controller.h"
#include "MyDantzigLCPSolver.h"
#include "dart/dart.h"

//  #define ODE_VANILLA  // decide to use MyDantizig or Danzig

class MyWindow : public dart::gui::SimWindow {
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

  private:
  std::unique_ptr<Controller> mController;
  std::unique_ptr<dart::collision::CollisionDetector> mCollisionDetector;
  std::vector<Eigen::Vector3d> mColor;

  int counter;
  int episodeLength;
  Eigen::Vector3d extForce;
  bool alwaysUpdateViewer;
};

#endif  // CNTCTLRNIN_MYWINDOW_H
