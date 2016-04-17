/*
 * Nonlinear Project
 * Yang Tian
 * Email: tianyang9310@gmail.com
 * Date: Mar. 8th 2016
*/

#include <iostream>
#include "dart/dart.h"
#include "libNonlinear/WorldSetup.h"
#include "libNonlinear/MyWindow.h"
#include "libNonlinear/Controller.h"


using namespace dart::common;
using namespace dart::utils;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;
using namespace dart::gui;

using namespace nonlinear;

int main(int argc, char* argv[])
{
  SkeletonPtr floor = createFloor();

  SkeletonPtr biped = loadBiped();
  
  WorldPtr world = std::make_shared<World>();
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  world->getConstraintSolver()->setCollisionDetector(new dart::collision::BulletCollisionDetector());

  world->addSkeleton(floor);
  world->addSkeleton(biped);

  // -----------------------------------------------------------------------------------------
  std::cout<<"##################   getCOM() in the main file#####################"<<std::endl;
  std::cout<<world->getSkeleton("biped")->getCOM().matrix()<<std::endl;
  std::cout<<"###################################################################"<<std::endl;
  // -----------------------------------------------------------------------------------------
  
  MyWindow window(world);

  // Initialize glut, initialize the window, and begin the glut event loop
  glutInit(&argc, argv);
  window.initWindow(1068, 768, "Nonlinear Project");
  glutMainLoop();
}
