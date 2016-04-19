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

//--------------------------------------------------------------------------
dart::collision::CollisionDetector* mGlobalDetector;
MyWindow* mGlobalWindow;
//--------------------------------------------------------------------------

int main(int argc, char* argv[])
{
  SkeletonPtr floor = createFloor();

  SkeletonPtr biped = loadBiped();
  
  WorldPtr world = std::make_shared<World>();
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  world->getConstraintSolver()->setCollisionDetector(new dart::collision::BulletCollisionDetector());
//--------------------------------------------------------------------------
  mGlobalDetector = world->getConstraintSolver()->getCollisionDetector();
//--------------------------------------------------------------------------

  world->addSkeleton(floor);
  world->addSkeleton(biped);

  MyWindow window(world);
  mGlobalWindow = &window;

  for (int i =0; i<biped->getNumDofs(); i++)
  {
  biped->getDof(i)->setPosition(window.mController->mCurrentStateMachine->mCurrentState->getDesiredJointPosition(i));
  }

  // -----------------------------------------------------------------------------------------
  std::cout<<"##################   getCOM() MAIN             #####################"<<std::endl;
  std::cout<<world->getSkeleton("biped")->getCOM().transpose()<<std::endl;
  std::cout<<world->getSkeleton("biped")->getBodyNode("h_foot_left")->getCOM().transpose()<<std::endl;
  std::cout<<"###################################################################"<<std::endl;
  // -----------------------------------------------------------------------------------------

  // Initialize glut, initialize the window, and begin the glut event loop
  glutInit(&argc, argv);
  window.initWindow(1068, 768, "Nonlinear Project");
  glutMainLoop();
}
