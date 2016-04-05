/*************************************************************************
    > File Name: WorldSetup.cpp
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Tue Apr  5 10:11:01 2016
 ************************************************************************/

#include "WorldSetup.h"
namespace nonlinear{

SkeletonPtr createFloor()
{
  SkeletonPtr floor = Skeleton::create("floor");
  
  // Give the floor a body
  BodyNodePtr body =
      floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;
  
  // Give the body a shape
  double floor_width = 10.0;
  double floor_height = 0.01;
  std::shared_ptr<BoxShape> box(
      new BoxShape(Eigen::Vector3d(floor_width, floor_height, floor_width)));
  box->setColor(dart::Color::Gray(0.2));
  
  body->addVisualizationShape(box);
  body->addCollisionShape(box);
  
  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, -1.0, 0.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);
  
  return floor;
}

// Load a biped model and enable joint limits and self-collision
SkeletonPtr loadBiped()
{
  
  // Create the world with a skeleton
  WorldPtr world = SkelParser::readWorld(DART_DATA_PATH"skel/Nonlinear_Biped.skel");
  assert(world != nullptr);

  SkeletonPtr biped = world->getSkeleton("biped");

  // To make sure the bipedal robots act like human, 
  // 1. enforce joint limit
  // 2. enable self-collision
  for (size_t i=0; i<biped->getNumJoints(); ++i)
  {
	  biped->getJoint(i)->setPositionLimited(true);
  }
  
  // Enable self-collision detection in DART
  // By default DART doesn't check the self collision
  biped->enableSelfCollision();

  // set initial position of the robot
  biped->setPosition(biped->getDof("j_thigh_left_z")->getIndexInSkeleton(), 0.15);
  biped->setPosition(biped->getDof("j_thigh_right_z")->getIndexInSkeleton(), 0.15);
  biped->setPosition(biped->getDof("j_shin_left")->getIndexInSkeleton(), -0.4);
  biped->setPosition(biped->getDof("j_shin_right")->getIndexInSkeleton(), -0.4);

  return biped;
}

} // namespace nonlinear 
