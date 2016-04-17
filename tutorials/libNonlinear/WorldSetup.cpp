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
  WorldPtr world = SkelParser::readWorld(DART_DATA_PATH"skel/Nonlinear_Biped_stick.skel");
  assert(world != nullptr);

  SkeletonPtr biped = world->getSkeleton("biped");

  // To make sure the bipedal robots act like human, 
  // 1. enforce joint limit
  // 2. enable self-collision
  for (size_t i=0; i<biped->getNumJoints(); ++i)
  {
	  biped->getJoint(i)->setPositionLimitEnforced(true);
  }
  
  // Enable self-collision detection in DART
  // By default DART doesn't check the self collision
  biped->enableSelfCollision();

  // set initial position of the robot
  biped->setPosition(biped->getDof("j_thigh_left")->getIndexInSkeleton(), 0.25);
  biped->setPosition(biped->getDof("j_thigh_right")->getIndexInSkeleton(), -0.15);
  biped->setPosition(biped->getDof("j_shin_left")->getIndexInSkeleton(), -0.4);
  biped->setPosition(biped->getDof("j_shin_right")->getIndexInSkeleton(), -0.4);

  WeldJoint::Properties properties_left;
  properties_left.mName=std::string("j_foot_left");
  WeldJoint::Properties properties_right;
  properties_right.mName=std::string("j_foot_right");
  
  BodyNodePtr leftFoot = biped->createJointAndBodyNodePair<WeldJoint>(biped->getBodyNode("h_shin_left"), properties_left, BodyNode::Properties(std::string("h_foot_left"))).second;
  BodyNodePtr rightFoot = biped->createJointAndBodyNodePair<WeldJoint>(biped->getBodyNode("h_shin_right"), properties_right, BodyNode::Properties(std::string("h_foot_right"))).second;
  std::shared_ptr<EllipsoidShape> leftsphere = std::make_shared<EllipsoidShape>(Eigen::Vector3d(0.007474, 0.007474, 0.007474));
  std::shared_ptr<EllipsoidShape> rightsphere = std::make_shared<EllipsoidShape>(Eigen::Vector3d(0.007474, 0.007474, 0.007474));
  leftFoot->addVisualizationShape(leftsphere);
  leftFoot->addCollisionShape(leftsphere);
  rightFoot->addVisualizationShape(rightsphere);
  rightFoot->addCollisionShape(rightsphere);

  Eigen::Isometry3d lefttf(Eigen::Isometry3d::Identity());
  Eigen::Isometry3d righttf(Eigen::Isometry3d::Identity());
  lefttf.translation() = Eigen::Vector3d(0, -0.3737, 0);
  righttf.translation() = Eigen::Vector3d(0, -0.3737, 0);
  leftFoot->getParentJoint()->setTransformFromParentBodyNode(lefttf);
  rightFoot->getParentJoint()->setTransformFromParentBodyNode(righttf);

  biped->getDof(2)->setName("j_pelvis_rot");
  for (size_t i=0; i<biped->getNumDofs();i++)
  {
	  std::cout<<biped->getDof(i)->getName()<<std::endl;
  }

  return biped;
}

} // namespace nonlinear 
