#include "addSkeles.h"

void AddSkel(WorldPtr world) {
  world->addSkeleton(AddBox());
  world->addSkeleton(AddGround());
}

SkeletonPtr AddBox() {
  double jnt_dmpin = 0.0;
  double frcton_cff = 0.0;
  double mass = 0.8;
  Eigen::Vector3d length_tuple(0.1, 0.1, 0.1);
  Eigen::Vector3d init_pos(0.0, 0.075, 0.0);
  Eigen::Quaterniond init_ori_Quat;  // arbitrary initial orientation
  init_ori_Quat.w() = 1.0;
  init_ori_Quat.vec() = Eigen::Vector3d::Random();
  init_ori_Quat.normalize();
  Eigen::Matrix3d init_ori = init_ori_Quat.toRotationMatrix();
  // Eigen::Matrix3d init_ori = Eigen::Matrix3d::Identity();

  SkeletonPtr mBox = Skeleton::create("mBox");

  BodyNodePtr bn = mBox->createJointAndBodyNodePair<FreeJoint>().second;
  bn->getParentJoint()->setName("Joint_1");
  bn->setName("BodyNode_1");

  std::shared_ptr<BoxShape> boxshpe = std::make_shared<BoxShape>(length_tuple);
  boxshpe->setColor(dart::Color::Red(1));
  bn->addVisualizationShape(boxshpe);
  bn->addCollisionShape(boxshpe);

  // set inertia
  Inertia boxinrtia;
  boxinrtia.setMass(mass);
  boxinrtia.setMoment(boxshpe->computeInertia(boxinrtia.getMass()));
  bn->setInertia(boxinrtia);

  // put the body into the right position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = init_pos;
  // tf.linear() = init_ori;
  bn->getParentJoint()->setTransformFromParentBodyNode(tf);

  // disable friction
  //   bn->setFrictionCoeff(frcton_cff);
  //
  //   // disable joint friction
  //   for (size_t i = 0; i<bn->getParentJoint()->getNumDofs();++i)
  //   {
  //       bn->getParentJoint()->getDof(i)->setDampingCoefficient(jnt_dmpin);
  //   }

  return mBox;
}

SkeletonPtr AddGround() {
  return dart::utils::SkelParser::readSkeleton(DART_DATA_PATH
                                               "skel/ground.skel");
}
