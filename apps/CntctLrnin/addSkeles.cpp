#include "addSkeles.h"

void AddSkel(WorldPtr world) {
  for (int i = 0; i < NUMCUBES; i++) {
    world->addSkeleton(AddBox("mBox" + std::to_string(i),
                              (Eigen::Vector3d() << 0, 0, 0.2 * i).finished()));
  }
  // world->addSkeleton(AddGround());
  world->addSkeleton(AddPlatform());
}

SkeletonPtr AddBox(string name, const Eigen::Vector3d& init_pos_offset) {
  // double jnt_dmpin = 0.0;
  // double frcton_cff = 0.0;
  // double rsttn_cff = 1.0;
  double mass = 1.0;
  Eigen::Vector3d length_tuple(0.1, 0.1, 0.1);
  Eigen::Vector3d init_pos(0.0, 0.255, 0.0);
  init_pos = init_pos + init_pos_offset;
  Eigen::Quaterniond init_ori_Quat;  // arbitrary initial orientation
  init_ori_Quat.w() = 1.0;
  init_ori_Quat.vec() = Eigen::Vector3d::Random();
  init_ori_Quat.normalize();
  Eigen::Matrix3d init_ori = init_ori_Quat.toRotationMatrix();
  // Eigen::Matrix3d init_ori = Eigen::Matrix3d::Identity();

  SkeletonPtr mBox = Skeleton::create(name);

  BodyNodePtr bn = mBox->createJointAndBodyNodePair<FreeJoint>().second;
  bn->getParentJoint()->setName("Joint_1");
  bn->setName("BodyNode_1");

  std::shared_ptr<Shape> shpe;
  switch (SHAPE) {
    case 0:
      shpe = std::make_shared<BoxShape>(length_tuple);
      break;
    case 1:
      shpe = std::make_shared<EllipsoidShape>(length_tuple);
      break;
    case 2:
      shpe = std::make_shared<CylinderShape>(0.05, 0.5);
    default:
      std::cout << "Unknown shape!!!" << std::endl;
  }

  shpe->setColor(dart::Color::Red(0.6));
  bn->addVisualizationShape(shpe);
  bn->addCollisionShape(shpe);

  // set inertia
  Inertia inrtia;
  inrtia.setMass(mass);
  inrtia.setMoment(shpe->computeInertia(inrtia.getMass()));
  bn->setInertia(inrtia);

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

  // change restitution coefficients
  bn->setRestitutionCoeff(rsttn_cff);

  return mBox;
}

SkeletonPtr AddGround() {
  return dart::utils::SkelParser::readSkeleton(DART_DATA_PATH
                                               "skel/ground.skel");
}

SkeletonPtr AddPlatform() {
  // double jnt_dmpin = 0.0;
  // double frcton_cff = 0.0;
  // double rsttn_cff = 1.0;
  double mass = 10.0;
  Eigen::Vector3d length_tuple(10000.0, 0.01, 1000.0);
  Eigen::Vector3d init_pos(0.0, 0.18, 0.0);
  Eigen::Quaterniond init_ori_Quat;  // arbitrary initial orientation
  init_ori_Quat.w() = 1.0;
  init_ori_Quat.vec() = Eigen::Vector3d::Random();
  init_ori_Quat.normalize();
  Eigen::Matrix3d init_ori = init_ori_Quat.toRotationMatrix();
  // Eigen::Matrix3d init_ori = Eigen::Matrix3d::Identity();

  SkeletonPtr mPlatform = Skeleton::create("mPlatform");

  BodyNodePtr bn0 = mPlatform->createJointAndBodyNodePair<WeldJoint>().second;
  bn0->getParentJoint()->setName("Joint_0");
  bn0->setName("BodyNode_0");

  std::shared_ptr<Shape> shpe0;
  shpe0 = std::make_shared<BoxShape>(Eigen::Vector3d(0.01, 0.01, 0.01));
  shpe0->setColor(dart::Color::Blue(0.8));
  bn0->addVisualizationShape(shpe0);
  bn0->addCollisionShape(shpe0);

  BodyNodePtr bn =
      mPlatform->createJointAndBodyNodePair<RevoluteJoint>(bn0).second;
  bn->getParentJoint()->setName("Joint_1");
  bn->setName("BodyNode_1");

  std::shared_ptr<Shape> shpe;
  shpe = std::make_shared<BoxShape>(length_tuple);
  shpe->setColor(dart::Color::Green(0.8));
  bn->addVisualizationShape(shpe);
  bn->addCollisionShape(shpe);

  // set inertia
  Inertia inrtia;
  inrtia.setMass(mass);
  inrtia.setMoment(shpe->computeInertia(inrtia.getMass()));
  bn->setInertia(inrtia);

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

  /*
   * // change restitution coefficients
   * bn->setRestitutionCoeff(rsttn_cff);
   */

  mPlatform->setMobile(false);
  mPlatform->disableSelfCollision();
  return mPlatform;
}
