#include "addSkeles.h"

void AddSkel(WorldPtr world) {
  world->addSkeleton(AddBox());
  world->addSkeleton(AddGround());
}

SkeletonPtr AddBox() {
  double jnt_dmpin = 0.0;
  double frcton_cff = 0.0;
  double mass = 1.0;
  Eigen::Vector3d length_tuple(0.1, 0.1, 0.1);
  Eigen::Vector3d init_pos(0.0, 0.0749, 0.0);
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

  std::shared_ptr<Shape> shpe;
  switch (SHAPE){
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

  return mBox;
}

SkeletonPtr AddGround() {
  return dart::utils::SkelParser::readSkeleton(DART_DATA_PATH
                                               "skel/ground.skel");
}
