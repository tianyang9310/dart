#include "addSkeles.h"

namespace CntctLrnin {

//==============================================================================
void addSkel(WorldPtr world) {
#ifdef UNIT_TEST
  world->addSkeleton(addBox());
#endif

#ifndef UNIT_TEST
  world->addSkeleton(
      addBox(NUMBODYNODES, (Eigen::Vector3d() << 0, 0, 0.2).finished(), true));
#endif
  world->addSkeleton(addPlatform());
}

//==============================================================================
SkeletonPtr addBox() {
  double mass = 1.0;
  Eigen::Vector3d lengthTuple(0.1, 0.1, 0.1);

  Eigen::Vector3d initPos(0.0, -0.2 + 0.01, 0.0);
  // Eigen::Quaterniond initOriQuat;
  // initOriQuat.w() = 1.0;
  // initOriQuat.vec() = Eigen::Vector3d::Random();
  // initOriQuat.normalize();
  // Eigen::Matrix3d initOri = initOriQuat.toRotationMatrix();
  // Eigen::Matrix3d initOri = Eigen::Matrix3d::Identity();

  SkeletonPtr mBox = Skeleton::create("mBox");

  BodyNodePtr bn = mBox->createJointAndBodyNodePair<FreeJoint>().second;
  bn->getParentJoint()->setName("Joint_" + std::to_string(0));
  bn->setName("BodyNode_" + std::to_string(0));

  std::shared_ptr<Shape> shpe;
  switch (SHAPE) {
    case mShapeType::cube:
      shpe = std::make_shared<BoxShape>(lengthTuple);
      break;
    case mShapeType::ball:
      shpe = std::make_shared<EllipsoidShape>(lengthTuple);
      break;
    default:
      dterr << "Unknown shape!!!" << std::endl;
  }

  shpe->setColor(dart::Color::Red(0.6));
  bn->addVisualizationShape(shpe);
  bn->addCollisionShape(shpe);

  Inertia inrtia;
  inrtia.setMass(mass);
  inrtia.setMoment(shpe->computeInertia(inrtia.getMass()));
  bn->setInertia(inrtia);

  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = initPos;
  // tf.linear() = initOri;
  bn->getParentJoint()->setTransformFromParentBodyNode(tf);

  bn->setRestitutionCoeff(rsttn_cff);

  mBox->enableSelfCollision();

  return mBox;
}

//==============================================================================
SkeletonPtr addBox(int numBodyNodes, const Eigen::Vector3d& initPos_offset,
                   bool isChain) {
  double mass = 1.0;
  Eigen::Vector3d lengthTuple(0.1, 0.1, 0.1);

  Eigen::Vector3d initPos(0.0, -0.2 + 0.01, 0.0);
  // Eigen::Quaterniond initOriQuat;
  // initOriQuat.w() = 1.0;
  // initOriQuat.vec() = Eigen::Vector3d::Random();
  // initOriQuat.normalize();
  // Eigen::Matrix3d initOri = initOriQuat.toRotationMatrix();
  // Eigen::Matrix3d initOri = Eigen::Matrix3d::Identity();

  SkeletonPtr mBox = Skeleton::create("mBox");

  if (isChain) {
    BodyNodePtr rootBodyNode =
        mBox->createJointAndBodyNodePair<FreeJoint>().second;
    rootBodyNode->getParentJoint()->setName("Joint_" + std::to_string(0));
    rootBodyNode->setName("BodyNode_" + std::to_string(0));

    std::shared_ptr<Shape> rootShpe;
    switch (SHAPE) {
      case mShapeType::cube:
        rootShpe = std::make_shared<BoxShape>(lengthTuple);
        break;
      case mShapeType::ball:
        rootShpe = std::make_shared<EllipsoidShape>(lengthTuple);
        break;
      default:
        dterr << "Unknown shape!!!" << std::endl;
    }

    rootShpe->setColor(dart::Color::Red(0.6));
    rootBodyNode->addVisualizationShape(rootShpe);
    rootBodyNode->addCollisionShape(rootShpe);

    Inertia rootInrtia;
    rootInrtia.setMass(mass);
    rootInrtia.setMoment(rootShpe->computeInertia(rootInrtia.getMass()));
    rootBodyNode->setInertia(rootInrtia);

    Eigen::Isometry3d roottf(Eigen::Isometry3d::Identity());
    roottf.translation() = initPos;
    // tf.linear() = initOri;
    rootBodyNode->getParentJoint()->setTransformFromParentBodyNode(roottf);

    rootBodyNode->setRestitutionCoeff(rsttn_cff);

    BodyNodePtr parentBn = rootBodyNode;

    for (int idxmBox = 1; idxmBox < NUMBODYNODES; ++idxmBox) {
      BodyNodePtr bn =
          mBox->createJointAndBodyNodePair<BallJoint>(parentBn).second;
      bn->getParentJoint()->setName("Joint_" + std::to_string(idxmBox));
      bn->setName("BodyNode_" + std::to_string(idxmBox));

      std::shared_ptr<Shape> shpe;
      switch (SHAPE) {
        case mShapeType::cube:
          shpe = std::make_shared<BoxShape>(lengthTuple);
          break;
        case mShapeType::ball:
          shpe = std::make_shared<EllipsoidShape>(lengthTuple);
          break;
        default:
          dterr << "Unknown shape!!!" << std::endl;
      }

      shpe->setColor(dart::Color::Red(0.6));
      bn->addVisualizationShape(shpe);
      bn->addCollisionShape(shpe);

      Inertia inrtia;
      inrtia.setMass(mass);
      inrtia.setMoment(shpe->computeInertia(inrtia.getMass()));
      bn->setInertia(inrtia);

      Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
      tf.translation() = initPos_offset;
      // tf.linear() = initOri;
      bn->getParentJoint()->setTransformFromParentBodyNode(tf);

      bn->setRestitutionCoeff(rsttn_cff);

      parentBn = bn;
    }
  } else {
    /// Dummy root body node
    BodyNodePtr rootBodyNode =
        mBox->createJointAndBodyNodePair<FreeJoint>().second;
    rootBodyNode->getParentJoint()->setName("RootJoint");
    rootBodyNode->setName("RootBodyNode");
    std::shared_ptr<Shape> rootShpe;
    rootShpe = std::make_shared<BoxShape>(Eigen::Vector3d(0.01, 0.01, 0.01));
    rootShpe->setColor(dart::Color::Orange(0.8));
    rootBodyNode->addVisualizationShape(rootShpe);
    rootBodyNode->addCollisionShape(rootShpe);

    // Eigen::Isometry3d roottf(Eigen::Isometry3d::Identity());
    // roottf.translation() = 2 * initPos;
    // rootBodyNode->getParentJoint()->setTransformFromParentBodyNode(roottf);

    for (int idxmBox = 0; idxmBox < numBodyNodes; idxmBox++) {
      BodyNodePtr bn =
          mBox->createJointAndBodyNodePair<EulerJoint>(rootBodyNode).second;
      bn->getParentJoint()->setName("Joint_" + std::to_string(idxmBox));
      bn->setName("BodyNode_" + std::to_string(idxmBox));

      std::shared_ptr<Shape> shpe;
      switch (SHAPE) {
        case mShapeType::cube:
          shpe = std::make_shared<BoxShape>(lengthTuple);
          break;
        case mShapeType::ball:
          shpe = std::make_shared<EllipsoidShape>(lengthTuple);
          break;
        default:
          dterr << "Unknown shape!!!" << std::endl;
      }

      shpe->setColor(dart::Color::Red(0.6));
      bn->addVisualizationShape(shpe);
      bn->addCollisionShape(shpe);

      Inertia inrtia;
      inrtia.setMass(mass);
      inrtia.setMoment(shpe->computeInertia(inrtia.getMass()));
      bn->setInertia(inrtia);

      Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
      tf.translation() = initPos + idxmBox * initPos_offset;
      // tf.linear() = initOri;
      bn->getParentJoint()->setTransformFromParentBodyNode(tf);

      bn->setRestitutionCoeff(rsttn_cff);
    }
  }

  mBox->enableSelfCollision();

  return mBox;
}

//==============================================================================
SkeletonPtr addPlatform() {
  double mass = 10.0;
#ifdef STATIC_SLOPE
  Eigen::Vector3d lengthTuple(1.0, 0.01, 1.0);
#else
  Eigen::Vector3d lengthTuple(10000.0, 0.01, 10000.0);
#endif

  Eigen::Vector3d initPos(0.0, 0.05, 0.0);
  // Eigen::Quaterniond initOriQuat;  // arbitrary initial orientation
  // initOriQuat.w() = 1.0;
  // initOriQuat.vec() = Eigen::Vector3d::Random();
  // initOriQuat.normalize();
  // Eigen::Matrix3d initOri = initOriQuat.toRotationMatrix();
  // Eigen::Matrix3d initOri = Eigen::Matrix3d::Identity();

  SkeletonPtr mPlatform = Skeleton::create("mPlatform");

  BodyNodePtr bn0 = mPlatform->createJointAndBodyNodePair<WeldJoint>().second;
  bn0->getParentJoint()->setName("Joint_0");
  bn0->setName("BodyNode_0");

  std::shared_ptr<Shape> shpe0;
  shpe0 = std::make_shared<BoxShape>(Eigen::Vector3d(0.01, 0.01, 0.01));
  shpe0->setColor(dart::Color::Blue(0.8));
  bn0->addVisualizationShape(shpe0);
  bn0->addCollisionShape(shpe0);

  Eigen::Isometry3d roottf(Eigen::Isometry3d::Identity());
  roottf.translation() = (Eigen::Vector3d() << 0.0, -0.3, 0.0).finished();
  bn0->getParentJoint()->setTransformFromParentBodyNode(roottf);

  BodyNodePtr bn =
      mPlatform->createJointAndBodyNodePair<RevoluteJoint>(bn0).second;
  bn->getParentJoint()->setName("Joint_1");
  bn->setName("BodyNode_1");

  std::shared_ptr<Shape> shpe;
  shpe = std::make_shared<BoxShape>(lengthTuple);
  shpe->setColor(dart::Color::Green(0.8));
  bn->addVisualizationShape(shpe);
  bn->addCollisionShape(shpe);

  Inertia inrtia;
  inrtia.setMass(mass);
  inrtia.setMoment(shpe->computeInertia(inrtia.getMass()));
  bn->setInertia(inrtia);

  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = initPos;
  // tf.linear() = initOri;
  bn->getParentJoint()->setTransformFromParentBodyNode(tf);

  /*
   * // change restitution coefficients
   * bn->setRestitutionCoeff(rsttn_cff);
   */

  mPlatform->setMobile(false);
  mPlatform->disableSelfCollision();
  return mPlatform;
}
}  // namespace CntctLrnin
