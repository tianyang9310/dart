#include "addSkeles.h"

void AddSkel(WorldPtr world)
{
    world->addSkeleton(AddBox());
}

SkeletonPtr AddBox()
{
    double jnt_dmpin = 0.0;
    double frcton_cff = 0.0;
    double mass = 1.0;
    Eigen::Vector3d length_tuple(0.1,0.1,0.1);
    Eigen::Vector3d init_pos(0.0, 1.0, 0.0);
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
    bn->getParentJoint()->setTransformFromParentBodyNode(tf);

    // disable friction
    bn->setFrictionCoeff(frcton_cff);

    // disable joint friction
    for (size_t i = 0; i<bn->getParentJoint()->getNumDofs();++i)
    {
        bn->getParentJoint()->getDof(i)->setDampingCoefficient(jnt_dmpin);
    }

    return mBox;
}
