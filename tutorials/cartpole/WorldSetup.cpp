/*************************************************************************
    > File Name: WorldSetup.cpp
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Thu May 19 22:53:10 2016
 ************************************************************************/

#include <iostream>
#include "dart/dart.h"
using namespace dart;
using namespace dynamics;
using namespace simulation;
using namespace utils;

SkeletonPtr addFloor()
{
	SkeletonPtr mFloor = Skeleton::create("mFloor");

	// create a bodynode 
	BodyNodePtr floor = mFloor->createJointAndBodyNodePair<WeldJoint>().second;

	floor->setName("floor");

	// attach a shape
	std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(
			Eigen::Vector3d(5, 5, 0.1));
	box->setColor(dart::Color::Gray(0.3));
	floor->addVisualizationShape(box);
	floor->addCollisionShape(box);

	// set inertia
	dart::dynamics::Inertia inertia;
	inertia.setMass(5510 * box->getVolume());
	inertia.setMoment(box->computeInertia(inertia.getMass()));
	floor->setInertia(inertia);

	// put the body into the right position
	Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
	tf.translation() = Eigen::Vector3d(0.0, 0.0, -0.1 / 2.0);
	floor->getParentJoint()->setTransformFromParentBodyNode(tf);

	// disable friction
	floor->setFrictionCoeff(0);
	std::cout<<"floor friction is set as "<<floor->getBodyNodeProperties().mFrictionCoeff<<std::endl;
	return mFloor;
}


SkeletonPtr addCartPole()
{
	SkeletonPtr mCartPole = Skeleton::create("mCartPole");
//--------------------------------------------------------------------------------------------------------------
	// create a bodynode 
	BodyNodePtr mCart = mCartPole->createJointAndBodyNodePair<PlanarJoint>().second;

	mCart->setName("mCart");

	// attach a shape
	std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(
			Eigen::Vector3d(0.2, 0.2, 0.1));
	box->setColor(dart::Color::Red(0.7));
	mCart->addVisualizationShape(box);
	mCart->addCollisionShape(box);

	// set inertia
	dart::dynamics::Inertia inertia_box;
	inertia_box.setMass(3500 * box->getVolume());
	inertia_box.setMoment(box->computeInertia(inertia_box.getMass()));
	mCart->setInertia(inertia_box);

	// put the body into the right position
	Eigen::Isometry3d tf_cart(Eigen::Isometry3d::Identity());
	tf_cart.translation() = Eigen::Vector3d(0.0, 0.0, 0.1 / 2.0);
	mCart->getParentJoint()->setTransformFromParentBodyNode(tf_cart);

	// disable friction
	mCart->setFrictionCoeff(0);
	//std::cout<<"mCart friction is set as "<<mCart->getBodyNodeProperties().mFrictionCoeff<<std::endl;
//--------------------------------------------------------------------------------------------------------------
	// create a bodynode 
	RevoluteJoint::Properties properties;
	properties.mName  = "Joint_cart_pole";
	BodyNodePtr mPole = mCartPole->createJointAndBodyNodePair<RevoluteJoint>(mCart, properties).second;
	mPole->setName("mPole");

	// attach a shape
	std::shared_ptr<CylinderShape> cylinder = std::make_shared<CylinderShape>(0.02,0.25);
	cylinder->setColor(dart::Color::Blue(0.7));
	mPole->addVisualizationShape(cylinder);
	mPole->addCollisionShape(cylinder);

	// set inertia
	dart::dynamics::Inertia inertia_cylinder;
	inertia_cylinder.setMass(3500 * cylinder->getVolume());
	inertia_cylinder.setMoment(cylinder->computeInertia(inertia_cylinder.getMass()));
	mPole->setInertia(inertia_cylinder);

	// put the body into the right position
	Eigen::Isometry3d tf_pole(Eigen::Isometry3d::Identity());
	tf_pole.translation() = Eigen::Vector3d(0.0, 0.0, 0.25 / 2.0);
	mPole->getParentJoint()->setTransformFromParentBodyNode(tf_pole);
	
	// disable frictin
	mPole->setFrictionCoeff(0);
//--------------------------------------------------------------------------------------------------------------
	return mCartPole;
}

void WorldSetup(WorldPtr mWorld)
{
	mWorld->addSkeleton(addFloor());
	mWorld->addSkeleton(addCartPole());
}
