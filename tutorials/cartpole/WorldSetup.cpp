/*************************************************************************
    > File Name: WorldSetup.cpp
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Thu May 19 22:53:10 2016
 ************************************************************************/

#include "WorldSetup.h"

SkeletonPtr addCartPole()
{
	double cart_length			= 0.15;
	double cart_width			= 0.1;
	double cart_height			= 0.025;
	double cart_density 		= 50;
	double pole_radius			= 0.005;
	double pole_height			= 0.3;
	double pole_density		    = 200;
	double BN_friction			= 0;
	double Joint_damping		= 0;
//--------------------------------------------------------------------------------------------------------------

	SkeletonPtr mCartPole = Skeleton::create("mCartPole");
//--------------------------------------------------------------------------------------------------------------
	BodyNodePtr mHold = mCartPole->createJointAndBodyNodePair<WeldJoint>().second;
	mHold->getParentJoint()->setName("Joint_world_hold");
	mHold->setName("mHold");
	mHold->setFrictionCoeff(BN_friction);

//--------------------------------------------------------------------------------------------------------------
	// create a bodynode 
	PrismaticJoint::Properties properties_cart;
	properties_cart.mAxis = Eigen::Vector3d::UnitX();
	BodyNodePtr mCart_body = mCartPole->createJointAndBodyNodePair<PrismaticJoint>(mHold,properties_cart).second;

	mCart_body->getParentJoint()->setName("Joint_hold_cart");
	mCart_body->setName("mCart_body");

	// attach a shape
	std::shared_ptr<BoxShape> box_cart = std::make_shared<BoxShape>(
			Eigen::Vector3d(cart_length,cart_width,cart_height));
	box_cart->setColor(dart::Color::Red(0.7));
	mCart_body->addVisualizationShape(box_cart);
	mCart_body->addCollisionShape(box_cart);

	// set inertia
	dart::dynamics::Inertia inertia_box;
	inertia_box.setMass(cart_density * box_cart->getVolume());
	inertia_box.setMoment(box_cart->computeInertia(inertia_box.getMass()));
	mCart_body->setInertia(inertia_box);

	// put the body into the right position
	Eigen::Isometry3d tf_cart(Eigen::Isometry3d::Identity());
	tf_cart.translation() = Eigen::Vector3d(0.0, 0.0, cart_height / 2.0);
	mCart_body->getParentJoint()->setTransformFromParentBodyNode(tf_cart);

	// disable friction
	mCart_body->setFrictionCoeff(BN_friction);
	std::cout<<"mCart_body friction is set as "<<mCart_body->getBodyNodeProperties().mFrictionCoeff<<std::endl;

	// disable joint friction
	for (size_t i = 0; i<mCart_body->getParentJoint()->getNumDofs();++i)
	{
		mCart_body->getParentJoint()->getDof(i)->setDampingCoefficient(Joint_damping);
	}
//--------------------------------------------------------------------------------------------------------------
	// create a bodynode 
	RevoluteJoint::Properties properties_pole;
	properties_pole.mAxis  = Eigen::Vector3d::UnitY();
	BodyNodePtr mPole_body = mCartPole->createJointAndBodyNodePair<RevoluteJoint>(mCart_body, properties_pole).second;

	mPole_body->getParentJoint()->setName("Joint_cart_pole");
	mPole_body->setName("mPole_body");

	std::cout<<"Axis of revolute joint is "<<properties_pole.mAxis.transpose()<<std::endl;

	// attach a shape
	std::shared_ptr<CylinderShape> cylinder_pole = std::make_shared<CylinderShape>(pole_radius,pole_height);
	cylinder_pole->setColor(dart::Color::Blue(0.7));
	mPole_body->addVisualizationShape(cylinder_pole);
	mPole_body->addCollisionShape(cylinder_pole);

	// set inertia
	dart::dynamics::Inertia inertia_cylinder;
	inertia_cylinder.setMass(pole_density * cylinder_pole->getVolume());
	inertia_cylinder.setMoment(cylinder_pole->computeInertia(inertia_cylinder.getMass()));
	mPole_body->setInertia(inertia_cylinder);

	std::cout<<"Pole moment is "<<std::endl<<mPole_body->getInertia().getMoment()<<std::endl;

	// put the body into the right position
	Eigen::Isometry3d Joint_cart_pole_parent(Eigen::Isometry3d::Identity());
	Joint_cart_pole_parent.translation() = Eigen::Vector3d(0.0, 0.0, cart_height/2);
	mPole_body->getParentJoint()->setTransformFromParentBodyNode(Joint_cart_pole_parent);

	Eigen::Isometry3d Joint_cart_pole_child(Eigen::Isometry3d::Identity());
	Joint_cart_pole_child.translation() = Eigen::Vector3d(0.0, 0.0, pole_height/2);
	mPole_body->getParentJoint()->setTransformFromChildBodyNode(Joint_cart_pole_child);

	
	// disable bodynode friction
	mPole_body->setFrictionCoeff(BN_friction);
	std::cout<<"mPole friction is set as "<<mPole_body->getBodyNodeProperties().mFrictionCoeff<<std::endl;

	// disable joint friction
	for (size_t i = 0; i<mPole_body->getParentJoint()->getNumDofs();++i)
	{
		mPole_body->getParentJoint()->getDof(i)->setDampingCoefficient(Joint_damping);
	}
//--------------------------------------------------------------------------------------------------------------
	mPole_body->getParentJoint()->getDof(0)->setPosition(0);
	return mCartPole;
}

void WorldSetup(WorldPtr mWorld)
{
	mWorld->addSkeleton(addCartPole());
}
