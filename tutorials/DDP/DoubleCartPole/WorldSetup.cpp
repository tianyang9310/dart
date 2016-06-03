#include "WorldSetup.h"

SkeletonPtr addDoubleCartPole()
{
	double hold_mass			= 1e200;
	double cart_length			= 0.15;
	double cart_width			= 0.08;
	double cart_height			= 0.025;
	double cart_mass 		    = 1;
	double pole_radius			= 0.005;
	double pole_height			= 0.25;
	double pole_mass		    = 1e-200;
	double end_perimeter		= 0.001*20;
	double end_mass			    = 1;
	double BN_friction			= 0;
	double Joint_damping		= 0;
// --------------------------------------------------
	SkeletonPtr mDoubleCartPole = Skeleton::create("mDoubleCartPole");
// --------------------------------------------------
	BodyNodePtr mHold = mDoubleCartPole->createJointAndBodyNodePair<WeldJoint>().second;
	mHold->getParentJoint()->setName("Joint_world_hold");
	mHold->setName("mHold");
	mHold->setFrictionCoeff(BN_friction);
	mHold->setMass(hold_mass);
// --------------------------------------------------
	// create a bodynode 
	PrismaticJoint::Properties properties_cart;
	properties_cart.mAxis = Eigen::Vector3d::UnitX();
	BodyNodePtr mCart_body = mDoubleCartPole->createJointAndBodyNodePair<PrismaticJoint>(mHold,properties_cart).second;

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
	inertia_box.setMass(cart_mass);
	inertia_box.setMoment(box_cart->computeInertia(inertia_box.getMass()));
	mCart_body->setInertia(inertia_box);

	// put the body into the right position
	Eigen::Isometry3d tf_cart(Eigen::Isometry3d::Identity());
	tf_cart.translation() = Eigen::Vector3d(0.0, 0.0, cart_height / 2.0);
	mCart_body->getParentJoint()->setTransformFromParentBodyNode(tf_cart);

	// disable friction
	mCart_body->setFrictionCoeff(BN_friction);

	// disable joint friction
	for (size_t i = 0; i<mCart_body->getParentJoint()->getNumDofs();++i)
	{
		mCart_body->getParentJoint()->getDof(i)->setDampingCoefficient(Joint_damping);
	}
// --------------------------------------------------
	// create a bodynode 
	RevoluteJoint::Properties properties_pole;
	properties_pole.mAxis  = Eigen::Vector3d::UnitY();
	BodyNodePtr mPole_body = mDoubleCartPole->createJointAndBodyNodePair<RevoluteJoint>(mCart_body, properties_pole).second;

	mPole_body->getParentJoint()->setName("Joint_cart_pole");
	mPole_body->setName("mPole_body");

	// attach a shape
	std::shared_ptr<CylinderShape> cylinder_pole = std::make_shared<CylinderShape>(pole_radius,pole_height);
	cylinder_pole->setColor(dart::Color::Blue(0.7));
	mPole_body->addVisualizationShape(cylinder_pole);
	mPole_body->addCollisionShape(cylinder_pole);

	// set inertia
	dart::dynamics::Inertia inertia_cylinder;
	inertia_cylinder.setMass(pole_mass);
	inertia_cylinder.setMoment(cylinder_pole->computeInertia(inertia_cylinder.getMass()));
	mPole_body->setInertia(inertia_cylinder);

	// put the body into the right position
	Eigen::Isometry3d Joint_cart_pole_parent(Eigen::Isometry3d::Identity());
	Joint_cart_pole_parent.translation() = Eigen::Vector3d(0.0, 0.0, cart_height/2);
	mPole_body->getParentJoint()->setTransformFromParentBodyNode(Joint_cart_pole_parent);

	Eigen::Isometry3d Joint_cart_pole_child(Eigen::Isometry3d::Identity());
	Joint_cart_pole_child.translation() = Eigen::Vector3d(0.0, 0.0, pole_height/2);
	mPole_body->getParentJoint()->setTransformFromChildBodyNode(Joint_cart_pole_child);

	
	// disable bodynode friction
	mPole_body->setFrictionCoeff(BN_friction);

	// disable joint friction
	for (size_t i = 0; i<mPole_body->getParentJoint()->getNumDofs();++i)
	{
		mPole_body->getParentJoint()->getDof(i)->setDampingCoefficient(Joint_damping);
	}

	mPole_body->getParentJoint()->getDof(0)->setPosition(0);//M_PI);
// --------------------------------------------------
	BodyNodePtr mPole_end = mDoubleCartPole->createJointAndBodyNodePair<WeldJoint>(mPole_body).second;
	mPole_end->getParentJoint()->setName("Joint_pole_end");
	mPole_end->setName("mPole_end");

	std::shared_ptr<EllipsoidShape> ellipsoidshape_end = std::make_shared<EllipsoidShape>(Eigen::Vector3d::Constant(end_perimeter));
	ellipsoidshape_end->setColor(dart::Color::Black(0.7));
	mPole_end->addVisualizationShape(ellipsoidshape_end);
	mPole_end->addCollisionShape(ellipsoidshape_end);

	dart::dynamics::Inertia Inertia_ellipsoid;
	Inertia_ellipsoid.setMass(end_mass);
	Inertia_ellipsoid.setMoment(ellipsoidshape_end->computeInertia(Inertia_ellipsoid.getMass()));
	mPole_end->setInertia(Inertia_ellipsoid);

	Eigen::Isometry3d Joint_pole_end(Eigen::Isometry3d::Identity());
	Joint_pole_end.translation() = Eigen::Vector3d(0.0, 0.0, -pole_height/2);
	mPole_end->getParentJoint()->setTransformFromParentBodyNode(Joint_pole_end);

	mPole_end->setFrictionCoeff(BN_friction);
// --------------------------------------------------
	// create a bodynode 
	RevoluteJoint::Properties properties_pole2;
	properties_pole2.mAxis  = Eigen::Vector3d::UnitY();
	BodyNodePtr mPole2_body = mDoubleCartPole->createJointAndBodyNodePair<RevoluteJoint>(mPole_body, properties_pole2).second;

	mPole2_body->getParentJoint()->setName("Joint_pole_pole2");
	mPole2_body->setName("mPole2_body");

	// attach a shape
	std::shared_ptr<CylinderShape> cylinder_pole2 = std::make_shared<CylinderShape>(pole_radius,pole_height);
	cylinder_pole2->setColor(dart::Color::Green(0.7));
	mPole2_body->addVisualizationShape(cylinder_pole2);
	mPole2_body->addCollisionShape(cylinder_pole2);

	// set inertia
	dart::dynamics::Inertia inertia_cylinder2;
	inertia_cylinder2.setMass(pole_mass);
	inertia_cylinder2.setMoment(cylinder_pole2->computeInertia(inertia_cylinder2.getMass()));
	mPole2_body->setInertia(inertia_cylinder2);

	// put the body into the right position
	Eigen::Isometry3d Joint_pole_pole2_parent(Eigen::Isometry3d::Identity());
	Joint_pole_pole2_parent.translation() = Eigen::Vector3d(0.0, 0.0, -pole_height/2);
	mPole2_body->getParentJoint()->setTransformFromParentBodyNode(Joint_pole_pole2_parent);

	Eigen::Isometry3d Joint_pole_pole2_child(Eigen::Isometry3d::Identity());
	Joint_pole_pole2_child.translation() = Eigen::Vector3d(0.0, 0.0, pole_height/2);
	mPole2_body->getParentJoint()->setTransformFromChildBodyNode(Joint_pole_pole2_child);

	
	// disable bodynode friction
	mPole2_body->setFrictionCoeff(BN_friction);

	// disable joint friction
	for (size_t i = 0; i<mPole2_body->getParentJoint()->getNumDofs();++i)
	{
		mPole2_body->getParentJoint()->getDof(i)->setDampingCoefficient(Joint_damping);
	}

	mPole2_body->getParentJoint()->getDof(0)->setPosition(0);//M_PI);
// --------------------------------------------------
	BodyNodePtr mPole2_end = mDoubleCartPole->createJointAndBodyNodePair<WeldJoint>(mPole2_body).second;
	mPole2_end->getParentJoint()->setName("Joint_pole2_end");
	mPole2_end->setName("mPole2_end");

	std::shared_ptr<EllipsoidShape> ellipsoidshape_end2 = std::make_shared<EllipsoidShape>(Eigen::Vector3d::Constant(end_perimeter));
	ellipsoidshape_end2->setColor(dart::Color::Black(0.7));
	mPole2_end->addVisualizationShape(ellipsoidshape_end2);
	mPole2_end->addCollisionShape(ellipsoidshape_end2);

	dart::dynamics::Inertia Inertia_ellipsoid2;
	Inertia_ellipsoid2.setMass(end_mass);
	Inertia_ellipsoid2.setMoment(ellipsoidshape_end2->computeInertia(Inertia_ellipsoid2.getMass()));
	mPole2_end->setInertia(Inertia_ellipsoid2);

	Eigen::Isometry3d Joint_pole2_end(Eigen::Isometry3d::Identity());
	Joint_pole2_end.translation() = Eigen::Vector3d(0.0, 0.0, -pole_height/2);
	mPole2_end->getParentJoint()->setTransformFromParentBodyNode(Joint_pole2_end);

	mPole2_end->setFrictionCoeff(BN_friction);

// --------------------------------------------------
#ifdef mSTAT
	std::cout<<"-----------------------------------------------------------------------------------"<<std::endl;
	std::cout<<"BodyNode volumes are "<<std::endl;
	std::cout<<"Cart body:		"<<box_cart->getVolume()<<std::endl;
	std::cout<<"Pole body:		"<<cylinder_pole->getVolume()<<std::endl;
	std::cout<<"Pole end:		"<<ellipsoidshape_end->getVolume()<<std::endl;
	std::cout<<"-----------------------------------------------------------------------------------"<<std::endl;
	std::cout<<"Mass and moment distribution: "<<std::endl;
	for (size_t i=0; i<mDoubleCartPole->getNumBodyNodes(); i++)
	{
		std::cout<<mDoubleCartPole->getBodyNode(i)->getName()<<std::endl<<"mass is "<<mDoubleCartPole->getBodyNode(i)->getInertia().getMass()<<std::endl<<"moment is "<<std::endl<<mDoubleCartPole->getBodyNode(i)->getInertia().getMoment()<<std::endl;
		std::cout<<"-----------------------------------------------------------------------------------"<<std::endl;
	}
	std::cout<<"BodyNode frictions are: "<<std::endl;
	for (size_t i=0; i<mDoubleCartPole->getNumBodyNodes(); i++)
	{
		std::cout<<mDoubleCartPole->getBodyNode(i)->getName()<<":		"<<mDoubleCartPole->getBodyNode(i)->getBodyNodeProperties().mFrictionCoeff<<std::endl;
	}
	std::cout<<"-----------------------------------------------------------------------------------"<<std::endl;
	std::cout<<"Degree of freedom dampings are: "<<std::endl;
	for (size_t i=0; i<mDoubleCartPole->getNumDofs(); i++)
	{
		std::cout<<mDoubleCartPole->getDof(i)->getName()<<":	"<<mDoubleCartPole->getDof(i)->getDampingCoefficient()<<std::endl;
	}
	std::cout<<"-----------------------------------------------------------------------------------"<<std::endl;
#endif
// --------------------------------------------------
	return mDoubleCartPole;
}

void WorldSetup(WorldPtr mWorld)
{
	mWorld->addSkeleton(addDoubleCartPole());
}
