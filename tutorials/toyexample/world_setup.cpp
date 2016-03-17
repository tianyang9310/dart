/*************************************************************************
    > File Name: world_setup.cpp
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Wed 16 Mar 2016 10:46:00 AM EDT
 ************************************************************************/

#include "world_setup.h"
namespace toyexample{

SkeletonPtr createFloor()
{
	SkeletonPtr world_setup = Skeleton::create("world_setup");

	// create a bodynode 
	BodyNodePtr floor = world_setup->createJointAndBodyNodePair<WeldJoint>().second;

	floor->setName("floor");

	// attach a shape
	std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(
			Eigen::Vector3d(floor_length, floor_width, floor_height));
	box->setColor(dart::Color::Gray(0.3));
	floor->addVisualizationShape(box);
	floor->addCollisionShape(box);

	// set inertia
	dart::dynamics::Inertia inertia;
	inertia.setMass(default_density * box->getVolume());
	inertia.setMoment(box->computeInertia(inertia.getMass()));
	floor->setInertia(inertia);

	// put the body into the right position
	Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
	tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0);
	floor->getParentJoint()->setTransformFromParentBodyNode(tf);

	return world_setup;
}


BodyNodePtr addWall(SkeletonPtr world_setup, BodyNodePtr parent, int wall_index)
{
	WeldJoint::Properties properties;
	properties.mName = "wall_weld_joint_" + std::to_string(wall_index);

	// flag1 defines different walls, flag2 defines different positions
	int flag1, flag2;							// 1, 2, 3, 4
	flag1 = 2* ((wall_index - 1) / 2) -1;       //-1,-1, 1, 1
	flag2 = 2* ((wall_index - 1) % 2) -1;       //-1, 1,-1, 1 

	// create a bodynode
	BodyNodePtr wall = world_setup->createJointAndBodyNodePair<WeldJoint>(parent, properties, BodyNode::Properties("wall_"+std::to_string(wall_index))).second;

	// attach a shape
	if( flag1 == -1)
	{
		std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(
			Eigen::Vector3d(wall_thickness, wall_length/2.0, wall_height));
		box->setColor(dart::Color::Gray(0.3));
		wall->addVisualizationShape(box);
		wall->addCollisionShape(box);
		
		// set inertia
		dart::dynamics::Inertia inertia;
		inertia.setMass(default_density * box->getVolume());
		inertia.setMoment(box->computeInertia(inertia.getMass()));
		wall->setInertia(inertia);
	}
	else
	{
		std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(
			Eigen::Vector3d(wall_length - 2*wall_thickness, wall_thickness, wall_height));
		box->setColor(dart::Color::Fuschia(0.3));
		wall->addVisualizationShape(box);
		wall->addCollisionShape(box);

		// set inertia
		dart::dynamics::Inertia inertia;
		inertia.setMass(default_density * box->getVolume());
		inertia.setMoment(box->computeInertia(inertia.getMass()));
		wall->setInertia(inertia);
	}

	// put the body node into the right position
	Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
	if( flag1 == -1)
	{
		tf.translation() = Eigen::Vector3d(flag2 * (floor_length / 2.0 - wall_thickness /2.0), 0.0, wall_height / 2.0);
	}
	else
	{
		tf.translation() = Eigen::Vector3d(0.0, flag2 * (floor_length / 4.0 -wall_thickness /2.0), wall_height / 2.0);
	}
	wall->getParentJoint()->setTransformFromParentBodyNode(tf);

	return wall;
}


BodyNodePtr addObstacle(SkeletonPtr world_setup, BodyNodePtr parent, int obstacle_index)
{
	WeldJoint::Properties properties;
	properties.mName = "obstacle_weld_joint_" + std::to_string(obstacle_index);

	// create a bodynode
	BodyNodePtr obstacle = world_setup->createJointAndBodyNodePair<WeldJoint>(parent, properties, BodyNode::Properties("obstacle_"+std::to_string(obstacle_index))).second;

	// create the shape
	if (obstacle_index <2)
	{
		std::shared_ptr<CylinderShape> cylinder = std::make_shared<CylinderShape>(obstacle_radius, obstacle_height);
		cylinder->setColor(dart::Color::Red(0.3));
		obstacle->addVisualizationShape(cylinder);
		obstacle->addCollisionShape(cylinder);
		// set inertia
		dart::dynamics::Inertia inertia;
		inertia.setMass(default_density * cylinder->getVolume());
		inertia.setMoment(cylinder->computeInertia(inertia.getMass()));
		obstacle->setInertia(inertia);
	}
	else
	{
		std::shared_ptr<CylinderShape> cylinder = std::make_shared<CylinderShape>(obstacle_radius/2.0, obstacle_height);
		cylinder->setColor(dart::Color::Red(0.3));
		obstacle->addVisualizationShape(cylinder);
		obstacle->addCollisionShape(cylinder);
		// set inertia
		dart::dynamics::Inertia inertia;
		inertia.setMass(default_density * cylinder->getVolume());
		inertia.setMoment(cylinder->computeInertia(inertia.getMass()));
		obstacle->setInertia(inertia);
	}

	// put it in the position
	Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
	if (obstacle_index < 2)
	{
		tf.translation() = Eigen::Vector3d(0.0, -floor_length/4 + obstacle_radius +  obstacle_2_wall, obstacle_height/2.0);
	}
	else
	{
		tf.translation() = Eigen::Vector3d(((obstacle_index - 2.5)*2)*floor_length/4.0, floor_length/4 - obstacle_radius/2.0 - obstacle_2_wall , obstacle_height/2.0);
	}

	obstacle->getParentJoint()->setTransformFromParentBodyNode(tf);

	return obstacle;
}


SkeletonPtr createCube()
{
	// create a Skeleton
	SkeletonPtr cube = Skeleton::create("cube");

	// create a BodyNode
	BodyNodePtr body = cube->createJointAndBodyNodePair<PlanarJoint>().second;
	
	// create a shape
	std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(Eigen::Vector3d(cube_length, cube_length, cube_length));
	//std::shared_ptr<EllipsoidShape> box = std::make_shared<EllipsoidShape>(Eigen::Vector3d(cube_length, cube_length, cube_length));
	box->setColor(dart::Color::Black(0.6));
	body->addVisualizationShape(box);
	body->addCollisionShape(box);
	

	// set inertia
	dart::dynamics::Inertia inertia;
	inertia.setMass(default_density * box->getVolume());
	inertia.setMoment(box->computeInertia(inertia.getMass()));
	body->setInertia(inertia);

	// put it in the right position
	Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
	tf.translation() = Eigen::Vector3d(-floor_length/2.0 + obstacle_2_wall, 0.0, cube_length/2.0);
	body->getParentJoint()->setTransformFromParentBodyNode(tf);

	return cube;
}

} // namespace toyexample
