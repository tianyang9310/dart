/*************************************************************************
    > File Name: Controller.cpp
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Wed 16 Mar 2016 01:09:05 PM EDT
 ************************************************************************/


#include "Controller.h"

namespace toyexample{

extern const double obstacle_radius;
extern const double wall_thickness;
extern const double cube_length;

Controller::Controller(SkeletonPtr cube, SkeletonPtr world_setup)
{
	mCube          = cube;
	mWorld_Setup   = world_setup;
	mSpeed         = 0.1;		
	mCollisionThre = cube_length;
}

void Controller::setCubeVelocity()
{
	mCube->getDof(0)->setVelocity(mSpeed);	
}

bool Controller::collisionEvent()
{
	bool collision = false;
	
	Eigen::MatrixXd cubePos(2,1);
	cubePos = mCube->getBodyNode(0)->getTransform().translation().matrix().block<2,1>(0, 0);

	Eigen::MatrixXd ob_1Pos(2,1);
	ob_1Pos = mWorld_Setup->getBodyNode("obstacle_1")->getTransform().translation().matrix().block<2,1>(0, 0);

	Eigen::MatrixXd ob_2Pos(2,1);
	ob_2Pos = mWorld_Setup->getBodyNode("obstacle_2")->getTransform().translation().matrix().block<2,1>(0, 0);

	Eigen::MatrixXd ob_3Pos(2,1);
	ob_3Pos = mWorld_Setup->getBodyNode("obstacle_3")->getTransform().translation().matrix().block<2,1>(0, 0);

	Eigen::MatrixXd wl_3Pos(2,1); // 0, -0.095
	wl_3Pos = mWorld_Setup->getBodyNode("wall_3")->getTransform().translation().matrix().block<2,1>(0, 0);

	Eigen::MatrixXd wl_4Pos(2,1); // 0, 0.095
	wl_4Pos = mWorld_Setup->getBodyNode("wall_4")->getTransform().translation().matrix().block<2,1>(0, 0);
	
	// detect the collision between cube and obstacle
	if ((cubePos-ob_1Pos).norm() <= (mCollisionThre + obstacle_radius))
	{
		collision = true;
		return collision;
	}
	else if ((cubePos-ob_2Pos).norm() <= (mCollisionThre + obstacle_radius/2.0))
	{
		collision = true;
		return collision;
	}
	else if ((cubePos-ob_3Pos).norm() <= (mCollisionThre + obstacle_radius/2.0))
	{
		collision = true;
		return collision;
	}
	else if ((cubePos.block<1,1>(1,0) - wl_3Pos.block<1,1>(1,0)).norm() <= (mCollisionThre + wall_thickness / 2.0))
	{
		collision = true;
		return collision;
	}
	else if ((cubePos.block<1,1>(1,0) - wl_4Pos.block<1,1>(1,0)).norm() <= (mCollisionThre + wall_thickness / 2.0))
	{
		collision = true;
		return collision;
	}

	return collision;
}

} //namespace toyexample
