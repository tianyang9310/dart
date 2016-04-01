/*************************************************************************
    > File Name: toy_example_MPC_2.cpp
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Wed 16 Mar 2016 09:10:48 AM EDT
 ************************************************************************/

#include <iostream>
#include "dart/dart.h"
#include "ControlPBP/ControlPBP.h"
#include "toyexample/world_setup.h"
#include "toyexample/MyWindow.h"
#include "toyexample/Controller.h"


using namespace dart::common;
using namespace dart::utils;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;
using namespace dart::gui;

using namespace toyexample;

// parameter setting


int main(int argc, char* argv[])
{

	std::cout<<"This is a toy example for Model Predictive Control"<<std::endl;
	SkeletonPtr world_setup = createFloor();
	addWall(world_setup, world_setup->getBodyNode("floor"),1);
	addWall(world_setup, world_setup->getBodyNode("floor"),2);
	addWall(world_setup, world_setup->getBodyNode("floor"),3);
	addWall(world_setup, world_setup->getBodyNode("floor"),4);
	addObstacle(world_setup, world_setup->getBodyNode("floor"),1);
	addObstacle(world_setup, world_setup->getBodyNode("floor"),2);
	addObstacle(world_setup, world_setup->getBodyNode("floor"),3);

	SkeletonPtr cube = createCube();

	WorldPtr world = std::make_shared<World>();
	world->addSkeleton(world_setup);
	world->addSkeleton(cube);

	MyWindow window(world);

	std::cout<<"1: target; 2: first obstacle; 3: second obstacle; 4: third obstacle"<<std::endl;
	std::cout<<"h: move left"<<std::endl<<"j: move down"<<std::endl<<"k: move up"<<std::endl<<"l: move right"<<std::endl<<"r: reset"<<std::endl;

	glutInit(&argc, argv);
	window.initWindow(1024, 768, "TOY EXAMPLE");
	glutMainLoop();
	
	return 0;
}
