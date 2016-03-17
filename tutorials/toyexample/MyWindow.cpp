/*************************************************************************
    > File Name: MyWindow.cpp > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Wed 16 Mar 2016 11:53:26 AM EDT
 ************************************************************************/

#include "MyWindow.h"
namespace toyexample{

MyWindow::MyWindow(WorldPtr world)
{
	setWorld(world);	

	mWorld->getConstraintSolver()->setCollisionDetector(new dart::collision::BulletCollisionDetector());
	detector = mWorld->getConstraintSolver()->getCollisionDetector();
	detector->detectCollision(true, true);

	mController =  std::unique_ptr<Controller>(new Controller(mWorld->getSkeleton("cube"), mWorld->getSkeleton("world_setup"), detector));

	mController->setCubeVelocity();
}

void MyWindow::timeStepping() 
{
	if (mController->collisionEvent_detector())
	{
		std::cout<<"collision detected!"<<std::endl;
	}

	SimWindow::timeStepping();
}




} // namespace toyexample
