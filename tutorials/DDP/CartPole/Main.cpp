#include <iostream>
#include "dart/dart.h"
#include "MyWindow.h"
#include "WorldSetup.h"
#include "Controller.h"
#include "CartPoleUtility.h"
#include "../libDDP/DDP.h"


using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace dart::common;
using namespace dart::math;
using namespace dart::gui;


int main(int argc, char* argv[])
{
	WorldPtr mWorld = std::make_shared<World>();
    mWorld->setTimeStep(0.02);
	WorldSetup(mWorld);

	MyWindow window(mWorld);

	mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setPosition((window.mController->x0)(0));
	mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->setPosition((window.mController->x0)(1));
	mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setVelocity((window.mController->x0)(2));
	mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->setVelocity((window.mController->x0)(3));
#ifdef mSTAT
	std::cout<<"-----------------------------------------------------------------------------------"<<std::endl;
	std::cout<<"Gravity is "<<mWorld->getGravity().transpose()<<std::endl;
	std::cout<<"-----------------------------------------------------------------------------------"<<std::endl;
#endif

	glutInit(&argc, argv);
	window.initWindow(1024, 768, "Vehicle");
	glutMainLoop();

	return 0;
}
