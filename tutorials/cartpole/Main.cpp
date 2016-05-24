#include <iostream>
#include "dart/dart.h"
#include "MyWindow.h"
#include "WorldSetup.h"
#include "Controller.h"
#include "DDP.h"


using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace dart::common;
using namespace dart::math;
using namespace dart::gui;

double g = -1;

int main(int argc, char* argv[])
{
	WorldPtr mWorld = std::make_shared<World>();
	WorldSetup(mWorld);

	g = -mWorld->getGravity()(2);

	MyWindow window(mWorld);
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
