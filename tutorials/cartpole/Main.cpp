#include <iostream>
#include "dart/dart.h"
#include "MyWindow.h"
#include "WorldSetup.h"

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace dart::common;
using namespace dart::math;
using namespace dart::gui;

int main(int argc, char* argv[])
{
	WorldPtr mWorld = std::make_shared<World>();
	WorldSetup(mWorld);

	MyWindow window(mWorld);
	std::cout<<"Gravity is "<<mWorld->getGravity().transpose()<<std::endl;
	std::cout<<"Degree of Freedoms are"<<std::endl;
	for(size_t i=0; i<mWorld->getSkeleton("mCartPole")->getNumDofs();i++)
	{
		std::cout<<mWorld->getSkeleton("mCartPole")->getDof(i)->getName()<<" "<<std::endl;
	}
	std::cout<<std::endl;

	glutInit(&argc, argv);
	window.initWindow(640, 480, "Vehicle");
	glutMainLoop();

	return 0;
}
