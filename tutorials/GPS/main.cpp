#include "dart/dart.h"
#include "CartPole/WorldSetup.h"
#include "GUI/MyWindow.h"

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


	glutInit(&argc, argv);
	window.initWindow(1024, 768, "Guided Policy Search");
	glutMainLoop();

	return 0;
}
