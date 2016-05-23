#include "MyWindow.h"

MyWindow::MyWindow(WorldPtr world)
{
	setWorld(world);
	mController = std::unique_ptr<Controller>(new Controller(mWorld->getSkeleton("mCartPole")));
}

void MyWindow::timeStepping() 
{
	SimWindow::timeStepping();
}

void MyWindow::drawSkels() 
{
//	glEnable(GL_LIGHTING);
//	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	SimWindow::drawSkels();
}

void MyWindow::keyboard(unsigned char _key, int _x, int _y) {
	switch(_key)
	{
		case 'a':
		mWorld->getSkeleton("mCartPole")->getDof(0)->setForce(1);
		break;

		case 'd':
		mWorld->getSkeleton("mCartPole")->getDof(0)->setForce(-1);
		break;

		case 'r':
		mWorld->getSkeleton("mCartPole")->getDof(0)->setPosition(0);
		break;

		default:
		SimWindow::keyboard(_key, _x, _y);
		break;
	}
}
