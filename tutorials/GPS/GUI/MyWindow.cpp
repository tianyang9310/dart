#include "MyWindow.h"

MyWindow::MyWindow(WorldPtr world)
{
	setWorld(world);
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
		case 'x':
		
		break;

		default:
		SimWindow::keyboard(_key, _x, _y);
		break;
	}
}
