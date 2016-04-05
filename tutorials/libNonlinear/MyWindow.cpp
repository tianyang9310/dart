/*************************************************************************
    > File Name: MyWindow.cpp
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Tue Apr  5 10:09:33 2016
 ************************************************************************/

#include "MyWindow.h"

namespace nonlinear{

MyWindow::MyWindow(WorldPtr world)
{
	setWorld(world);	

	mController =  std::unique_ptr<Controller>(new Controller(mWorld->getSkeleton("biped")));
}

void MyWindow::timeStepping() 
{
	SimWindow::timeStepping();
}

void MyWindow::drawSkels()
{
	glEnable(GL_LIGHTING);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	SimWindow::drawSkels();
}


void MyWindow::keyboard(unsigned char key, int x, int y)
{
	SimWindow::keyboard(key, x, y); // ' ', 'p', '[', ']', 'v', 's', ',', '.', 'c', 'ESC'
}


} // namespace nonlinear
