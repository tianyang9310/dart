/*************************************************************************
    > File Name: MyWindow.cpp
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Tue Apr  5 10:09:33 2016
 ************************************************************************/

#include "MyWindow.h"

namespace nonlinear{

MyWindow::MyWindow(WorldPtr world):SimWindow()
{
	setWorld(world);	

	mController =  std::unique_ptr<Controller>(new Controller(mWorld->getSkeleton("biped")));
}

void MyWindow::timeStepping() 
{
	mController->update(mWorld->getTime());
	SimWindow::timeStepping();
}

void MyWindow::drawCOM(Eigen::Vector3d mCOM)
{
	// ----------------------------------------------------------------------------
	//				draw center of mass
	double mCOM_x = mCOM(0);
	double mCOM_y = mCOM(1);
	double mCOM_z = mCOM(2);

	GLUquadricObj *mC;
	mC = gluNewQuadric();
    gluQuadricDrawStyle(mC, GLU_FILL);
	gluQuadricNormals(mC, GLU_SMOOTH);

	glPushMatrix();
	glTranslated(mCOM_x, mCOM_y, mCOM_z);
	gluSphere(mC, 0.005, 100,20);
	glPopMatrix();

	gluDeleteQuadric(mC);
	// ----------------------------------------------------------------------------
}

void MyWindow::drawSkels()
{
	drawCOM(mWorld->getSkeleton("biped")->getCOM());
	for (size_t i = 0; i<mWorld->getSkeleton("biped")->getNumBodyNodes()-2; i++)
	{
		drawCOM(mWorld->getSkeleton("biped")->getBodyNode(i)->getCOM());
	}

	glEnable(GL_LIGHTING);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	SimWindow::drawSkels();
}


void MyWindow::keyboard(unsigned char key, int x, int y)
{
	SimWindow::keyboard(key, x, y); // ' ', 'p', '[', ']', 'v', 's', ',', '.', 'c', 'ESC'
}


} // namespace nonlinear
