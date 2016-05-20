#include "MyWindow.h"

MyWindow::MyWindow()
  : SimWindow() {
}

MyWindow::~MyWindow() {
}

void MyWindow::timeStepping() {
  mWorld->step();
}

void MyWindow::drawSkels() {
  glEnable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  SimWindow::drawSkels();
}

void MyWindow::keyboard(unsigned char _key, int _x, int _y) {
      Win3D::keyboard(_key, _x, _y);
	  glutPostRedisplay();
}
