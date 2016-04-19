/*************************************************************************
    > File Name: MyWindow.h
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Tue Apr  5 10:09:39 2016
 ************************************************************************/

#ifndef MYWINDOW_H
#define MYWINDOW_H

#include "dart/dart.h"
#include "Controller.h"

namespace nonlinear{
	
using namespace dart::common;
using namespace dart::utils;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;
using namespace dart::gui;

class MyWindow:public dart::gui::SimWindow
{
public:
	MyWindow(WorldPtr world);
	void MyFlush();
	void drawCOM(Eigen::Vector3d mCOM);
	void drawSkels() override;
	void timeStepping() override;
	void keyboard(unsigned char key, int x, int y) override;
  std::unique_ptr<Controller> mController;
};
				 
} // namespace nonlinear
#endif
