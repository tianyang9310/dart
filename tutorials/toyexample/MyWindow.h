/*************************************************************************
    > File Name: MyWindow.h
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Wed 16 Mar 2016 11:45:41 AM EDT
 ************************************************************************/

#ifndef MYWINDOW_H
#define MYWINDOW_H

#include "dart/dart.h"
#include "Controller.h"

namespace toyexample{
	
using namespace dart::common;
using namespace dart::utils;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;
using namespace dart::gui;

class MyWindow : public dart::gui::SimWindow
{
public:
	MyWindow(WorldPtr world);
	double MyControlPBP();
	bool simCube(float *state, float ctrlAcc, float *nextState, double &pos_dof0, WorldPtr mSubWorld, Controller* mSubController);
	void timeStepping() override;
protected:
	std::unique_ptr<Controller> mController;
	dart::collision::CollisionDetector* detector;
};
				 
} // namespace toyexample
#endif
