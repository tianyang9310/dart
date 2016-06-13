#ifndef MYWINDOW_H
#define MYWINDOW_H

#include "dart/dart.h"
#include "Controller.h"


using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace dart::common;
using namespace dart::math;
using namespace dart::gui;

class MyWindow : public dart::gui::SimWindow {
public:
	MyWindow(WorldPtr world);
	MyWindow();
	
	void timeStepping() override;
	
	void drawSkels() override;
	
	void keyboard(unsigned char _key, int _x, int _y) override;

	void mPointer_Debug();

	void mDofStat();

	std::unique_ptr<Controller> mController;
	WorldPtr mSnapshot;
	int mDDP_iter;
	Eigen::MatrixXd x_fly;
};

#endif  
