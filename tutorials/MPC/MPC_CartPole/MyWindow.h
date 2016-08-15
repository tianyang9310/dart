#ifndef MYWINDOW_H_
#define MYWINDOW_H_

#include "dart/dart.h"
#include "Controller.h"
#include "CartPoleUtility.h"

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace dart::common;
using namespace dart::math;
using namespace dart::gui;

class MyWindow : public dart::gui::SimWindow {
public:
	MyWindow(WorldPtr world);
    
    void inner_loop();
	
	void timeStepping() override;
	
	void drawSkels() override;
	
	void keyboard(unsigned char _key, int _x, int _y) override;

	std::unique_ptr<Controller> mController;
	WorldPtr mSnapshot;
	int mMPC_iter;
};

#endif  
