#ifndef MYWINDOW_H
#define MYWINDOW_H

#include <Python.h>
#include "dart/dart.h"
#include "../CartPole/CartPoleUtility.h"
#include "../libGPS/GPS.h"

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace dart::common;
using namespace dart::math;
using namespace dart::gui;
using namespace GPS_NSpace;
using namespace std;
using namespace CartPoleUtility;

class MyWindow : public dart::gui::SimWindow {
public:
	MyWindow(WorldPtr world, unique_ptr<GPS> _mGPS);
	
	void timeStepping() override;
	
	void drawSkels() override;
	
	void keyboard(unsigned char _key, int _x, int _y) override;

    void mReset();

    void mReset_wo_fine();

// ------------------
	unique_ptr<GPS> mGPS;
    int idxDDP;
	WorldPtr mSnapShot;
    double MaxJointPos;
    double CurJointPos;

};

#endif  
