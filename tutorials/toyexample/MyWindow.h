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
	void drawSkels() override;
	bool simCube(float *state, float ctrlAcc, float *nextState, double &pos_dof0, double &pos_dof2, double &vel_dof2, WorldPtr mSubWorld, Controller* mSubController, int smpl_idx, int tim_idx);
	void timeStepping() override;
	void keyboard(unsigned char key, int x, int y) override;
	void moveObstacle(int obstacle_idx,float delta);
protected:
	std::unique_ptr<Controller> mController;
	dart::collision::CollisionDetector* detector;
	double mNewTimeStep;
	int N;   // nSamples
	int K;   // nTimeSteps
	Eigen::MatrixXd traj_dof0_x;  // x position
	Eigen::MatrixXd traj_dof1_y;  // y position
	float targetPos_dof0_x; 
	float targetPos_dof1_y; 
	float targetVel_dof0_x; 
	float targetVel_dof1_y; 
	float delta_targetPos_dof1_y;
};
				 
} // namespace toyexample
#endif
