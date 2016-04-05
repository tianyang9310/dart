/*************************************************************************
    > File Name: Controller.cpp
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Tue Apr  5 10:09:54 2016
 ************************************************************************/


#include "Controller.h"

namespace nonlinear{


Controller::Controller(const SkeletonPtr& biped):mBiped(biped),mPreOffset(0.0),mSpeed(0.0)
{

}

void Controller::setTargetPositions(const Eigen::VectorXd& pose)
{
	mTargetPositions = pose;
}

} //namespace nonlinear

