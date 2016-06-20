#ifndef GPS_H
#define GPS_H

#include <memory>
#include <functional>
#include "utility.h"
#include "../../DDP/libDDP/DDP.h"

namespace GPS_NSpace
{

using namespace std;
using namespace DDP_NSpace;

class GPS
{
public:
	GPS(int _numDDPIters, int _conditions, int T);
	void run();
	void initialDDPPolicy();
	


// --------------------
//	built-in function for debugging
	void GaussianSamplerDebug();
// --------------------

	int T;
	shared_ptr<DDP> mDDP;
	int numDDPIters;
	int DDPIter;
	int conditions;
	vector<Vector4d> x0Bundle;
	vector<shared_ptr<DDP>> DDPBundle;
	vector<pair<vector<function<VectorXd(VectorXd)>>,vector<MatrixXd>>> DDPPolicyBundle;

};

}

#endif
