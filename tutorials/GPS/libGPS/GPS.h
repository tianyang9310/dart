#ifndef GPS_H
#define GPS_H

#include <memory>
#include <functional>
#include <random>
#include <chrono>
#include <Python.h>
#include <string>
#include "utility.h"
#include "sample.h"
#include "../../DDP/libDDP/DDP.h"

#include <typeinfo>

namespace GPS_NSpace
{

using namespace std;
using namespace DDP_NSpace;

class GPS
{
public:
	GPS(int _T, int _x_dim, int _u_dim, int _numDDPIters, int _conditions, int _numSamplesPerCond, function<VectorXd(const VectorXd, const VectorXd)> _StepDynamics);
    ~GPS();
	void run();
	void InitDDPPolicy();
	void InitNNPolicy();
	vector<shared_ptr<sample>> trajSampleGeneratorFromDDP(int numSamples);
    // shared_ptr<sample> trajSampleGeneratorFromNN();
	


// --------------------
//	built-in function for debugging
	void GaussianSamplerDebug();
// --------------------

	int T;
    const int x_dim;
    const int u_dim;
	shared_ptr<DDP> mDDP;
	int numDDPIters;
	int DDPIter;
	int conditions;
	int numSamplesPerCond;
	function<VectorXd(const VectorXd, const VectorXd)> StepDynamics;
	vector<VectorXd> x0Bundle;
	vector<shared_ptr<DDP>> DDPBundle;
	vector<pair<vector<function<VectorXd(VectorXd)>>,vector<MatrixXd>>> DDPPolicyBundle;

// --------------------
//  python wrapper
    PyObject *pInstancePolicyOptCaffe;
    PyObject *pInstanceCaffePolicy;
    void InitPolicyOptCaffe();
};

}

#endif
