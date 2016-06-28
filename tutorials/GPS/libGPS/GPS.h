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
// --------------------
//  member function
	GPS(int _T, int _x_dim, int _u_dim, int _numDDPIters, int _conditions, int _numSamplesPerCond, function<VectorXd(const VectorXd, const VectorXd)> _StepDynamics);
    ~GPS();
	void run();
	void InitDDPPolicy();
	void InitNNPolicy();
    void BuildInitSamples();
    vector<shared_ptr<sample>> trajSampleGeneratorFromDDP(int numSamples, int DDPIdx);
	vector<shared_ptr<sample>> trajSampleGeneratorFromDDPMix(int numSamples);
    vector<shared_ptr<sample>> trajSampleGeneratorFromNN(int numSamples);
// --------------------

// --------------------
//  member variable
	int T;
    const int x_dim;
    const int u_dim;
	int numDDPIters;
	int DDPIter;
	int conditions;
	int numSamplesPerCond;
    int GPS_iterations;
	function<VectorXd(const VectorXd, const VectorXd)> StepDynamics;
	vector<VectorXd> x0Bundle;
	vector<shared_ptr<DDP>> DDPBundle;
	vector<pair<vector<function<VectorXd(VectorXd)>>,vector<MatrixXd>>> DDPPolicyBundle;
    vector<shared_ptr<sample>> GPSSampleLists;
// --------------------

// --------------------
//	built-in function for debugging
	void GaussianSamplerDebug();
// --------------------

// --------------------
//  python wrapper
    PyObject *pInstancePolicyOptCaffe;
    PyObject *pInstanceCaffePolicy;
    void InitPolicyOptCaffe();

    void write4numpy_X(vector<shared_ptr<sample>> data, const std::string name);
    void write4numpy_U(vector<shared_ptr<sample>> data, const std::string name);
    void write4numpy_Quu_inv(vector<shared_ptr<sample>> data, const std::string name);
// --------------------
};

}

#endif
