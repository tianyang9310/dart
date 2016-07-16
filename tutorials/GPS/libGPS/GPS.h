#ifndef GPS_H
#define GPS_H

#include <algorithm>
#include <iterator>
#include <iostream>
#include <memory>
#include <functional>
#include <random>
#include <chrono>
#include <Python.h>
#include <string>
#include <math.h>
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
    virtual ~GPS();
	void run();
    void rund();
    void DDPdemonstration();
	void InitDDPPolicy();
	void InitNNPolicy();
    void BuildInitSamples();
    void FineTunePolicy();
    void RetrieveLoss_wo(bool previous);
    void replacetheta();
    void restoretheta();
    void appendSamplesFromThetaK(); 
    void ChooseSubSets();
    vector<shared_ptr<sample>> trajSampleGeneratorFromDDP(int numSamples, int DDPIdx);
	vector<shared_ptr<sample>> trajSampleGeneratorFromDDPMix(int numSamples);
    vector<shared_ptr<sample>> trajSampleGeneratorFromNN(int numSamples);
    vector<shared_ptr<sample>> trajSampleGeneratorFromNN_ThetaK(int numSamples);
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
    int mPhi;
    int GPS_iterations;
    double previous_lossvalue_wo;
    double current_lossvalue_wo;
	function<VectorXd(const VectorXd, const VectorXd)> StepDynamics;
	vector<VectorXd> x0Bundle;
	vector<shared_ptr<DDP>> DDPBundle;
	vector<pair<vector<function<VectorXd(VectorXd)>>,vector<MatrixXd>>> DDPPolicyBundle;
    vector<shared_ptr<sample>> GPSSampleLists;
    vector<shared_ptr<sample>> cur_GPSSampleLists;
    vector<shared_ptr<sample>> trajSamples4NNpretrain;
// --------------------
//  python wrapper
    PyObject *pInstancePolicyOptCaffe;
    PyObject *pInstanceCaffePolicy;
    PyObject *pInstancePolicyRepo;
    void InitPolicyOptCaffe();
    void EvalProb_Logq();
    void EvalProb_Logqd();
    void writeSubSampleSets2file();
    void modifymPhi();
    void restoremPhi();
    vector<PyObject*> pInstanceThetaKLists;

    void write4numpy_X(vector<shared_ptr<sample>> data, const std::string name);
    void write4numpy_U(vector<shared_ptr<sample>> data, const std::string name);
    void write4numpy_Quu_inv(vector<shared_ptr<sample>> data, const std::string name);
    void write4numpy_Logq(vector<shared_ptr<sample>> data, const std::string name);
    void writeLogqd(VectorXd Logqd, const std::string name);
// --------------------
};

}

#endif
