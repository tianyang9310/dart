#include "GPS.h"

namespace GPS_NSpace
{
void printDict(PyObject* obj);   
void GaussianSamplerDebug();
void GaussianEvaluatorDebug();

GPS::GPS(int _T, int _x_dim, int _u_dim, int _numDDPIters, int _conditions, int _numSamplesPerCond, function<VectorXd(const VectorXd, const VectorXd)> _StepDynamics):
    T(_T),
    x_dim(_x_dim),
    u_dim(_u_dim),
    numDDPIters(_numDDPIters),
    conditions(_conditions),
    numSamplesPerCond(_numSamplesPerCond),
    StepDynamics(_StepDynamics),
    x0Bundle(_conditions),
    DDPBundle(_conditions),
    DDPPolicyBundle(_conditions)
{
    DDPIter     = 0;
    // m is a misc variable and mPhi is a unique meaningful variable
    mPhi        = (conditions+1)*4;
    GPS_iterations = 1;
    previous_lossvalue_wo = 0;
    current_lossvalue_wo = 0;

    Py_Initialize();
    PyRun_SimpleString("import sys");  
    // add src path of libgps
    PyRun_SimpleString("sys.path.append('../../../tutorials/GPS/libGPS/')");  
    // PyRun_SimpleString("print sys.path");

    InitPolicyOptCaffe();
}

void GPS::InitPolicyOptCaffe()
{
    if (!Py_IsInitialized())  
    {
        cout<<"Python Interpreter not Initialized!!!"<<endl;
    }
    PyObject* pModule = PyImport_Import(PyString_FromString("Policy_Opt_Caffe"));
    if (!pModule) {  
        cout<<"Cant open python file!"<<endl;  
    }  
    PyObject* pDict = PyModule_GetDict(pModule);  
    if (!pDict) {  
        cout<<"Cant find dictionary!!!"<<endl;  
    }  
    printDict(pDict); 
    PyObject *pClassPolicyOptCaffe = PyDict_GetItemString(pDict, "PolicyOptCaffe");
    if (!pClassPolicyOptCaffe) {  
        cout<<"Cant find PolicyOptCaffe class!!!"<<endl;  
    }  
    /*
        x_dim:
        u_dim:
        T:     time horizon of one episode (normally T-1, since no control at the very end step)
        N:     used in pretrain method. 
               pretrain is called after ReadX, ReadU and ReadQuu_inv. Therefore the total number of data pairs is self.T * N
               now N is the number of batches per epoch. Considering caffe_iterations is 50, so the total epochs should be 50/N
        mPhi:  used in finetrain method.
               mPhi is the total number of samples in optimizing Phi
        batch_size: change from 25 to T-1
    */
    PyObject* pArgs = PyTuple_New(5);
    PyTuple_SetItem(pArgs,0, PyInt_FromLong(x_dim));
    PyTuple_SetItem(pArgs,1, PyInt_FromLong(u_dim));
    PyTuple_SetItem(pArgs,2, PyInt_FromLong(T-1));
    int m = numSamplesPerCond * conditions;
    PyTuple_SetItem(pArgs,3, PyInt_FromLong(m));
    PyTuple_SetItem(pArgs,4, PyInt_FromLong(mPhi));
    pInstancePolicyOptCaffe = PyInstance_New(pClassPolicyOptCaffe,pArgs,NULL);
    pInstanceCaffePolicy    = PyObject_GetAttrString(pInstancePolicyOptCaffe,"policy");
    
    PyObject_CallMethod(pInstancePolicyOptCaffe,"setWr",NULL);

    // testing whether reference and how to call method of class
//    auto foo_before = PyObject_GetAttrString(pInstanceCaffePolicy, "foo"); 
//    auto foo_before_c = PyInt_AsLong(foo_before);
//    cout<<"before is "<<foo_before_c<<endl;
//    PyObject_CallMethod(pInstancePolicyOptCaffe,"setFoo",NULL);
//    auto foo_after = PyObject_GetAttrString(pInstanceCaffePolicy, "foo"); 
//    auto foo_after_c = PyInt_AsLong(foo_after);
//    cout<<"after is "<<foo_after_c<<endl;
    // -------------------------

    // free pointer
    Py_DECREF(pModule);
    Py_DECREF(pDict);
    Py_DECREF(pClassPolicyOptCaffe);
    Py_DECREF(pArgs);

}

GPS::~GPS()
{
    Py_Finalize();
}

void GPS::run()
{
    // GaussianSamplerDebug();
    // GaussianEvaluatorDebug();

    InitDDPPolicy();
    cout<<"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl;
    cout<<"@@@@@@ Initialize DDP Bundles @@@@@@@@"<<endl;
    cout<<"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl<<endl;
    cout<<"Press any key to continue..."<<endl;
    cin.get();

    InitNNPolicy();
    cout<<"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl;
    cout<<"@@@@@@ Initialize Neural Net @@@@@@@@@"<<endl;
    cout<<"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl<<endl;
    cout<<"Press any key to continue..."<<endl;
    cin.get();

    BuildInitSamples();
    cout<<"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl;
    cout<<"@@@@@@ Build Initiali Sample Lists@@@@"<<endl;
    cout<<"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl<<endl;
    cout<<"Press any key to continue..."<<endl;
    cin.get();

    for (int _GPS_iter=0; _GPS_iter<GPS_iterations; _GPS_iter++)
    {
        // shuffle and choose sub sample sets Sk

        // keep track of previous_lossvalue_wo
        RetrieveLoss_wo(true);
        // Optimize theta_k w.r.t. Phi
        FineTunePolicy();

        // generate samples from theta_k
        // Optionally generate adaptive guiding samples
        
        // Evaluate eq(2) to see whether replace theta_k or increase wr
        // Here this evaluation can be retrieved directly from python interface
        RetrieveLoss_wo(false);

        if (current_lossvalue_wo <= previous_lossvalue_wo)
        {
            replacetheta();
        }
        else
        {
            restoretheta();
        }
        
        //
        // return the best policy
        
        cout<<"& & & & & & & & & & & & & & &"<<endl;
        cout<<"& & One iteration of Run  & &"<<endl;
        cout<<"& Press any key to continue &"<<endl;
        cout<<"& & & & & & & & & & & & & & &"<<endl;
        cin.get();
    }
}

void GPS::DDPdemonstration()
{
    for (int _cond=0; _cond<conditions; _cond++)
    {
        for(int i=0; i<numDDPIters; i++)
        {
            DDPBundle[_cond]->trajopt();
            cout<<"########################################"<<endl;
            cout<<"  "<<_cond<<" DDP finish "<<(DDPIter)%numDDPIters+1+numDDPIters*(DDPIter/(numDDPIters*conditions))<<"th iteration"<<endl;
            DDPIter++;
            cout<<"########################################"<<endl;
        }
        
        cout<<"[BEFORE] "<<_cond<<" DDP mu is "<<DDPBundle[_cond]->mu<<endl;
        DDPBundle[_cond]->setMu();
        cout<<"[AFTER]"<<_cond<<" DDP mu is "<<DDPBundle[_cond]->mu<<endl;

    }
}

void GPS::InitDDPPolicy()
{
    DDPdemonstration();
    for (int _cond=0; _cond<conditions; _cond++)
    {
        DDPBundle[_cond]->trajopt();

        DDPPolicyBundle[_cond]=make_pair(DDPBundle[_cond]->gx, DDPBundle[_cond]->Quu_inv);
    }
}

void GPS::InitNNPolicy()
{
    int m = numSamplesPerCond * conditions;

//  generate traj samples from mixture of DDP policies
//  Linear combination of mutually independent normal random vectors
    vector<shared_ptr<sample>> trajSamples4NNpretrain;
    trajSamples4NNpretrain = trajSampleGeneratorFromDDPMix(m);

//  transform from c++ vector to python numpy
    if (!Py_IsInitialized())  
    {
        cout<<"Python Interpreter not Initialized!!!"<<endl;
    }

    write4numpy_X(trajSamples4NNpretrain, "X");
    PyObject_CallMethod(pInstancePolicyOptCaffe,"ReadX",NULL);
    
    write4numpy_U(trajSamples4NNpretrain, "U");
    PyObject_CallMethod(pInstancePolicyOptCaffe,"ReadU",NULL);

    write4numpy_Quu_inv(trajSamples4NNpretrain, "Quu_inv");
    PyObject_CallMethod(pInstancePolicyOptCaffe,"ReadQuu_inv",NULL);

//  initialization of theta_star
    PyObject_CallMethod(pInstancePolicyOptCaffe,"printFoo",NULL);
    PyObject_CallMethod(pInstancePolicyOptCaffe,"pretrain",NULL);
    PyObject_CallMethod(pInstancePolicyOptCaffe,"printFoo",NULL);
}

void GPS::BuildInitSamples()
{
    // here no casting because mPhi is designed to be dividible
    int m = mPhi/(conditions+1);
    GPSSampleLists = trajSampleGeneratorFromNN(m);
    for (int _cond=0; _cond<conditions; _cond++)
    {
        auto tmpSampleLists = trajSampleGeneratorFromDDP(m, _cond);
        GPSSampleLists.insert(GPSSampleLists.end(), tmpSampleLists.begin(), tmpSampleLists.end());
    }
    write4numpy_X(GPSSampleLists, "SampleSets_X");
    write4numpy_U(GPSSampleLists, "SampleSets_U");
    write4numpy_Quu_inv(GPSSampleLists, "SampleSets_Quu_inv");
    EvalProb_Logq();
    write4numpy_Logq(GPSSampleLists, "SampleSets_Logq");
}


void GPS::FineTunePolicy()
{
//  transform from c++ vector to python numpy
    if (!Py_IsInitialized())  
    {
        cout<<"Python Interpreter not Initialized!!!"<<endl;
    }

    PyObject_CallMethod(pInstancePolicyOptCaffe,"ReadSampleSets_X",NULL);
    PyObject_CallMethod(pInstancePolicyOptCaffe,"ReadSampleSets_U",NULL);
    PyObject_CallMethod(pInstancePolicyOptCaffe,"ReadSampleSets_Quu_inv",NULL);
    PyObject_CallMethod(pInstancePolicyOptCaffe,"ReadSampleSets_Logq",NULL);

//  initialization of theta_star
    PyObject_CallMethod(pInstancePolicyOptCaffe,"printFoo2",NULL);
    PyObject_CallMethod(pInstancePolicyOptCaffe,"finetune",NULL);
    PyObject_CallMethod(pInstancePolicyOptCaffe,"printFoo2",NULL);
}


void GPS::RetrieveLoss_wo(bool previous)
{
    if (!Py_IsInitialized())  
    {
        cout<<"Python Interpreter not Initialized!!!"<<endl;
    }

    PyObject_CallMethod(pInstancePolicyOptCaffe,"ReadSampleSets_X",NULL);
    PyObject_CallMethod(pInstancePolicyOptCaffe,"ReadSampleSets_U",NULL);
    PyObject_CallMethod(pInstancePolicyOptCaffe,"ReadSampleSets_Quu_inv",NULL);
    PyObject_CallMethod(pInstancePolicyOptCaffe,"ReadSampleSets_Logq",NULL);

    // TODO retrieve member value of pInstancePolicyOptCaffe
    if (previous)
    {
        PyObject_CallMethod(pInstancePolicyOptCaffe,"trainnet2forward",NULL);
        previous_lossvalue_wo = PyFloat_AsDouble(PyObject_GetAttrString(pInstancePolicyOptCaffe,"lossvalue_wo"));
    }
    else
    {
        PyObject_CallMethod(pInstancePolicyOptCaffe,"trainnet2forward",NULL);
        current_lossvalue_wo = PyFloat_AsDouble(PyObject_GetAttrString(pInstancePolicyOptCaffe,"lossvalue_wo"));
    }
}


void GPS::replacetheta()
{
    // replace theta_star with theta_k
    // pass since optimization is essentially a replace precess
    
    // decrease wr
    PyObject_CallMethod(pInstancePolicyOptCaffe,"decreaseWr",NULL);
}

void GPS::restoretheta()
{
    // restore theta_star 
    if (!Py_IsInitialized())  
    {
        cout<<"Python Interpreter not Initialized!!!"<<endl;
    }
    PyObject_CallMethod(pInstancePolicyOptCaffe,"solver2copyfrompolicy",NULL);

    // increase wr
    PyObject_CallMethod(pInstancePolicyOptCaffe,"increaseWr",NULL);
}

void GPS::EvalProb_Logq()
{
//  this function computes the evaluation of trajectories in terms of mixture of probabilities.
    for_each(GPSSampleLists.begin(), GPSSampleLists.end(), 
            [=](shared_ptr<sample> &SampleEntry)
            {
                SampleEntry->Logq.setZero(T);
                for (int i=0; i<T-1; i++)
                {
                    double tmpq    = 0;
                    // condition+1
                    for (int _cond=0; _cond<conditions; _cond++)
                    {
                        tmpq += GaussianEvaluator((DDPPolicyBundle[_cond].first)[i](SampleEntry->x.col(i))(0), (DDPPolicyBundle[_cond].second)[i](0), SampleEntry->u.col(i)(0));
                    }

                    PyObject* pArgs = PyTuple_New(4);
                    PyTuple_SetItem(pArgs,0, PyFloat_FromDouble(SampleEntry->x.col(i)[0]));
                    PyTuple_SetItem(pArgs,1, PyFloat_FromDouble(SampleEntry->x.col(i)[1]));
                    PyTuple_SetItem(pArgs,2, PyFloat_FromDouble(SampleEntry->x.col(i)[2]));
                    PyTuple_SetItem(pArgs,3, PyFloat_FromDouble(SampleEntry->x.col(i)[3]));
                    PyObject* pResult =  PyObject_CallMethodObjArgs(pInstanceCaffePolicy,PyString_FromString("act"), pArgs, NULL);
                    if (! pResult)
                    {
                        cout<<"Failing to CALL act method of Caffe Policy"<<endl;
                    }
                    VectorXd __ut;
                    __ut.setZero(u_dim);
                    double u_Policy;
                    if (! PyArg_ParseTuple(pResult, "d", &u_Policy))
                    {
                        cout<<"Failing to PARSE data from act method"<<endl;
                    }
                    __ut<<u_Policy;
                    
                    MatrixXd __Quu_inv;
                    __Quu_inv.setZero(u_dim,u_dim);
                    double Quu_inv_Policy;
                    Quu_inv_Policy = PyFloat_AsDouble(PyObject_GetAttrString(pInstanceCaffePolicy,"var"));
                    __Quu_inv<<Quu_inv_Policy;
                    Py_DECREF(pArgs);
                    Py_DECREF(pResult);
                    tmpq += GaussianEvaluator(__ut(0), __Quu_inv(0), SampleEntry->u.col(i)(0));

                    tmpq = tmpq/double(conditions+1);
                    if (i==0)
                    {
                        SampleEntry->Logq(i) = log(tmpq);
                    }
                    {
                        SampleEntry->Logq(i) = SampleEntry->Logq(i-1) + log(tmpq);
                    }
                }
            });
}

void GPS::EvalConditionalProb_Logq()
{
//  this function computes the evaluation of trajectories in terms of mixture of probabilities.
/*
    for_each(GPSSampleLists.begin(), GPSSampleLists.end(), 
            [=](shared_ptr<sample> &SampleEntry)
            {
                SampleEntry->Logq.setZero(T);
                for (int i=0; i<T-1; i++)
                {
                    double tmpq    = 0;
                    // condition+1
                    for (int _cond=0; _cond<conditions; _cond++)
                    {
                        tmpq += GaussianEvaluator((DDPPolicyBundle[_cond].first)[i](SampleEntry->x.col(i))(0), (DDPPolicyBundle[_cond].second)[i](0), SampleEntry->u.col(i)(0));
                    }
                    tmpq = tmpq/double(conditions);
                    if (i==0)
                    {
                        SampleEntry->Logq(i) = log(tmpq);
                    }
                    {
                        SampleEntry->Logq(i) = SampleEntry->Logq(i-1) + log(tmpq);
                    }
                }
            });
*/
}

vector<shared_ptr<sample>> GPS::trajSampleGeneratorFromNN(int numSamples)
{
    if (!Py_IsInitialized())  
    {
        cout<<"Python Interpreter not Initialized!!!"<<endl;
    }

    vector<shared_ptr<sample>> IndSampleLists(numSamples);
    for_each(IndSampleLists.begin(),IndSampleLists.end(),
            [=](shared_ptr<sample> &SampleEntry)
            {
//  randomly settle down x0 by uniform distribution
                unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
                default_random_engine generator(seed);
                uniform_int_distribution<int> distribution(0,conditions-1);
                VectorXd _x0;
                _x0 = x0Bundle[distribution(generator)];
                SampleEntry = shared_ptr<sample>(new sample());
                 
                SampleEntry->x.resize(x_dim,T);
                SampleEntry->u.resize(u_dim,T);
                SampleEntry->Quu_inv.resize(T);
                
                SampleEntry->x.col(0)=_x0;
                for (int i=0; i<T-1; i++)
                {
                    PyObject* pArgs = PyTuple_New(4);
                    PyTuple_SetItem(pArgs,0, PyFloat_FromDouble(SampleEntry->x.col(i)[0]));
                    PyTuple_SetItem(pArgs,1, PyFloat_FromDouble(SampleEntry->x.col(i)[1]));
                    PyTuple_SetItem(pArgs,2, PyFloat_FromDouble(SampleEntry->x.col(i)[2]));
                    PyTuple_SetItem(pArgs,3, PyFloat_FromDouble(SampleEntry->x.col(i)[3]));
                    PyObject* pResult =  PyObject_CallMethodObjArgs(pInstanceCaffePolicy,PyString_FromString("act"), pArgs, NULL);
                    if (! pResult)
                    {
                        cout<<"Failing to CALL act method of Caffe Policy"<<endl;
                    }

                    VectorXd __ut;
                    __ut.setZero(u_dim);

                    double u_Policy;
                    if (! PyArg_ParseTuple(pResult, "d", &u_Policy))
                    {
                        cout<<"Failing to PARSE data from act method"<<endl;
                    }
                    __ut<<u_Policy;
                    SampleEntry->u.col(i) = __ut;
                    SampleEntry->x.col(i+1) = StepDynamics(SampleEntry->x.col(i),SampleEntry->u.col(i));
                    
                    MatrixXd __Quu_inv;
                    __Quu_inv.setZero(u_dim,u_dim);
                    double Quu_inv_Policy;
                    Quu_inv_Policy = PyFloat_AsDouble(PyObject_GetAttrString(pInstanceCaffePolicy,"var"));
                    __Quu_inv<<Quu_inv_Policy;
                    SampleEntry->Quu_inv[i] = __Quu_inv;

                    Py_DECREF(pArgs);
                    Py_DECREF(pResult);
                }
                SampleEntry->u.col(T-1) = VectorXd::Constant(SampleEntry->u.col(0).size(),std::nan("0"));
            });
    return IndSampleLists;
}

vector<shared_ptr<sample>> GPS::trajSampleGeneratorFromDDP(int numSamples, int DDPIdx)
{
    vector<shared_ptr<sample>> IndSampleLists(numSamples);

    for_each(IndSampleLists.begin(),IndSampleLists.end(),
            [=](shared_ptr<sample> &SampleEntry)
            {
                VectorXd _x0;
                _x0 = x0Bundle[DDPIdx];
                SampleEntry = shared_ptr<sample>(new sample());
                 
                SampleEntry->x.resize(x_dim,T);
                SampleEntry->u.resize(u_dim,T);
                SampleEntry->Quu_inv.resize(T);
                
                SampleEntry->x.col(0)=_x0;
                for (int i=0; i<T-1; i++)
                {
                    VectorXd __ut;
                    MatrixXd __Quu_inv;
                    __ut.setZero(u_dim);
                    __Quu_inv.setZero(u_dim,u_dim);
                    __ut = (DDPPolicyBundle[DDPIdx].first)[i](SampleEntry->x.col(i));
                    __Quu_inv = (DDPPolicyBundle[DDPIdx].second)[i];
                    SampleEntry->u.col(i) = GaussianSampler(__ut, __Quu_inv);
                    SampleEntry->x.col(i+1) = StepDynamics(SampleEntry->x.col(i),SampleEntry->u.col(i));
                    SampleEntry->Quu_inv[i] = __Quu_inv;
                }
                SampleEntry->u.col(T-1) = VectorXd::Constant(SampleEntry->u.col(0).size(),std::nan("0"));
            });
    return IndSampleLists;
}

vector<shared_ptr<sample>> GPS::trajSampleGeneratorFromDDPMix(int numSamples)
{
    vector<shared_ptr<sample>> MixSampleLists(numSamples);

    for_each(MixSampleLists.begin(),MixSampleLists.end(),
            [=](shared_ptr<sample> &SampleEntry)
            {
//  randomly settle down x0 by uniform distribution
                unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
                default_random_engine generator(seed);
                uniform_int_distribution<int> distribution(0,conditions-1);
                VectorXd _x0;
                _x0 = x0Bundle[distribution(generator)];
                SampleEntry = shared_ptr<sample>(new sample());
                 
                SampleEntry->x.resize(x_dim,T);
                SampleEntry->u.resize(u_dim,T);
                SampleEntry->Quu_inv.resize(T);
                
                SampleEntry->x.col(0)=_x0;
                for (int i=0; i<T-1; i++)
                {
                    VectorXd __ut;
                    __ut.setZero(u_dim);
                    __ut = (DDPPolicyBundle[distribution(generator)].first)[i](SampleEntry->x.col(i));
                    MatrixXd __Quu_inv;
                    __Quu_inv.setZero(u_dim,u_dim);
                    __Quu_inv = (DDPPolicyBundle[distribution(generator)].second)[i];

                    SampleEntry->u.col(i) = GaussianSampler(__ut, __Quu_inv);
                    SampleEntry->x.col(i+1) = StepDynamics(SampleEntry->x.col(i),SampleEntry->u.col(i));
                    SampleEntry->Quu_inv[i] = __Quu_inv;
                }
                SampleEntry->u.col(T-1) = VectorXd::Constant(SampleEntry->u.col(0).size(),std::nan("0"));
            });
    return MixSampleLists;
}

// -----------------------------------------
inline void GaussianSamplerDebug()
{
    const int nrolls=10000;
    const int nstars=100;       
    int p[10]={};

    VectorXd numberlog;
    numberlog.resize(nrolls);
    for (int i=0; i<nrolls; ++i) {
    double number;
    auto tmp = GaussianSampler((VectorXd(1)<<5).finished(),(MatrixXd(1,1)<<10).finished());
    number = tmp(0);
    numberlog(i)=number;
    if ((number>=0.0)&&(number<10.0)) ++p[int(number)];
    }

    for (int i=0; i<10; ++i) {
    cout << i << "-" << (i+1) << ": ";
    cout << string(p[i]*nstars/nrolls,'*') << endl;
    }

    double mean = numberlog.mean();
    cout<<"Mean is "<<mean<<endl;
    cout<<"Variance is "<< (numberlog.array()-mean).square().sum()/double(nrolls)<<endl;
    cin.get();
}

inline void GaussianEvaluatorDebug()
{
    cout<<"The probability of 5 in N(5,1) is "<<GaussianEvaluator(5,1,5)<<endl;
    cout<<"The probability of 4 in N(5,1) is "<<GaussianEvaluator(5,1,4)<<endl;
    cout<<"The probability of 3 in N(5,1) is "<<GaussianEvaluator(5,1,3)<<endl;
    cin.get();
}

void printDict(PyObject* obj) {  
    if (!PyDict_Check(obj))  
        return;  
    PyObject *k, *keys;  
    keys = PyDict_Keys(obj);  
    for (int i = 0; i < PyList_GET_SIZE(keys); i++) {  
        k = PyList_GET_ITEM(keys, i);  
        char* c_name = PyString_AsString(k);  
        cout<<c_name<<endl;
        // printf("%s/n", c_name);  
    }  
}  

void GPS::write4numpy_X(vector<shared_ptr<sample>> data, const std::string name)
{
//  remove last state, since no control in last step.
//  later may resume since loss function requires last step
    std::string name_ext = name;
    name_ext.append(".numpyout");
    std::ofstream outFile(name_ext, std::ios::out);
    if (outFile.fail())
    {
        dtmsg << "Cannot open "<<name<<" file, please check..."<<std::endl;
    }
    outFile.precision(8);
    for (size_t i =0; i<data.size(); i++)
    {
        outFile<<data[i]->x.leftCols(T-1).transpose()<<std::endl; 
    }
    outFile.close();
}

void GPS::write4numpy_U(vector<shared_ptr<sample>> data, const std::string name)
{
//  remove last state, since no control in last step.
//  later may resume since loss function requires last step
    std::string name_ext = name;
    name_ext.append(".numpyout");
    std::ofstream outFile(name_ext, std::ios::out);
    if (outFile.fail())
    {
        dtmsg << "Cannot open "<<name<<" file, please check..."<<std::endl;
    }
    outFile.precision(8);
    for (size_t i =0; i<data.size(); i++)
    {
        outFile<<data[i]->u.leftCols(T-1).transpose()<<std::endl; 
    }
    outFile.close();
}

void GPS::write4numpy_Quu_inv(vector<shared_ptr<sample>> data, const std::string name)
{
//  remove last state, since no control in last step.
//  later may resume since loss function requires last step
    std::string name_ext = name;
    name_ext.append(".numpyout");
    std::ofstream outFile(name_ext, std::ios::out);
    if (outFile.fail())
    {
        dtmsg << "Cannot open "<<name<<" file, please check..."<<std::endl;
    }
    outFile.precision(8);
    for (size_t i =0; i<data.size(); i++)
    {
        for (size_t j=0; j<data[i]->Quu_inv.size()-1; j++)
        {
            outFile<<data[i]->Quu_inv[j]<<std::endl; 
        }
    }
    outFile.close();
}

void GPS::write4numpy_Logq(vector<shared_ptr<sample>> data, const std::string name)
{
//  remove last state, since no control in last step.
//  later may resume since loss function requires last step
    std::string name_ext = name;
    name_ext.append(".numpyout");
    std::ofstream outFile(name_ext, std::ios::out);
    if (outFile.fail())
    {
        dtmsg << "Cannot open "<<name<<" file, please check..."<<std::endl;
    }
    outFile.precision(8);
    for (size_t i =0; i<data.size(); i++)
    {
        outFile<<data[i]->Logq.head(T-1)<<std::endl; 
    }
    outFile.close();
}

}
