#include "GPS.h"

namespace GPS_NSpace
{
void printDict(PyObject* obj);   

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
    PyObject* pArgs = PyTuple_New(4);
    PyTuple_SetItem(pArgs,0, PyInt_FromLong(x_dim));
    PyTuple_SetItem(pArgs,1, PyInt_FromLong(u_dim));
    PyTuple_SetItem(pArgs,2, PyInt_FromLong(T-1));
    int m = numSamplesPerCond * conditions;
    PyTuple_SetItem(pArgs,3, PyInt_FromLong(m));
    pInstancePolicyOptCaffe = PyInstance_New(pClassPolicyOptCaffe,pArgs,NULL);
    pInstanceCaffePolicy    = PyObject_GetAttrString(pInstancePolicyOptCaffe,"policy");

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

    InitDDPPolicy();
    InitNNPolicy();
}

void GPS::InitDDPPolicy()
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

        DDPPolicyBundle[_cond]=make_pair(DDPBundle[_cond]->gx, DDPBundle[_cond]->Quu_inv);
    }
}

void GPS::InitNNPolicy()
{
    int m = numSamplesPerCond * conditions;

//  generate traj samples from mixture of DDP policies
//  Linear combination of mutually independent normal random vectors
    vector<shared_ptr<sample>> trajSamples4NNpretrain;
    trajSamples4NNpretrain = trajSampleGeneratorFromDDP(m);

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

    PyObject_CallMethod(pInstancePolicyOptCaffe,"printFoo",NULL);
    PyObject_CallMethod(pInstancePolicyOptCaffe,"pretrain",NULL);
    PyObject_CallMethod(pInstancePolicyOptCaffe,"printFoo",NULL);

    cout<<"@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl;
    cout<<"@ Initialize Neural Net @@"<<endl;
    cout<<"@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl;

    // free pointer
    
    cin.get();

//  initilization of theta_star

}


vector<shared_ptr<sample>> GPS::trajSampleGeneratorFromDDP(int numSamples)
{
    vector<shared_ptr<sample>> sampleLists(numSamples);

    for_each(sampleLists.begin(),sampleLists.end(),
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
                    MatrixXd __Quu_inv;
                    __Quu_inv.setZero(u_dim,u_dim);
                    MatrixXd uEye;
                    uEye.setIdentity(u_dim,u_dim);
                    for (int _cond=0; _cond<conditions; _cond++)
                    {
                        __ut = __ut + (DDPPolicyBundle[_cond].first)[i](SampleEntry->x.col(i));
                        __Quu_inv = __Quu_inv + (1 / double(conditions) * uEye) * 
                                                ((DDPPolicyBundle[_cond].second)[i]) * 
                                                (1 / double(conditions) * uEye).transpose();
                    }
                    __ut = 1 / double(conditions) * uEye * __ut;
                    SampleEntry->u.col(i) = GaussianSampler(__ut, __Quu_inv);
                    SampleEntry->x.col(i+1) = StepDynamics(SampleEntry->x.col(i),SampleEntry->u.col(i));
                    SampleEntry->Quu_inv[i] = __Quu_inv;
                }
                SampleEntry->u.col(T-1) = VectorXd::Constant(SampleEntry->u.col(0).size(),std::nan("0"));
            });
    return sampleLists;
}

// -----------------------------------------
inline void GPS::GaussianSamplerDebug()
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


}
