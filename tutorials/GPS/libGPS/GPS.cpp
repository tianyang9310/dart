#include "GPS.h"

namespace GPS_NSpace
{

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
}

void GPS::run()
{
    // GaussianSamplerDebug();

    initialDDPPolicy();
    initialNNPolicy();
}

void GPS::initialDDPPolicy()
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

void GPS::initialNNPolicy()
{
    int m = numSamplesPerCond * conditions;

//  generate traj samples from mixture of DDP policies
//  Linear combination of mutually independent normal random vectors
    vector<unique_ptr<sample>> trajSamples4NNpretrain(m);
    trajSamples4NNpretrain = trajSampleGeneratorFromDDP(m);

//  initilization of theta_star

}

vector<unique_ptr<sample>> GPS::trajSampleGeneratorFromDDP(int numSamples)
{
    vector<unique_ptr<sample>> sampleLists(numSamples);
    for_each(sampleLists.begin(),sampleLists.end(),
            [this](unique_ptr<sample> &SampleEntry)
            {
                SampleEntry = unique_ptr<sample>(new sample());
                 
                SampleEntry->x.resize(_x0.rows(),T);
                SampleEntry->u.resize(_Quu_inv[0].rows(),T);
                
                SampleEntry->x.col(0)=_x0;
                for (int i=0; i<T-1; i++)
                {
                    VectorXd __ut = VectorXd::Zero(u_dim);
                    MatrixXd __Quu_inv = MatrixXd::Zero(u_dim,u_dim);
                    for (int _cond=0; _cond<this->conditions; _cond++)
                    {
                        __ut = __ut + (this->DDPPolicyBundle[_cond].first)[i](SampleEntry->x.col(i));
                        __Quu_inv = __Quu_inv + (1 / double(conditions) * Matrix(double, u_dim, u_dim)::Identity()) * 
                                                ((this->DDPPolicyBundle[_cond].second)[i]) * 
                                                (1 / double(conditions) * Matrix(double, u_dim, u_dim)::Identity()).transpose();
                    }
                    __ut = 1 / double(conditions) * Matrix(double, u_dim, u_dim)::Identity() * __ut;
                    SampleEntry->u.col(i) = GaussianSampler(_gx[i](__ut, __Quu_inv);
                    SampleEntry->x.col(i+1) = StepDynamics(SampleEntry->x.col(i),SampleEntry->u.col(i));
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

}
