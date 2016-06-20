#include "GPS.h"

namespace GPS_NSpace
{

GPS::GPS(int _T, int _numDDPIters, int _conditions, int _numSamplesPerCond, function<VectorXd(const VectorXd, const VectorXd)> _StepDynamics):
	T(_T),
	numDDPIters(_numDDPIters),
	conditions(_conditions),
	StepDynamics(_StepDynamics),
	x0Bundle(_conditions),
	DDPBundle(_conditions),
	DDPPolicyBundle(_conditions)
{
	DDPIter		= 0;
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
//  generate traj samples from mixture of DDP policies

	auto result = trajSampleGenerator(x0Bundle[0],DDPPolicyBundle[0].first,DDPPolicyBundle[0].second);
	cout<<result->x<<endl;
	cin.get();
//  initilization of theta_star

}

unique_ptr<sample> GPS::trajSampleGenerator(VectorXd _x0, vector<function<VectorXd(VectorXd)>> _gx, vector<MatrixXd> _Quu_inv)
{
	unique_ptr<sample> sample_ptr = unique_ptr<sample>(new sample());	

	sample_ptr->x.resize(_x0.rows(),T);
	sample_ptr->u.resize(_Quu_inv[0].rows(),T);

	sample_ptr->x.col(0)=_x0;
	for (int i=0; i<T-1; i++)
	{
		sample_ptr->u.col(i) = GaussianSampler(_gx[i](sample_ptr->x.col(i)), _Quu_inv[i]);
		sample_ptr->x.col(i+1) = StepDynamics(sample_ptr->x.col(i),sample_ptr->u.col(i));
	}

	sample_ptr->u.col(T-1) = VectorXd::Constant(sample_ptr->u.col(0).size(),std::nan("0"));

	return sample_ptr;
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
