#include "GPS.h"

namespace GPS_NSpace
{

GPS::GPS(int _numDDPIters, int _conditions, int T):
	T(T),
	numDDPIters(_numDDPIters),
	conditions(_conditions),
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

		DDPPolicyBundle[_cond]=make_pair(DDPBundle[_cond]->gx, DDPBundle[_cond]->Quu_neg_inv);
	}
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
