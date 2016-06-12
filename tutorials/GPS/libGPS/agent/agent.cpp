#include "agent.h"

namespace GPS_NSpace
{

mySample::mySample()
{
	T = 10;
}

agent::agent(hyperparameters& mHyperparameters):mHyperparameters(mHyperparameters),_samples(mHyperparameters.conditions)
{
	cout<<"agent constructor "<<endl;

	dH	= 0;
	// auto noise_body_idx;
	// auto noise_body_var;
	// auto pos_body_idx;
	// auto pos_body_offset;
	smooth_noise = true;
	smooth_noise_renormalizae = true;
	smooth_noise_var = 2.0;
	x0var = 0;

	// update hyperparameters with above var
	mHyperparameters.tupleInt.insert(pair<string,int>("dH",dH));
	// mHyperparameters.tupleBool.insert(pair<string,bool>("rendering",rendering));
	// mHyperparameters.tupleBool.insert(pair<string,bool>("rendering",rendering));

}

void agent::sample()
{
	cout<<"NotImplementationError"<<endl; 
	throw 0;
}

void agent::get_samples()
{
	cout<<"NotImplementationError"<<endl; 
	throw 0;
}


}
