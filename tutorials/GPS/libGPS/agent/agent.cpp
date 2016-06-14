#include "agent.h"
#include "../utility/utility.h"

namespace GPS_NSpace
{
extern hyperparameters mHyperparameters;

mySample::mySample()
{
	T = 10;
}

agent::agent()
{
	// placeholders doing nothing
	// should be able to remove when add mysample contructor function
}

agent::agent(bool placeholders):_samples(mHyperparameters.conditions)
{
	cout<<"agent constructor "<<endl;

// ---------------------------------------
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
	
// ---------------------------------------
	T = mHyperparameters.T;
	dU = mHyperparameters.SensorDim["action"];
	// x_data_types state_include
	// obs_data_types obs_include
	// meta_data_types meta_include
	

// ---------------------------------------
//  list of indices for each data type in stateX
	dX = mHyperparameters.SensorDim["state"];
// ---------------------------------------
//  list of indices for each data type in obs
	dU = mHyperparameters.SensorDim["observation"];
// ---------------------------------------
//  list of indices for each data type in meta data
// ---------------------------------------
//  _x_data_idx
//  _obs_data_idx
//  _meta_data_idx


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
