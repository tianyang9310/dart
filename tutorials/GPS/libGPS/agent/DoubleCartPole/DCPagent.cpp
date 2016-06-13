#include "DCPagent.h"

namespace GPS_NSpace
{

extern hyperparameters mHyperparameters;

DCPagent::DCPagent():agent(true)
{
	rendering = true;
	mHyperparameters.tupleBool.insert(pair<string,bool>("rendering",rendering));

	// setup_conditions
	// ('x0', 'x0var', 'pos_body_idx', 'pos_body_offset', 'noisy_body_idx', 'noisy_body_var') populate to vector as vector
}

void DCPagent::sample()
{
	cout<<"DCP_agent sample"<<endl;
}

void DCPagent::get_samples()
{
	cout<<"DCP_agent get samples"<<endl;
}



}
