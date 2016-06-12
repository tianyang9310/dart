#include "agent.h"

namespace GPS_NSpace
{


agent::agent()
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
