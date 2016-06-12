#ifndef AGENT_H
#define AGENT_H

#include <iostream>
#include <string>

namespace GPS_NSpace
{

using namespace std;

class agent
{
public:
	agent();
	virtual void sample();
	virtual void get_samples();
// --------------------------------------------------
//  Agent config
	int dH;
	// auto noise_body_idx;
	// auto noise_body_var;
	// auto pos_body_idx;
	// auto pos_body_offset;
	bool smooth_noise;
	bool smooth_noise_renormalizae;
	float smooth_noise_var;
	int x0var;

};



}


#endif
