#ifndef AGENT_H
#define AGENT_H

#include <iostream>
#include <string>
#include <vector>

namespace GPS_NSpace
{

using namespace std;


class mySample;

class agent
{
public:
	agent();
	agent(bool placeholders);
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

// --------------------------------------------------
//  Agent member var
	vector<mySample> _samples;
	int T;
};


class mySample
{
public:
	mySample();
	int T;
	agent mAgent;
};


}


#endif
