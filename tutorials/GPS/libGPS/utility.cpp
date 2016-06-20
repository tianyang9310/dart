#include "utility.h"

namespace GPS_NSpace
{

double GaussianSampler(double mu, double sigma)
{
//  mu is mean
//  sigma is standard deviation

	unsigned seed = chrono::system_clock::now().time_since_epoch().count();
	default_random_engine generator(seed);

	normal_distribution<double> distribution(mu, sigma);

	return distribution(generator);
}

}
