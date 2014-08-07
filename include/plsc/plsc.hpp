#ifndef _PROBABILISTIC_LEAST_SQAURE_CLASSIFICATION_HPP_
#define _PROBABILISTIC_LEAST_SQAURE_CLASSIFICATION_HPP_

#include <cmath>

namespace GPMap {
	
// constants
const float a1(0.254829592f);
const float a2(-0.284496736f);
const float a3(1.421413741f);
const float a4(-1.453152027f);
const float a5(1.061405429f);
const float p(0.3275911f);
const float sqrt2(sqrt(2.f));

inline float normcdf(float x)
{

	// Save the sign of x
	float sign = 1.f;
	if (x < 0) sign = -1.f;
	x = fabs(x)/sqrt2;

	// A&S formula 7.1.26
	const float t = 1.f/(1.f + p*x);
	const float y = 1.f - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*exp(-x*x);

	return 0.5f * (1.f + sign*y);
}

const float PLSC_mean(0.05f);
const float PLSC_variance(0.0001f);
inline float PLSC(const float mean, const float variance)
{
	return normcdf((PLSC_mean - mean) / sqrt(variance + PLSC_variance));
}

}

#endif
