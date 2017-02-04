#ifndef ALGORITHM_H_
#define ALGORITHM_H_

#include <stdint.h>

extern int rangeScaleLinear
(uint32_t x,
		uint32_t x_min,
		uint32_t x_max,
		uint32_t y_min,
		uint32_t y_max);


extern double rangeScaleVoltPerOclave
(double v,
		double v0,
		double f0
		);

#endif /* ALGORITHM_H_ */
