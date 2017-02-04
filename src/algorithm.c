/*
 * algorithm.c
 *
 *  Created on: 11 lip 2016
 *      Author: kormo
 */
#include "algorithm.h"
#include "resources.h"
#include <math.h>

/*
Zak³adaj¹c, ¿e mamy wartoœæ x z przedzia³u [x_min, x_max]
i chcemy j¹ przenieœæ do [y_min, y_max], mo¿na skorzystaæ ze wzoru:

*/
int rangeScaleLinear
(uint32_t x,
		uint32_t x_min,
		uint32_t x_max,
		uint32_t y_min,
		uint32_t y_max)

{
	return (uint32_t) y_min+(x-x_min)*(y_max-y_min)/(x_max-x_min);
};

// f(v)= f0*2^ (v-v0/v0)

double rangeScaleVoltPerOclave
(double v,
		double v0,
		double f0
		)

{
	double f;
	double power;
	int pint;
	uint16_t powindex;
	double pfrac;
	double power2;
	f=0.0;

	power = ((v-v0)/v0);

	pint=(int)floor(power);

	pfrac=(double) (power-pint);

	powindex = (int)(pfrac*1000.0);
	power2=PowerOf2_16bit[powindex];

	//f=f0 * pow(2.0,power);
	//PowerOf2_16bit

	// moving multiply by 2^(pint-1) -> 2<<(pint-1)
	// zero
	if (pint==0)
	{

		f=(double)(f0*PowerOf2_16bit[powindex]);
	}

	// positive power
	if (pint>0)
	{
		f=(double)(f0*power2)*(2<<(pint-1));
	}

	// negative power, division by abs(pint-1)
	if (pint<0)
	{
		f=(double)(f0*PowerOf2_16bit[powindex])/(2<<-(pint+1));

	}


	return f;

};
