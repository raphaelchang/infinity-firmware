#ifndef _UTILS_H_
#define _UTILS_H_

#include "ch.h"
#include "hal.h"

void utils_sincos(float angle, float *sin, float *cos);
bool utils_saturate_vector_2d(float *x, float *y, float max);

#endif
