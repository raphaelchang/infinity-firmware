#ifndef _ZSM_H_
#define _ZSM_H_

#include "datatypes.h"

void zsm_sinusoidal(volatile float *a, volatile float *b, volatile float *c);
void zsm_midpoint_clamp(volatile float *a, volatile float *b, volatile float *c);
void zsm_top_clamp(volatile float *a, volatile float *b, volatile float *c);
void zsm_bottom_clamp(volatile float *a, volatile float *b, volatile float *c);
void zsm_top_bottom_clamp(volatile float *a, volatile float *b, volatile float *c);

#endif