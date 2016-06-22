#ifndef _ZSM_H_
#define _ZSM_H_

typedef enum
{
	SINUSOIDAL,
	MIDPOINT_CLAMP,
	TOP_CLAMP,
	BOTTOM_CLAMP,
	TOP_BOTTOM_CLAMP
} ZSMMode;

void zsm_sinusoidal(float *a, float *b, float *c);
void zsm_midpoint_clamp(float *a, float *b, float *c);
void zsm_top_clamp(float *a, float *b, float *c);
void zsm_bottom_clamp(float *a, float *b, float *c);
void zsm_top_bottom_clamp(float *a, float *b, float *c);

#endif