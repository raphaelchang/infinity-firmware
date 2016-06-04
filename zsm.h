#ifndef _ZSM_H_
#define _ZSM_H_

enum ZSMMode
{
	SINUSOIDAL,
	MIDPOINT_CLAMP,
	TOP_CLAMP,
	BOTTOM_CLAMP,
	TOP_BOTTOM_CLAMP
};

void zsm_sinusoidal(int *a, int *b, int *c);
void zsm_midpoint_clamp(int *a, int *b, int *c);
void zsm_top_clamp(int *a, int *b, int *c);
void zsm_bottom_clamp(int *a, int *b, int *c);
void zsm_top_bottom_clamp(int *a, int *b, int *c);

#endif