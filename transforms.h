#ifndef _TRANSFORMS_H_
#define _TRANSFORMS_H_

void transforms_park(float alpha, float beta, float theta, float *d, float *q);
void transforms_inverse_park(float d, float q, float theta, float *alpha, float *beta);
void transforms_clarke(float a, float b, float c, float *alpha, float *beta);
void transforms_inverse_clarke(float alpha, float beta, float *a, float *b, float *c);

#endif