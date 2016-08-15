#ifndef _TRANSFORMS_H_
#define _TRANSFORMS_H_

#define ONE_BY_SQRT3            (0.57735026919)
#define TWO_BY_SQRT3            (2.0f * 0.57735026919)
#define SQRT3_BY_2              (0.86602540378)

void transforms_park(volatile float alpha, volatile float beta, volatile float theta, volatile float *d, volatile float *q);
void transforms_inverse_park(volatile float d, volatile float q, volatile float theta, volatile float *alpha, volatile float *beta);
void transforms_clarke(volatile float a, volatile float b, volatile float c, volatile float *alpha, volatile float *beta);
void transforms_inverse_clarke(volatile float alpha, volatile float beta, volatile float *a, volatile float *b, volatile float *c);

#endif
