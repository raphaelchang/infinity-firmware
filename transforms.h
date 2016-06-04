#ifndef _TRANSFORMS_H_
#define _TRANSFORMS_H_

void transforms_park(int alpha, int beta, int theta, int *d, int *q);
void transforms_inverse_park(int d, int q, int theta, int *alpha, int *beta);
void transforms_clarke(int a, int b, int c, int *alpha, int *beta);
void transforms_inverse_clarke(int alpha, int beta, int *a, int *b, int *c);

#endif