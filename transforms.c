#include "transforms.h"
#include <math.h>
#include "utils.h"

void transforms_park(volatile float alpha, volatile float beta, volatile float theta, volatile float *d, volatile float *q)
{
    float sin, cos;
    utils_sincos(fmod(theta, 360), &sin, &cos);
    *d = alpha * cos + beta * sin;
    *q = beta * cos - alpha * sin;
}

void transforms_inverse_park(volatile float d, volatile float q, volatile float theta, volatile float *alpha, volatile float *beta)
{
    float sin, cos;
    utils_sincos(fmod(theta, 360), &sin, &cos);
    *alpha = d * cos - q * sin;
    *beta = d * sin + q * cos;
}

void transforms_clarke(volatile float a, volatile float b, volatile float c, volatile float *alpha, volatile float *beta)
{
    *alpha = a;
    *beta = (ONE_BY_SQRT3 * a) + (TWO_BY_SQRT3 * b);
}

void transforms_inverse_clarke(volatile float alpha, volatile float beta, volatile float *a, volatile float *b, volatile float *c)
{
    *a = alpha;
    *b = -alpha / 2.0 + (SQRT3_BY_2 * beta);
    *c = -alpha / 2.0 - (SQRT3_BY_2 * beta);
    // Multiply by 1/sqrt(3) because phase voltages have amplitude 1/sqrt(3) of bus voltage
    *a = (ONE_BY_SQRT3 * (*a));
    *b = (ONE_BY_SQRT3 * (*b));
    *c = (ONE_BY_SQRT3 * (*c));
}
