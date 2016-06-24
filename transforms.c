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
    *beta = (0.577 * a) + (1.154 * b);
}

void transforms_inverse_clarke(volatile float alpha, volatile float beta, volatile float *a, volatile float *b, volatile float *c)
{
    *a = alpha;
    *b = -alpha / 2.0 + (0.866 * beta);
    *c = -alpha / 2.0 - (0.866 * beta);
    // Multiply by 1/sqrt(3) because phase voltages have amplitude 1/sqrt(3) of bus voltage
    *a = (0.577 * (*a));
    *b = (0.577 * (*b));
    *c = (0.577 * (*c));
    //*a = (2 * alpha) / 3;
    //*b = -alpha / 3 + (577 * beta) / factor;
    //*c = -alpha / 3 - (577 * beta) / factor;
}