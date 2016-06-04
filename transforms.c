#include "transforms.h"

void transforms_park(int alpha, int beta, int theta, int *d, int *q)
{
    *d = (alpha * Util::Sin((theta + 90) % 360) + beta * Util::Sin(theta % 360)) / factor;
    *q = (beta * Util::Sin((theta + 90) % 360) - alpha * Util::Sin(theta % 360)) / factor;
}

void transforms_inverse_park(int d, int q, int theta, int *alpha, int *beta)
{
    *alpha = (d * Util::Sin((theta + 90) % 360) - q * Util::Sin(theta % 360)) / factor;
    *beta = (d * Util::Sin(theta % 360) + q * Util::Sin((theta + 90) % 360)) / factor;
}

void transforms_clarke(int a, int b, int c, int *alpha, int *beta)
{
    *alpha = a;
    *beta = (577 * a) / factor + (1154 * b) / factor;
}

void transforms_inverse_clarke(int alpha, int beta, int *a, int *b, int *c)
{
    *a = alpha;
    *b = -alpha / 2 + (866 * beta) / factor;
    *c = -alpha / 2 - (866 * beta) / factor;
    // Multiply by 1/sqrt(3) because phase voltages have amplitude 1/sqrt(3) of bus voltage
    *a = (577 * (*a)) / factor;
    *b = (577 * (*b)) / factor;
    *c = (577 * (*c)) / factor;
    //*a = (2 * alpha) / 3;
    //*b = -alpha / 3 + (577 * beta) / factor;
    //*c = -alpha / 3 - (577 * beta) / factor;
}