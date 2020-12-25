#include "zsm.h"
#include "utils.h"

#define min(x, y) (x < y ? x : y)
#define max(x, y) (x > y ? x : y)

void zsm_sinusoidal(volatile float *a, volatile float *b, volatile float *c)
{
    *a = (SQRT3_BY_2 * (*a)) + 1.0f / 2;
    *b = (SQRT3_BY_2 * (*b)) + 1.0f / 2;
    *c = (SQRT3_BY_2 * (*c)) + 1.0f / 2;
}

void zsm_midpoint_clamp(volatile float *a, volatile float *b, volatile float *c)
{
    float shift = (1 - (min(min(*a, *b), *c) + max(max(*a, *b), *c))) / 2.0;
    *a += shift;
    *b += shift;
    *c += shift;
}

void zsm_top_clamp(volatile float *a, volatile float *b, volatile float *c)
{
    float shift = 1 - max(max(*a, *b), *c);
    *a += shift;
    *b += shift;
    *c += shift;
}

void zsm_bottom_clamp(volatile float *a, volatile float *b, volatile float *c)
{
    float shift = min(min(*a, *b), *c);
    *a -= shift;
    *b -= shift;
    *c -= shift;
}

void zsm_top_bottom_clamp(volatile float *a, volatile float *b, volatile float *c)
{
    if ((*a) * (*b) * (*c) > 0) // Two negatives, largest amplitude positive
    {
          zsm_top_clamp(a, b, c);
    }
    else // Two positives, largest amplitude negative
    {
          zsm_bottom_clamp(a, b, c);
    }
}
