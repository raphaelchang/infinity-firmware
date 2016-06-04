#include "zsm.h"

void zsm_sinusoidal(int *a, int *b, int *c)
{
    *a = (866 * (*a)) / factor + factor / 2;
    *b = (866 * (*b)) / factor + factor / 2;
    *c = (866 * (*c)) / factor + factor / 2;
}

void zsm_midpoint_clamp(int *a, int *b, int *c)
{
    int shift = (factor - (min(min(*a, *b), *c) + max(max(*a, *b), *c))) / 2;
    *a += shift;
    *b += shift;
    *c += shift;
}

void zsm_top_clamp(int *a, int *b, int *c)
{
    int shift = factor - max(max(*a, *b), *c);
    *a += shift;
    *b += shift;
    *c += shift;
}

void zsm_bottom_clamp(int *a, int *b, int *c)
{
    int shift = min(min(*a, *b), *c);
    *a -= shift;
    *b -= shift;
    *c -= shift;
}

void zsm_top_bottom_clamp(int *a, int *b, int *c)
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