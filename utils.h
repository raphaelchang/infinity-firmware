#ifndef _UTILS_H_
#define _UTILS_H_

#define ONE_BY_SQRT3            (0.57735026919)
#define TWO_BY_SQRT3            (2.0f * 0.57735026919)
#define SQRT3_BY_2              (0.86602540378)
#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

#include "ch.h"
#include "hal.h"

void utils_sincos(float angle, float *sin, float *cos);
bool utils_saturate_vector_2d(float *x, float *y, float max);
void utils_sys_lock_cnt(void);
void utils_sys_unlock_cnt(void);
void utils_norm_angle_rad(float *angle);

#endif
