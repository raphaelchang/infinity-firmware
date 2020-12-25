#ifndef _HW_CONF_H_
#define _HW_CONF_H_

#include "fw_conf.h"

#ifdef INFINITY_3_0
#include "hw_conf_v3.0.h"
#elif defined INFINITY_4_0
#include "hw_conf_v4.0.h"
#elif defined INFINITY_4_1
#include "hw_conf_v4.1.h"
#else
#error "Hardware version not defined!"
#endif

#endif
