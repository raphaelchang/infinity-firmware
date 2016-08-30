#ifndef _SCOPE_H_
#define _SCOPE_H_

#include "ch.h"

#define MAX_SAMPLES (uint16_t)1024
#define NUM_CHANNELS 4

void scope_init(void);
/* Set the number of samples */
void scope_set_window(uint16_t samples);
/* Reset the scope after a trigger */
void scope_arm(void);
/* Stop logging data after the number of samples is reached */
void scope_trigger(void);
/* Log data to a channel */
void scope_log(uint8_t channel, float data);
/* Return whether the scope has been triggered */
bool scope_is_triggered(uint8_t channel);
/* Return the average of the triggered window */
float scope_get_triggered_average(uint8_t channel);
/* Return the average of all samples */
float scope_get_average(uint8_t channel);

#endif /* _SCOPE_H_ */
