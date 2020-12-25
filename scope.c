#include "scope.h"
#include <string.h>

static volatile float scope_data[NUM_CHANNELS][MAX_SAMPLES];
static volatile uint16_t num_samples;
static volatile uint16_t samples_since_trigger[NUM_CHANNELS];
static volatile bool triggered;
static volatile uint16_t curr_index[NUM_CHANNELS];
static volatile float avg_sum_triggered[NUM_CHANNELS];
static volatile uint16_t trigger_start_curr_index[NUM_CHANNELS];
static volatile bool logging = false;

void scope_init(void)
{
    num_samples = MAX_SAMPLES;
    uint8_t i = 0;
    for (i = 0; i < NUM_CHANNELS; i++)
    {
        samples_since_trigger[i] = 0;
        curr_index[i] = 0;
        avg_sum_triggered[i] = 0;
        trigger_start_curr_index[i] = 0;
    }
    triggered = false;
    logging = false;
    memset((float*)scope_data, 0, sizeof(float) * NUM_CHANNELS * MAX_SAMPLES);
}

void scope_set_window(uint16_t samples)
{
    if (samples > MAX_SAMPLES)
    {
        num_samples = MAX_SAMPLES;
        return;
    }
    num_samples = samples;
}

void scope_clear(uint8_t channel)
{
    memset((float*)scope_data[channel], 0, sizeof(float) * MAX_SAMPLES);
    curr_index[channel] = 0;
}

void scope_arm(void)
{
    triggered = false;
    logging = true;
}

void scope_trigger(void)
{
    triggered = true;
    uint8_t i = 0;
    for (i = 0; i < NUM_CHANNELS; i++)
    {
        samples_since_trigger[i] = 0;
        avg_sum_triggered[i] = 0;
        trigger_start_curr_index[i] = curr_index[i];
    }
}

void scope_log(uint8_t channel, float data)
{
    if (!logging)
    {
        return;
    }
    if (channel >= NUM_CHANNELS)
    {
        return;
    }
    scope_data[channel][curr_index[channel]] = data;
    curr_index[channel] = (curr_index[channel] + 1) % num_samples;
    if (triggered)
    {
        samples_since_trigger[channel]++;
        avg_sum_triggered[channel] += data;
        if (scope_is_triggered(channel))
        {
            logging = false;
        }
    }
}

bool scope_is_triggered(uint8_t channel)
{
    return triggered && samples_since_trigger[channel] >= 64;
}

float scope_get_triggered_average(uint8_t channel)
{
    if (samples_since_trigger[channel] == 0)
        return 0.0;
    return avg_sum_triggered[channel] / (float)samples_since_trigger[channel];
}

float scope_get_average(uint8_t channel)
{
    uint16_t i = 0;
    float sum = 0;
    for (i = 0; i < num_samples; i++)
    {
        sum += scope_data[channel][i];
    }
    return sum / (float)num_samples;
}

uint32_t scope_get_data(uint8_t channel, uint16_t samples, float *buffer)
{
    for (uint16_t i = 0; i < samples; i++)
    {
        buffer[i] = scope_data[channel][(curr_index[channel] + i + num_samples - samples) % num_samples];
    }
    return samples;
}
