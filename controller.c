#include "controller.h"
#include "ch.h"
#include "hal.h"
#include "isr_vector_table.h"

#include "zsm.h"

static ZSMMode zsm_mode = SINUSOIDAL;

CH_IRQ_HANDLER(ADC1_2_3_IRQHandler) {
    CH_IRQ_PROLOGUE();
    ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
    controller_update();
    CH_IRQ_EPILOGUE();
}

void controller_init(void)
{
}

/* Updates the controller state machine. Called as an interrupt handler on new ADC sample. */
void controller_update(void)
{

}

void controller_apply_zsm(int *a, int *b, int *c)
{
    switch(zsm_mode)
    {
        case SINUSOIDAL:
            zsm_sinusoidal(a, b, c);
            break;
        case MIDPOINT_CLAMP:
            zsm_midpoint_clamp(a, b, c);
            break;
        case TOP_CLAMP:
            zsm_top_clamp(a, b, c);
            break;
        case BOTTOM_CLAMP:
            zsm_bottom_clamp(a, b, c);
            break;
        case TOP_BOTTOM_CLAMP:
            zsm_top_clamp(a, b, c);
            break;
    }
}