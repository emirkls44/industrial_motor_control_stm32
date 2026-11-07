#ifndef ANALOG_INPUT_H
#define ANALOG_INPUT_H

#include "stm32f0xx_hal.h"

typedef struct
{
    ADC_HandleTypeDef *hadc;
    uint32_t last_raw;    /* averaged 12-bit reading      */
    uint8_t last_percent; /* 0..100 %, with hysteresis    */
} AnalogInput_t;

/* Calibrates the ADC and configures `channel` with a long sampling time.
 * Call once after MX_ADC_Init(). */
HAL_StatusTypeDef AnalogInput_Init(AnalogInput_t *input, ADC_HandleTypeDef *hadc, uint32_t channel);

uint32_t AnalogInput_ReadRaw(AnalogInput_t *input);
uint8_t AnalogInput_ReadPercent(AnalogInput_t *input);

#endif /* ANALOG_INPUT_H */
