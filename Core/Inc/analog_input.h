#ifndef ANALOG_INPUT_H
#define ANALOG_INPUT_H

#include "stm32f0xx_hal.h"

typedef struct {
	ADC_HandleTypeDef *hadc;
	uint32_t last_raw;
} AnalogInput_t;

void AnalogInput_Init(AnalogInput_t *input, ADC_HandleTypeDef *hadc);
uint32_t AnalogInput_ReadRaw(AnalogInput_t *input);
uint8_t AnalogInput_ReadPercent(AnalogInput_t *input);

#endif /* ANALOG_INPUT_H */
