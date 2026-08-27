#include "analog_input.h"

#define ADC_MAX_VALUE 4095U

void AnalogInput_Init(AnalogInput_t *input, ADC_HandleTypeDef *hadc) {
	if (input == NULL) {
		return;
	}

	input->hadc = hadc;
	input->last_raw = 0U;
}

uint32_t AnalogInput_ReadRaw(AnalogInput_t *input) {
	if ((input == NULL) || (input->hadc == NULL)) {
		return 0U;
	}

	if (HAL_ADC_Start(input->hadc) != HAL_OK) {
		return input->last_raw;
	}

	if (HAL_ADC_PollForConversion(input->hadc, 10U) == HAL_OK) {
		input->last_raw = HAL_ADC_GetValue(input->hadc);
	}

	(void) HAL_ADC_Stop(input->hadc);

	return input->last_raw;
}

uint8_t AnalogInput_ReadPercent(AnalogInput_t *input) {
	const uint32_t raw = AnalogInput_ReadRaw(input);

	return (uint8_t) ((raw * 100U) / ADC_MAX_VALUE);
}
