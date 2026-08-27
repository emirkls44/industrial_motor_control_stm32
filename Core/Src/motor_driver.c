#include "motor_driver.h"

HAL_StatusTypeDef MotorDriver_Init(MotorDriver_t *driver,
		TIM_HandleTypeDef *htim, uint32_t channel) {
	if ((driver == NULL) || (htim == NULL)) {
		return HAL_ERROR;
	}

	driver->htim = htim;
	driver->channel = channel;

	__HAL_TIM_SET_COMPARE(driver->htim, driver->channel, 0U);

	return HAL_TIM_PWM_Start(driver->htim, driver->channel);
}

void MotorDriver_SetDuty(MotorDriver_t *driver, uint8_t duty) {
	if ((driver == NULL) || (driver->htim == NULL)) {
		return;
	}

	if (duty > 100U) {
		duty = 100U;
	}

	const uint32_t period = __HAL_TIM_GET_AUTORELOAD(driver->htim) + 1U;
	const uint32_t compare = (period * duty) / 100U;

	__HAL_TIM_SET_COMPARE(driver->htim, driver->channel, compare);
}

void MotorDriver_Stop(MotorDriver_t *driver) {
	MotorDriver_SetDuty(driver, 0U);
}
