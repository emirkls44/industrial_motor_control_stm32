#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "stm32f0xx_hal.h"

typedef struct {
	TIM_HandleTypeDef *htim;
	uint32_t channel;
} MotorDriver_t;

HAL_StatusTypeDef MotorDriver_Init(MotorDriver_t *driver,
		TIM_HandleTypeDef *htim, uint32_t channel);

void MotorDriver_SetDuty(MotorDriver_t *driver, uint8_t duty);
void MotorDriver_Stop(MotorDriver_t *driver);

#endif /* MOTOR_DRIVER_H */
