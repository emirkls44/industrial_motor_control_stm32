#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "motor_driver.h"

typedef enum {
	MOTOR_STOPPED = 0, MOTOR_STARTING, MOTOR_RUNNING, MOTOR_STOPPING
} MotorState_t;

typedef struct {
	MotorDriver_t *driver;
	MotorState_t state;
	uint8_t current_duty;
	uint8_t target_duty;
	uint32_t last_ramp_tick;
} MotorControl_t;

void MotorControl_Init(MotorControl_t *motor, MotorDriver_t *driver);
void MotorControl_Start(MotorControl_t *motor);
void MotorControl_Stop(MotorControl_t *motor);
void MotorControl_EmergencyStop(MotorControl_t *motor);
void MotorControl_SetTargetDuty(MotorControl_t *motor, uint8_t duty);
void MotorControl_Update(MotorControl_t *motor);

MotorState_t MotorControl_GetState(const MotorControl_t *motor);
uint8_t MotorControl_GetCurrentDuty(const MotorControl_t *motor);
uint8_t MotorControl_GetTargetDuty(const MotorControl_t *motor);

#endif /* MOTOR_CONTROL_H */
