#include "motor_control.h"

#define MOTOR_MIN_DUTY        43U
#define MOTOR_RAMP_STEP       2U
#define MOTOR_RAMP_PERIOD_MS  50U

void MotorControl_Init(MotorControl_t *motor, MotorDriver_t *driver)
{
    if (motor == NULL)
    {
        return;
    }

    motor->driver = driver;
    motor->state = MOTOR_STOPPED;
    motor->current_duty = 0U;
    motor->target_duty = 0U;
    motor->last_ramp_tick = HAL_GetTick();

    MotorDriver_Stop(driver);
}

void MotorControl_Start(MotorControl_t *motor)
{
    if ((motor == NULL) || (motor->state != MOTOR_STOPPED))
    {
        return;
    }

    motor->state = MOTOR_STARTING;
    motor->last_ramp_tick = HAL_GetTick();
}

void MotorControl_Stop(MotorControl_t *motor)
{
    if (motor == NULL)
    {
        return;
    }

    if (motor->state != MOTOR_STOPPED)
    {
        motor->state = MOTOR_STOPPING;
    }
}

void MotorControl_EmergencyStop(MotorControl_t *motor)
{
    if (motor == NULL)
    {
        return;
    }

    motor->current_duty = 0U;
    motor->target_duty = 0U;
    motor->state = MOTOR_STOPPED;

    MotorDriver_Stop(motor->driver);
}

void MotorControl_SetTargetDuty(MotorControl_t *motor, uint8_t duty)
{
    if (motor == NULL)
    {
        return;
    }

    if (duty > 100U)
    {
        duty = 100U;
    }

    if (duty < MOTOR_MIN_DUTY)
    {
        duty = 0U;
    }

    motor->target_duty = duty;
}

void MotorControl_Update(MotorControl_t *motor)
{
    if ((motor == NULL) || (motor->driver == NULL))
    {
        return;
    }

    if (motor->state == MOTOR_STOPPED)
    {
        return;
    }

    const uint32_t now = HAL_GetTick();

    if ((now - motor->last_ramp_tick) < MOTOR_RAMP_PERIOD_MS)
    {
        return;
    }

    motor->last_ramp_tick = now;

    /* Normal STOP: soft-stop */
    if (motor->state == MOTOR_STOPPING)
    {
        if (motor->current_duty <= MOTOR_RAMP_STEP)
        {
            motor->current_duty = 0U;
            motor->state = MOTOR_STOPPED;
        }
        else
        {
            motor->current_duty -= MOTOR_RAMP_STEP;
        }

        MotorDriver_SetDuty(motor->driver, motor->current_duty);
        return;
    }

    /* STARTING / RUNNING: hedef duty'ye yumuşak geçiş */
    if (motor->current_duty < motor->target_duty)
    {
        const uint8_t difference = motor->target_duty - motor->current_duty;

        if (difference <= MOTOR_RAMP_STEP)
        {
            motor->current_duty = motor->target_duty;
        }
        else
        {
            motor->current_duty += MOTOR_RAMP_STEP;
        }
    }
    else if (motor->current_duty > motor->target_duty)
    {
        const uint8_t difference = motor->current_duty - motor->target_duty;

        if (difference <= MOTOR_RAMP_STEP)
        {
            motor->current_duty = motor->target_duty;
        }
        else
        {
            motor->current_duty -= MOTOR_RAMP_STEP;
        }
    }

    MotorDriver_SetDuty(motor->driver, motor->current_duty);

    if ((motor->state == MOTOR_STARTING) &&
        (motor->current_duty == motor->target_duty))
    {
        motor->state = MOTOR_RUNNING;
    }
}

MotorState_t MotorControl_GetState(const MotorControl_t *motor)
{
    if (motor == NULL)
    {
        return MOTOR_STOPPED;
    }

    return motor->state;
}

uint8_t MotorControl_GetCurrentDuty(const MotorControl_t *motor)
{
    return (motor != NULL) ? motor->current_duty : 0U;
}

uint8_t MotorControl_GetTargetDuty(const MotorControl_t *motor)
{
	return (motor != NULL) ? motor->target_duty : 0U;
}
