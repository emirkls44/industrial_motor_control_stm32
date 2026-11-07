#include "motor_control.h"

#define MOTOR_MAX_DUTY       100U
#define MOTOR_RAMP_STEP      2U  /* % per ramp period        */
#define MOTOR_RAMP_PERIOD_MS 50U /* -> 40 %/s ramp           */

static void apply_duty(MotorControl_t *motor, uint8_t duty)
{
    motor->current_duty = duty;
    MotorDriver_SetDuty(motor->driver, duty);
}

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
    if (motor == NULL)
    {
        return;
    }

    /* A start request during a soft stop ramps back up from the current duty. */
    if ((motor->state == MOTOR_STOPPED) || (motor->state == MOTOR_STOPPING))
    {
        motor->state = MOTOR_STARTING;
        motor->last_ramp_tick = HAL_GetTick();
    }
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

    motor->target_duty = 0U;
    motor->state = MOTOR_STOPPED;
    apply_duty(motor, 0U);
}

void MotorControl_SetTargetDuty(MotorControl_t *motor, uint8_t duty)
{
    if (motor == NULL)
    {
        return;
    }

    if (duty > MOTOR_MAX_DUTY)
    {
        duty = MOTOR_MAX_DUTY;
    }

    if (duty < MOTOR_CONTROL_MIN_DUTY)
    {
        duty = 0U;
    }

    motor->target_duty = duty;
}

static void ramp_down_to_stop(MotorControl_t *motor)
{
    /* Ramping through the dead zone below MIN_DUTY only heats the motor, so
     * the output is cut as soon as the duty reaches MIN_DUTY. */
    if (motor->current_duty <= MOTOR_CONTROL_MIN_DUTY)
    {
        apply_duty(motor, 0U);
        motor->state = MOTOR_STOPPED;
        return;
    }

    const uint8_t above_min = motor->current_duty - MOTOR_CONTROL_MIN_DUTY;
    const uint8_t step = (above_min < MOTOR_RAMP_STEP) ? above_min : MOTOR_RAMP_STEP;

    apply_duty(motor, motor->current_duty - step);
}

static void ramp_towards_target(MotorControl_t *motor)
{
    uint8_t duty = motor->current_duty;

    /* Start directly at the lowest duty that turns the motor instead of
     * spending ~1 s stalled between 0 % and MIN_DUTY. */
    if (duty < MOTOR_CONTROL_MIN_DUTY)
    {
        duty = MOTOR_CONTROL_MIN_DUTY;
    }
    else if (duty < motor->target_duty)
    {
        const uint8_t difference = motor->target_duty - duty;
        duty += (difference < MOTOR_RAMP_STEP) ? difference : MOTOR_RAMP_STEP;
    }
    else if (duty > motor->target_duty)
    {
        const uint8_t difference = duty - motor->target_duty;
        duty -= (difference < MOTOR_RAMP_STEP) ? difference : MOTOR_RAMP_STEP;
    }

    apply_duty(motor, duty);

    if ((motor->state == MOTOR_STARTING) && (duty == motor->target_duty))
    {
        motor->state = MOTOR_RUNNING;
    }
}

void MotorControl_Update(MotorControl_t *motor)
{
    if ((motor == NULL) || (motor->driver == NULL) || (motor->state == MOTOR_STOPPED))
    {
        return;
    }

    const uint32_t now = HAL_GetTick();

    if ((now - motor->last_ramp_tick) < MOTOR_RAMP_PERIOD_MS)
    {
        return;
    }

    motor->last_ramp_tick = now;

    /* A target of 0 % stops the motor. It then stays stopped until a new
     * start command, even if the potentiometer is turned up again. */
    if (motor->target_duty == 0U)
    {
        motor->state = MOTOR_STOPPING;
    }

    if (motor->state == MOTOR_STOPPING)
    {
        ramp_down_to_stop(motor);
    }
    else
    {
        ramp_towards_target(motor);
    }
}

MotorState_t MotorControl_GetState(const MotorControl_t *motor)
{
    return (motor != NULL) ? motor->state : MOTOR_STOPPED;
}

uint8_t MotorControl_GetCurrentDuty(const MotorControl_t *motor)
{
    return (motor != NULL) ? motor->current_duty : 0U;
}

uint8_t MotorControl_GetTargetDuty(const MotorControl_t *motor)
{
    return (motor != NULL) ? motor->target_duty : 0U;
}
