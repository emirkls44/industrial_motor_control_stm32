#include "motor_driver.h"

#define MAX_DUTY_PERCENT 100U

HAL_StatusTypeDef MotorDriver_Init(MotorDriver_t *driver, TIM_HandleTypeDef *htim, uint32_t channel)
{
    if ((driver == NULL) || (htim == NULL))
    {
        return HAL_ERROR;
    }

    driver->htim = htim;
    driver->channel = channel;

    /* CubeMX initialises the channel with a 50 % pulse; force 0 % before the
     * output is enabled. */
    __HAL_TIM_SET_COMPARE(driver->htim, driver->channel, 0U);

    return HAL_TIM_PWM_Start(driver->htim, driver->channel);
}

void MotorDriver_SetDuty(MotorDriver_t *driver, uint8_t duty)
{
    if ((driver == NULL) || (driver->htim == NULL))
    {
        return;
    }

    if (duty > MAX_DUTY_PERCENT)
    {
        duty = MAX_DUTY_PERCENT;
    }

    /* CCR = ARR + 1 gives a constant high output (100 %). */
    const uint32_t period = __HAL_TIM_GET_AUTORELOAD(driver->htim) + 1U;
    const uint32_t compare = (period * duty) / MAX_DUTY_PERCENT;

    __HAL_TIM_SET_COMPARE(driver->htim, driver->channel, compare);
}

void MotorDriver_Stop(MotorDriver_t *driver)
{
    MotorDriver_SetDuty(driver, 0U);
}
