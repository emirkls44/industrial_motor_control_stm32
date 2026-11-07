#include "analog_input.h"

#define ADC_MAX_VALUE             4095U
#define ADC_SAMPLES_TO_AVERAGE    8U
#define ADC_CONVERSION_TIMEOUT_MS 2U
#define PERCENT_HYSTERESIS        1U /* ignore +/-1 % jitter of the potentiometer */

HAL_StatusTypeDef AnalogInput_Init(AnalogInput_t *input, ADC_HandleTypeDef *hadc, uint32_t channel)
{
    if ((input == NULL) || (hadc == NULL))
    {
        return HAL_ERROR;
    }

    input->hadc = hadc;
    input->last_raw = 0U;
    input->last_percent = 0U;

    /* The STM32F0 ADC must be calibrated once while it is disabled. */
    if (HAL_ADCEx_Calibration_Start(hadc) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* CubeMX generated a 1.5-cycle sampling time. That is too short for the
     * sampling capacitor to charge through a potentiometer's source
     * impedance, which gives low and noisy readings. The input changes
     * slowly, so the longest sampling time costs nothing. This is set here
     * (and not only in the .ioc) so it survives code regeneration. */
    ADC_ChannelConfTypeDef config = {0};
    config.Channel = channel;
    config.Rank = ADC_RANK_CHANNEL_NUMBER;
    config.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

    if (HAL_ADC_ConfigChannel(hadc, &config) != HAL_OK)
    {
        return HAL_ERROR;
    }

    (void)AnalogInput_ReadPercent(input);

    return HAL_OK;
}

uint32_t AnalogInput_ReadRaw(AnalogInput_t *input)
{
    if ((input == NULL) || (input->hadc == NULL))
    {
        return 0U;
    }

    uint32_t sum = 0U;
    uint32_t samples = 0U;

    for (uint32_t i = 0U; i < ADC_SAMPLES_TO_AVERAGE; i++)
    {
        if (HAL_ADC_Start(input->hadc) != HAL_OK)
        {
            break;
        }

        if (HAL_ADC_PollForConversion(input->hadc, ADC_CONVERSION_TIMEOUT_MS) == HAL_OK)
        {
            sum += HAL_ADC_GetValue(input->hadc);
            samples++;
        }

        (void)HAL_ADC_Stop(input->hadc);
    }

    /* On failure keep the previous value rather than reporting 0 %. */
    if (samples > 0U)
    {
        input->last_raw = sum / samples;
    }

    return input->last_raw;
}

uint8_t AnalogInput_ReadPercent(AnalogInput_t *input)
{
    if (input == NULL)
    {
        return 0U;
    }

    const uint32_t raw = AnalogInput_ReadRaw(input);
    const uint8_t percent = (uint8_t)((raw * 100U + (ADC_MAX_VALUE / 2U)) / ADC_MAX_VALUE);

    const uint8_t difference = (percent > input->last_percent) ? (percent - input->last_percent)
                                                               : (input->last_percent - percent);

    /* The end stops are always reachable; elsewhere small jitter is ignored so
     * the duty does not toggle around the motor's minimum duty. */
    if ((difference > PERCENT_HYSTERESIS) || (percent == 0U) || (percent == 100U))
    {
        input->last_percent = percent;
    }

    return input->last_percent;
}
