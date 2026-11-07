/* Minimal host stand-in for the STM32 HAL, used only by the unit tests. */
#ifndef STM32F0XX_HAL_H
#define STM32F0XX_HAL_H

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    HAL_OK = 0,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT
} HAL_StatusTypeDef;

typedef struct
{
    int unused;
} TIM_HandleTypeDef;

uint32_t HAL_GetTick(void);

#endif
