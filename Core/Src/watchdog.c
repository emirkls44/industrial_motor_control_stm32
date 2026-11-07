#include "watchdog.h"

#include "stm32f0xx_hal.h"

/* The HAL IWDG module is not part of this CubeMX project, so the peripheral
 * is driven through its four registers directly (RM0091, section 26). */
#define IWDG_KEY_RELOAD 0x0000AAAAU
#define IWDG_KEY_ENABLE 0x0000CCCCU
#define IWDG_KEY_ACCESS 0x00005555U

#define LSI_TYPICAL_HZ  40000U
#define IWDG_PRESCALER  64U /* PR = 4 -> 625 Hz watchdog clock */
#define IWDG_PR_DIV64   4U
#define IWDG_RELOAD_MAX 0x0FFFU

void Watchdog_Start(uint32_t timeout_ms)
{
    uint32_t reload = (timeout_ms * (LSI_TYPICAL_HZ / IWDG_PRESCALER)) / 1000U;

    if (reload == 0U)
    {
        reload = 1U;
    }
    else if (reload > IWDG_RELOAD_MAX)
    {
        reload = IWDG_RELOAD_MAX;
    }

    /* Do not reset the MCU while it is halted at a breakpoint. */
    __HAL_RCC_DBGMCU_CLK_ENABLE();
    __HAL_DBGMCU_FREEZE_IWDG();

    IWDG->KR = IWDG_KEY_ENABLE; /* also starts the LSI oscillator */
    IWDG->KR = IWDG_KEY_ACCESS;
    IWDG->PR = IWDG_PR_DIV64;
    IWDG->RLR = reload;

    while (IWDG->SR != 0U)
    {
        /* wait until PR and RLR are updated in the LSI domain */
    }

    IWDG->KR = IWDG_KEY_RELOAD;
}

void Watchdog_Kick(void)
{
    IWDG->KR = IWDG_KEY_RELOAD;
}

bool Watchdog_ConsumeResetFlag(void)
{
    const bool by_watchdog = (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0U);

    __HAL_RCC_CLEAR_RESET_FLAGS();

    return by_watchdog;
}
