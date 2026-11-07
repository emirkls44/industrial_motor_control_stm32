#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "protocol.h"
#include "stm32f0xx_hal.h"

/* Periodic telemetry transmitter (STM32 -> ESP32). */
typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t sequence;
    uint32_t last_tx_tick;
    uint32_t period_ms;
} Communication_t;

void Communication_Init(Communication_t *comm, UART_HandleTypeDef *huart, uint32_t period_ms);

/* Sends `telemetry` if the period has elapsed since the last frame. */
void Communication_Update(Communication_t *comm, const ProtocolTelemetry_t *telemetry);

#endif /* COMMUNICATION_H */
