#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "stm32f0xx_hal.h"
#include "protocol.h"

typedef struct {
	UART_HandleTypeDef *huart;
	uint8_t sequence;
	uint32_t last_tx_tick;
	uint32_t period_ms;
} Communication_t;

void Communication_Init(Communication_t *comm, UART_HandleTypeDef *huart,
		uint32_t period_ms);

void Communication_Update(Communication_t *comm,
		const ProtocolTelemetry_t *telemetry);

#endif /* COMMUNICATION_H */
