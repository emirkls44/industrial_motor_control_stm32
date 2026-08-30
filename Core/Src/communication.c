#include "communication.h"

void Communication_Init(Communication_t *comm, UART_HandleTypeDef *huart,
		uint32_t period_ms) {
	if (comm == NULL) {
		return;
	}

	comm->huart = huart;
	comm->sequence = 0U;
	comm->last_tx_tick = HAL_GetTick();
	comm->period_ms = period_ms;
}

void Communication_Update(Communication_t *comm,
		const ProtocolTelemetry_t *telemetry) {
	if ((comm == NULL) || (comm->huart == NULL) || (telemetry == NULL)) {
		return;
	}

	const uint32_t now = HAL_GetTick();

	if ((now - comm->last_tx_tick) < comm->period_ms) {
		return;
	}

	comm->last_tx_tick = now;

	uint8_t packet[PROTOCOL_PACKET_SIZE];

	const uint16_t length = Protocol_BuildTelemetry(packet, sizeof(packet),
			comm->sequence, telemetry);

	if (length == 0U) {
		return;
	}

	if (HAL_UART_Transmit(comm->huart, packet, length, 10U) == HAL_OK) {
		comm->sequence++;
	}
}
