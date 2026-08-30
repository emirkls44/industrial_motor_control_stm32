#include "protocol.h"

uint8_t Protocol_Crc8(const uint8_t *data, uint16_t length) {
	uint8_t crc = 0x00U;

	for (uint16_t i = 0U; i < length; i++) {
		crc ^= data[i];

		for (uint8_t bit = 0U; bit < 8U; bit++) {
			if ((crc & 0x80U) != 0U) {
				crc = (uint8_t) ((crc << 1U) ^ 0x07U);
			} else {
				crc <<= 1U;
			}
		}
	}

	return crc;
}

uint16_t Protocol_BuildTelemetry(uint8_t *buffer, uint16_t buffer_size,
		uint8_t sequence, const ProtocolTelemetry_t *telemetry) {
	if ((buffer == 0) || (telemetry == 0)
			|| (buffer_size < PROTOCOL_PACKET_SIZE)) {
		return 0U;
	}

	buffer[0] = PROTOCOL_SOF_1;
	buffer[1] = PROTOCOL_SOF_2;
	buffer[2] = PROTOCOL_VERSION;
	buffer[3] = PROTOCOL_MSG_TELEMETRY;
	buffer[4] = PROTOCOL_TELEMETRY_PAYLOAD_SIZE;
	buffer[5] = sequence;
	buffer[6] = telemetry->state;
	buffer[7] = telemetry->command_duty;
	buffer[8] = telemetry->target_duty;
	buffer[9] = telemetry->current_duty;
	buffer[10] = telemetry->estop;
	buffer[11] = (uint8_t) (telemetry->adc_raw & 0xFFU);
	buffer[12] = (uint8_t) ((telemetry->adc_raw >> 8U) & 0xFFU);

	buffer[13] = Protocol_Crc8(&buffer[2], 11U);

	return PROTOCOL_PACKET_SIZE;
}
