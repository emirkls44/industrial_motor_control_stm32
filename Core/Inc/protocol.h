#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#define PROTOCOL_SOF_1             0xAAU
#define PROTOCOL_SOF_2             0x55U
#define PROTOCOL_VERSION           0x01U
#define PROTOCOL_MSG_TELEMETRY     0x01U
#define PROTOCOL_TELEMETRY_PAYLOAD_SIZE 7U
#define PROTOCOL_PACKET_SIZE       14U

typedef struct {
	uint8_t state;
	uint8_t command_duty;
	uint8_t target_duty;
	uint8_t current_duty;
	uint8_t estop;
	uint16_t adc_raw;
} ProtocolTelemetry_t;

uint8_t Protocol_Crc8(const uint8_t *data, uint16_t length);

uint16_t Protocol_BuildTelemetry(uint8_t *buffer, uint16_t buffer_size,
		uint8_t sequence, const ProtocolTelemetry_t *telemetry);

#endif /* PROTOCOL_H */
