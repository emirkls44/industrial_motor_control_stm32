#include "communication.h"

/* 14 bytes at 115200 baud take ~1.2 ms; 10 ms leaves a wide margin. */
#define TELEMETRY_TX_TIMEOUT_MS 10U

void Communication_Init(Communication_t *comm, UART_HandleTypeDef *huart, uint32_t period_ms)
{
    if (comm == NULL)
    {
        return;
    }

    comm->huart = huart;
    comm->sequence = 0U;
    comm->last_tx_tick = HAL_GetTick();
    comm->period_ms = period_ms;
}

void Communication_Update(Communication_t *comm, const ProtocolTelemetry_t *telemetry)
{
    if ((comm == NULL) || (comm->huart == NULL) || (telemetry == NULL))
    {
        return;
    }

    const uint32_t now = HAL_GetTick();

    if ((now - comm->last_tx_tick) < comm->period_ms)
    {
        return;
    }

    comm->last_tx_tick = now;

    uint8_t frame[PROTOCOL_TELEMETRY_FRAME_SIZE];
    const size_t length = Protocol_EncodeTelemetry(telemetry, comm->sequence, frame, sizeof(frame));

    if (length == 0U)
    {
        return;
    }

    /* The sequence number advances even if a frame is lost, so the receiver
     * can detect gaps. */
    comm->sequence++;

    (void)HAL_UART_Transmit(comm->huart, frame, (uint16_t)length, TELEMETRY_TX_TIMEOUT_MS);
}
