#ifndef COMMAND_RECEIVER_H
#define COMMAND_RECEIVER_H

#include "protocol.h"
#include "stm32f0xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * Receives command frames from the ESP32 gateway with a one-byte interrupt
 * reception. Bytes are parsed in the UART interrupt; the main loop picks up
 * the latest command with CommandReceiver_GetCommand().
 */
typedef struct
{
    UART_HandleTypeDef *huart;
    ProtocolParser_t parser;
    uint8_t rx_byte;

    volatile uint32_t last_frame_tick; /* HAL tick of the last valid frame (any command) */
    volatile uint16_t uart_errors;

    volatile bool command_pending;
    volatile uint8_t pending_command; /* ProtocolCommand_t */
    volatile uint8_t pending_value;
} CommandReceiver_t;

void CommandReceiver_Init(CommandReceiver_t *receiver, UART_HandleTypeDef *huart);

/* Must be called after HAL_UART_Init() and after the USART IRQ is enabled. */
HAL_StatusTypeDef CommandReceiver_Start(CommandReceiver_t *receiver);

/* Call from HAL_UART_RxCpltCallback() / HAL_UART_ErrorCallback(). */
void CommandReceiver_OnRxComplete(CommandReceiver_t *receiver, UART_HandleTypeDef *huart);
void CommandReceiver_OnError(CommandReceiver_t *receiver, UART_HandleTypeDef *huart);

/* Returns true and the command if a new START/STOP/SET_DUTY command arrived.
 * Heartbeats only refresh the link timestamp and are never returned here. */
bool CommandReceiver_GetCommand(CommandReceiver_t *receiver, ProtocolCommand_t *command,
                                uint8_t *value);

uint32_t CommandReceiver_GetLastFrameTick(const CommandReceiver_t *receiver);

#endif /* COMMAND_RECEIVER_H */
