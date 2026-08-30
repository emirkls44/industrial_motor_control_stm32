#ifndef COMMAND_RECEIVER_H
#define COMMAND_RECEIVER_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    REMOTE_CMD_NONE = 0,
    REMOTE_CMD_START = 1,
    REMOTE_CMD_STOP = 2,
    REMOTE_CMD_SET_DUTY = 3
} RemoteCommand_t;

typedef struct
{
    UART_HandleTypeDef *huart;

    uint8_t rx_byte;
    uint8_t frame[8];
    uint8_t index;

    volatile bool command_pending;
    volatile RemoteCommand_t pending_command;
    volatile uint8_t pending_value;
} CommandReceiver_t;

void CommandReceiver_Init(CommandReceiver_t *receiver,
                          UART_HandleTypeDef *huart);

void CommandReceiver_Start(CommandReceiver_t *receiver);

void CommandReceiver_RxCpltCallback(CommandReceiver_t *receiver,
                                    UART_HandleTypeDef *huart);

bool CommandReceiver_GetCommand(CommandReceiver_t *receiver,
                                RemoteCommand_t *command,
                                uint8_t *value);

#endif
