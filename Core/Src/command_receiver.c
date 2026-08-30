#include "command_receiver.h"

#define HEADER_1       0xAA
#define HEADER_2       0x55
#define PROTOCOL_VER   0x01
#define MSG_COMMAND    0x02
#define PAYLOAD_LEN    0x02
#define FRAME_SIZE     8

static uint8_t crc8(const uint8_t *data, uint16_t length)
{
    uint8_t crc = 0x00;

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x80)
            {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static void process_byte(CommandReceiver_t *receiver, uint8_t byte)
{
    if (receiver->index == 0)
    {
        if (byte == HEADER_1)
        {
            receiver->frame[0] = byte;
            receiver->index = 1;
        }
        return;
    }

    if (receiver->index == 1)
    {
        if (byte == HEADER_2)
        {
            receiver->frame[1] = byte;
            receiver->index = 2;
        }
        else
        {
            receiver->index = 0;
        }
        return;
    }

    receiver->frame[receiver->index++] = byte;

    if (receiver->index < FRAME_SIZE)
    {
        return;
    }

    receiver->index = 0;

    if (receiver->frame[2] != PROTOCOL_VER ||
        receiver->frame[3] != MSG_COMMAND ||
        receiver->frame[4] != PAYLOAD_LEN)
    {
        return;
    }

    if (crc8(&receiver->frame[2], 5) != receiver->frame[7])
    {
        return;
    }

    RemoteCommand_t command = (RemoteCommand_t)receiver->frame[5];

    if (command != REMOTE_CMD_START &&
        command != REMOTE_CMD_STOP &&
        command != REMOTE_CMD_SET_DUTY)
    {
        return;
    }

    receiver->pending_command = command;
    receiver->pending_value = receiver->frame[6];
    receiver->command_pending = true;
}

void CommandReceiver_Init(CommandReceiver_t *receiver,
                          UART_HandleTypeDef *huart)
{
    receiver->huart = huart;
    receiver->rx_byte = 0;
    receiver->index = 0;
    receiver->command_pending = false;
    receiver->pending_command = REMOTE_CMD_NONE;
    receiver->pending_value = 0;
}

void CommandReceiver_Start(CommandReceiver_t *receiver)
{
    HAL_UART_Receive_IT(receiver->huart, &receiver->rx_byte, 1);
}

void CommandReceiver_RxCpltCallback(CommandReceiver_t *receiver,
                                    UART_HandleTypeDef *huart)
{
    if (huart != receiver->huart)
    {
        return;
    }

    process_byte(receiver, receiver->rx_byte);

    HAL_UART_Receive_IT(receiver->huart, &receiver->rx_byte, 1);
}

bool CommandReceiver_GetCommand(CommandReceiver_t *receiver,
                                RemoteCommand_t *command,
                                uint8_t *value)
{
    if (!receiver->command_pending)
    {
        return false;
    }

    __disable_irq();

    *command = receiver->pending_command;
    *value = receiver->pending_value;
    receiver->command_pending = false;

    __enable_irq();

    return true;
}
