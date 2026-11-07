#include "command_receiver.h"

void CommandReceiver_Init(CommandReceiver_t *receiver, UART_HandleTypeDef *huart)
{
    if (receiver == NULL)
    {
        return;
    }

    receiver->huart = huart;
    Protocol_ParserInit(&receiver->parser);
    receiver->rx_byte = 0U;
    receiver->last_frame_tick = HAL_GetTick();
    receiver->uart_errors = 0U;
    receiver->command_pending = false;
    receiver->pending_command = 0U;
    receiver->pending_value = 0U;
}

HAL_StatusTypeDef CommandReceiver_Start(CommandReceiver_t *receiver)
{
    if ((receiver == NULL) || (receiver->huart == NULL))
    {
        return HAL_ERROR;
    }

    return HAL_UART_Receive_IT(receiver->huart, &receiver->rx_byte, 1U);
}

void CommandReceiver_OnRxComplete(CommandReceiver_t *receiver, UART_HandleTypeDef *huart)
{
    if ((receiver == NULL) || (huart != receiver->huart))
    {
        return;
    }

    ProtocolFrame_t frame;

    if (Protocol_ParserFeed(&receiver->parser, receiver->rx_byte, &frame))
    {
        ProtocolCommand_t command;
        uint8_t value;

        if (Protocol_DecodeCommand(&frame, &command, &value))
        {
            receiver->last_frame_tick = HAL_GetTick();

            if (command != PROTOCOL_CMD_HEARTBEAT)
            {
                receiver->pending_command = (uint8_t)command;
                receiver->pending_value = value;
                receiver->command_pending = true;
            }
        }
    }

    if (HAL_UART_Receive_IT(receiver->huart, &receiver->rx_byte, 1U) != HAL_OK)
    {
        receiver->uart_errors++;
    }
}

void CommandReceiver_OnError(CommandReceiver_t *receiver, UART_HandleTypeDef *huart)
{
    if ((receiver == NULL) || (huart != receiver->huart))
    {
        return;
    }

    receiver->uart_errors++;

    /* An overrun aborts the HAL reception. Without restarting it here the
     * command channel would stay dead until the next reset. For non-blocking
     * errors (framing, noise) the reception is still running and this call
     * simply returns HAL_BUSY. */
    (void)HAL_UART_Receive_IT(receiver->huart, &receiver->rx_byte, 1U);
}

bool CommandReceiver_GetCommand(CommandReceiver_t *receiver, ProtocolCommand_t *command,
                                uint8_t *value)
{
    if ((receiver == NULL) || (command == NULL) || (value == NULL))
    {
        return false;
    }

    bool available = false;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (receiver->command_pending)
    {
        *command = (ProtocolCommand_t)receiver->pending_command;
        *value = receiver->pending_value;
        receiver->command_pending = false;
        available = true;
    }

    __set_PRIMASK(primask);

    return available;
}

uint32_t CommandReceiver_GetLastFrameTick(const CommandReceiver_t *receiver)
{
    return (receiver != NULL) ? receiver->last_frame_tick : 0U;
}
