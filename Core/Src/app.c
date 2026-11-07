#include "app.h"

#include "analog_input.h"
#include "button.h"
#include "command_receiver.h"
#include "communication.h"
#include "main.h"
#include "motor_control.h"
#include "motor_driver.h"
#include "protocol.h"
#include "watchdog.h"

#include <stdbool.h>

/* Peripheral handles created by CubeMX in main.c */
extern ADC_HandleTypeDef hadc;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;

/* ---------------------------------------------------------------------------
 * Configuration
 * ------------------------------------------------------------------------- */
#define START_BUTTON_PORT        GPIOA
#define START_BUTTON_PIN         GPIO_PIN_0 /* user button on the Discovery board */
#define START_BUTTON_DEBOUNCE_MS 30U

#define ESTOP_PORT GPIOA
#define ESTOP_PIN  GPIO_PIN_3 /* internal pull-up */

/* E-stop contact type:
 *   0 = normally open: pressing the button pulls PA3 to GND (current wiring).
 *       A broken wire reads as "not pressed", so this is NOT fail-safe.
 *   1 = normally closed: PA3 is held at GND while the circuit is intact;
 *       pressing the button OR a broken wire reads as "pressed". Recommended.
 * See the README for why this input must not be the only stop path. */
#ifndef ESTOP_CONTACT_NORMALLY_CLOSED
#define ESTOP_CONTACT_NORMALLY_CLOSED 0
#endif

#define POT_ADC_CHANNEL ADC_CHANNEL_1 /* PA1 */

#define MOTOR_PWM_TIMER   htim1
#define MOTOR_PWM_CHANNEL TIM_CHANNEL_1 /* PA8 */

#define TELEMETRY_PERIOD_MS 250U
#define LINK_TIMEOUT_MS     2000U /* gateway sends a heartbeat every 500 ms */
#define WATCHDOG_TIMEOUT_MS 500U

/* ---------------------------------------------------------------------------
 * State
 * ------------------------------------------------------------------------- */
typedef enum
{
    CONTROL_LOCAL = 0, /* start button + potentiometer */
    CONTROL_REMOTE     /* MQTT commands via the ESP32 gateway */
} ControlSource_t;

static MotorDriver_t s_motor_driver;
static MotorControl_t s_motor;
static AnalogInput_t s_pot;
static Button_t s_start_button;
static Button_t s_estop_button;
static Communication_t s_telemetry_tx;
static CommandReceiver_t s_command_rx;

static ControlSource_t s_control_source = CONTROL_LOCAL;
static uint8_t s_remote_duty = 0U;
static bool s_remote_duty_valid = false;
static bool s_link_lost = false;
static bool s_watchdog_reset = false;

/* ---------------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------------- */
static void switch_to_local_control(void)
{
    s_control_source = CONTROL_LOCAL;
    s_remote_duty_valid = false;
}

static void toggle_start_stop(void)
{
    const MotorState_t state = MotorControl_GetState(&s_motor);

    if ((state == MOTOR_STOPPED) || (state == MOTOR_STOPPING))
    {
        MotorControl_Start(&s_motor);
    }
    else
    {
        MotorControl_Stop(&s_motor);
    }
}

static void handle_start_button(void)
{
    if (Button_WasPressed(&s_start_button) == 0U)
    {
        return;
    }

    /* Any local action takes control back from the remote side. */
    switch_to_local_control();
    s_link_lost = false;
    toggle_start_stop();
}

static void handle_remote_command(void)
{
    ProtocolCommand_t command;
    uint8_t value;

    if (!CommandReceiver_GetCommand(&s_command_rx, &command, &value))
    {
        return;
    }

    switch (command)
    {
    case PROTOCOL_CMD_START:
        s_control_source = CONTROL_REMOTE;
        s_link_lost = false;
        MotorControl_Start(&s_motor);
        break;

    case PROTOCOL_CMD_STOP:
        /* A remote stop is always honoured, whoever started the motor. */
        MotorControl_Stop(&s_motor);
        break;

    case PROTOCOL_CMD_SET_DUTY:
        s_control_source = CONTROL_REMOTE;
        s_remote_duty = value;
        s_remote_duty_valid = true;
        break;

    default:
        break;
    }
}

static void supervise_link(void)
{
    if ((s_control_source != CONTROL_REMOTE) || (MotorControl_GetState(&s_motor) == MOTOR_STOPPED))
    {
        return;
    }

    /* Read the timestamp before the current tick: the UART interrupt may
     * update it in between, and reading it afterwards could make
     * `now - last_frame` wrap around to a huge value. */
    const uint32_t last_frame = CommandReceiver_GetLastFrameTick(&s_command_rx);
    const uint32_t now = HAL_GetTick();

    if ((now - last_frame) > LINK_TIMEOUT_MS)
    {
        MotorControl_Stop(&s_motor);
        s_link_lost = true;
        switch_to_local_control();
    }
}

static uint8_t requested_duty(uint8_t pot_duty)
{
    if ((s_control_source == CONTROL_REMOTE) && s_remote_duty_valid)
    {
        return s_remote_duty;
    }

    return pot_duty;
}

static void discard_pending_remote_command(void)
{
    ProtocolCommand_t command;
    uint8_t value;

    (void)CommandReceiver_GetCommand(&s_command_rx, &command, &value);
}

static void send_telemetry(uint8_t command_duty, bool estop_active)
{
    uint8_t flags = 0U;

    if (estop_active)
    {
        flags |= PROTOCOL_FLAG_ESTOP;
    }
    if (s_control_source == CONTROL_REMOTE)
    {
        flags |= PROTOCOL_FLAG_REMOTE;
    }
    if (s_link_lost)
    {
        flags |= PROTOCOL_FLAG_LINK_LOST;
    }
    if (s_watchdog_reset)
    {
        flags |= PROTOCOL_FLAG_WATCHDOG_RESET;
    }

    const ProtocolTelemetry_t telemetry = {
        .state = (uint8_t)MotorControl_GetState(&s_motor),
        .command_duty = command_duty,
        .target_duty = MotorControl_GetTargetDuty(&s_motor),
        .current_duty = MotorControl_GetCurrentDuty(&s_motor),
        .flags = flags,
        .adc_raw = (uint16_t)s_pot.last_raw,
    };

    Communication_Update(&s_telemetry_tx, &telemetry);
}

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */
void App_Init(void)
{
    s_watchdog_reset = Watchdog_ConsumeResetFlag();

    if (MotorDriver_Init(&s_motor_driver, &MOTOR_PWM_TIMER, MOTOR_PWM_CHANNEL) != HAL_OK)
    {
        Error_Handler();
    }
    MotorControl_Init(&s_motor, &s_motor_driver);

    if (AnalogInput_Init(&s_pot, &hadc, POT_ADC_CHANNEL) != HAL_OK)
    {
        Error_Handler();
    }

    Button_Init(&s_start_button, START_BUTTON_PORT, START_BUTTON_PIN, GPIO_PIN_SET,
                START_BUTTON_DEBOUNCE_MS);

    /* No debounce on the e-stop: it must act on the first sample. */
    Button_Init(&s_estop_button, ESTOP_PORT, ESTOP_PIN,
                ESTOP_CONTACT_NORMALLY_CLOSED ? GPIO_PIN_SET : GPIO_PIN_RESET, 0U);

    Communication_Init(&s_telemetry_tx, &huart1, TELEMETRY_PERIOD_MS);
    CommandReceiver_Init(&s_command_rx, &huart1);

    /* The USART1 interrupt is enabled here rather than in CubeMX; its handler
     * lives in the USER CODE section of stm32f0xx_it.c. */
    HAL_NVIC_SetPriority(USART1_IRQn, 0U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    if (CommandReceiver_Start(&s_command_rx) != HAL_OK)
    {
        Error_Handler();
    }

    Watchdog_Start(WATCHDOG_TIMEOUT_MS);
}

void App_Update(void)
{
    Watchdog_Kick();

    Button_Update(&s_estop_button);
    Button_Update(&s_start_button);

    const uint8_t pot_duty = AnalogInput_ReadPercent(&s_pot);
    const bool estop_active = (Button_IsPressed(&s_estop_button) != 0U);

    if (estop_active)
    {
        MotorControl_EmergencyStop(&s_motor);

        /* Presses and commands received during the e-stop must not start the
         * motor once it is released. */
        (void)Button_WasPressed(&s_start_button);
        discard_pending_remote_command();
        switch_to_local_control();
    }
    else
    {
        handle_start_button();
        handle_remote_command();
        supervise_link();

        MotorControl_SetTargetDuty(&s_motor, requested_duty(pot_duty));
        MotorControl_Update(&s_motor);
    }

    send_telemetry(requested_duty(pot_duty), estop_active);
}

void App_EmergencyOutputOff(void)
{
    /* Force OC1REF inactive. Unlike a CCR write, the OC1M field is not
     * preloaded, so the output goes low immediately. */
    TIM1->CCMR1 = (TIM1->CCMR1 & ~TIM_CCMR1_OC1M) | TIM_CCMR1_OC1M_2;
    TIM1->CCR1 = 0U;
}

/* ---------------------------------------------------------------------------
 * HAL callbacks
 * ------------------------------------------------------------------------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    CommandReceiver_OnRxComplete(&s_command_rx, huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    CommandReceiver_OnError(&s_command_rx, huart);
}
