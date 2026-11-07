/*
 * Host unit tests for the motor state machine (Core/Src/motor_control.c).
 * The HAL is replaced by fakes/stm32f0xx_hal.h and the PWM driver by the
 * functions below, which record the last duty in g_duty.
 */
#include "motor_control.h"

#include <stdio.h>

static int g_failures = 0;
static uint32_t g_tick = 0;
static uint8_t g_duty = 0; /* last duty written to the "PWM" */

#define CHECK(condition)                                                                           \
    do                                                                                             \
    {                                                                                              \
        if (!(condition))                                                                          \
        {                                                                                          \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);                          \
            g_failures++;                                                                          \
        }                                                                                          \
    } while (0)

uint32_t HAL_GetTick(void)
{
    return g_tick;
}

void MotorDriver_SetDuty(MotorDriver_t *driver, uint8_t duty)
{
    (void)driver;
    g_duty = duty;
}

void MotorDriver_Stop(MotorDriver_t *driver)
{
    (void)driver;
    g_duty = 0;
}

/* Advances time in 50 ms ramp periods, calling Update each period. */
static void run_periods(MotorControl_t *motor, int periods)
{
    for (int i = 0; i < periods; i++)
    {
        g_tick += 50;
        MotorControl_Update(motor);
    }
}

static void setup(MotorControl_t *motor, MotorDriver_t *driver)
{
    g_tick = 1000;
    g_duty = 77; /* garbage, Init must clear it */
    MotorControl_Init(motor, driver);
}

static void test_init_stops_output(void)
{
    MotorControl_t motor;
    MotorDriver_t driver;
    setup(&motor, &driver);

    CHECK(g_duty == 0);
    CHECK(MotorControl_GetState(&motor) == MOTOR_STOPPED);
}

static void test_soft_start_begins_at_min_duty(void)
{
    MotorControl_t motor;
    MotorDriver_t driver;
    setup(&motor, &driver);

    MotorControl_SetTargetDuty(&motor, 80);
    MotorControl_Start(&motor);
    run_periods(&motor, 1);

    CHECK(MotorControl_GetState(&motor) == MOTOR_STARTING);
    CHECK(g_duty == MOTOR_CONTROL_MIN_DUTY);

    run_periods(&motor, 1);
    CHECK(g_duty == MOTOR_CONTROL_MIN_DUTY + 2);
}

static void test_reaches_target_and_runs(void)
{
    MotorControl_t motor;
    MotorDriver_t driver;
    setup(&motor, &driver);

    MotorControl_SetTargetDuty(&motor, 80);
    MotorControl_Start(&motor);
    run_periods(&motor, 40);

    CHECK(MotorControl_GetState(&motor) == MOTOR_RUNNING);
    CHECK(g_duty == 80);
}

static void test_ramp_is_rate_limited(void)
{
    MotorControl_t motor;
    MotorDriver_t driver;
    setup(&motor, &driver);

    MotorControl_SetTargetDuty(&motor, 100);
    MotorControl_Start(&motor);
    g_tick += 50;
    MotorControl_Update(&motor);
    const uint8_t first = g_duty;

    /* Calls within the same 50 ms period must not change the duty */
    g_tick += 10;
    MotorControl_Update(&motor);
    MotorControl_Update(&motor);
    CHECK(g_duty == first);
}

static void test_below_min_duty_is_zero(void)
{
    MotorControl_t motor;
    MotorDriver_t driver;
    setup(&motor, &driver);

    MotorControl_SetTargetDuty(&motor, MOTOR_CONTROL_MIN_DUTY - 1);
    CHECK(MotorControl_GetTargetDuty(&motor) == 0);

    MotorControl_SetTargetDuty(&motor, 150);
    CHECK(MotorControl_GetTargetDuty(&motor) == 100);
}

/* Regression: turning the potentiometer to zero used to leave the motor in
 * RUNNING at 0 %, and turning it back up restarted the motor on its own. */
static void test_pot_to_zero_stops_and_does_not_restart(void)
{
    MotorControl_t motor;
    MotorDriver_t driver;
    setup(&motor, &driver);

    MotorControl_SetTargetDuty(&motor, 70);
    MotorControl_Start(&motor);
    run_periods(&motor, 40);
    CHECK(MotorControl_GetState(&motor) == MOTOR_RUNNING);

    MotorControl_SetTargetDuty(&motor, 0);
    run_periods(&motor, 40);
    CHECK(MotorControl_GetState(&motor) == MOTOR_STOPPED);
    CHECK(g_duty == 0);

    MotorControl_SetTargetDuty(&motor, 70);
    run_periods(&motor, 40);
    CHECK(MotorControl_GetState(&motor) == MOTOR_STOPPED);
    CHECK(g_duty == 0);
}

static void test_start_with_pot_at_zero_does_nothing(void)
{
    MotorControl_t motor;
    MotorDriver_t driver;
    setup(&motor, &driver);

    MotorControl_SetTargetDuty(&motor, 0);
    MotorControl_Start(&motor);
    run_periods(&motor, 5);

    CHECK(MotorControl_GetState(&motor) == MOTOR_STOPPED);
    CHECK(g_duty == 0);
}

static void test_soft_stop_cuts_output_at_min_duty(void)
{
    MotorControl_t motor;
    MotorDriver_t driver;
    setup(&motor, &driver);

    MotorControl_SetTargetDuty(&motor, 50);
    MotorControl_Start(&motor);
    run_periods(&motor, 20);
    CHECK(g_duty == 50);

    MotorControl_Stop(&motor);
    int periods = 0;
    while ((MotorControl_GetState(&motor) != MOTOR_STOPPED) && (periods < 100))
    {
        run_periods(&motor, 1);
        CHECK((g_duty == 0) || (g_duty >= MOTOR_CONTROL_MIN_DUTY));
        periods++;
    }

    CHECK(MotorControl_GetState(&motor) == MOTOR_STOPPED);
    CHECK(g_duty == 0);
    CHECK(periods <= 5); /* 50 -> 48 -> 46 -> 44 -> 43 -> 0 */
}

static void test_restart_during_soft_stop(void)
{
    MotorControl_t motor;
    MotorDriver_t driver;
    setup(&motor, &driver);

    MotorControl_SetTargetDuty(&motor, 90);
    MotorControl_Start(&motor);
    run_periods(&motor, 40);

    MotorControl_Stop(&motor);
    run_periods(&motor, 3);
    CHECK(MotorControl_GetState(&motor) == MOTOR_STOPPING);

    MotorControl_Start(&motor);
    run_periods(&motor, 10);
    CHECK(MotorControl_GetState(&motor) == MOTOR_RUNNING);
    CHECK(g_duty == 90);
}

static void test_emergency_stop_is_immediate(void)
{
    MotorControl_t motor;
    MotorDriver_t driver;
    setup(&motor, &driver);

    MotorControl_SetTargetDuty(&motor, 100);
    MotorControl_Start(&motor);
    run_periods(&motor, 40);

    MotorControl_EmergencyStop(&motor);
    CHECK(g_duty == 0);
    CHECK(MotorControl_GetState(&motor) == MOTOR_STOPPED);
}

static void test_tick_wraparound(void)
{
    MotorControl_t motor;
    MotorDriver_t driver;
    setup(&motor, &driver);
    g_tick = 0xFFFFFFF0u;
    MotorControl_Init(&motor, &driver);

    MotorControl_SetTargetDuty(&motor, 60);
    MotorControl_Start(&motor);
    run_periods(&motor, 20);

    CHECK(g_duty == 60);
}

int main(void)
{
    struct
    {
        const char *name;
        void (*run)(void);
    } const tests[] = {
        {"init_stops_output", test_init_stops_output},
        {"soft_start_begins_at_min_duty", test_soft_start_begins_at_min_duty},
        {"reaches_target_and_runs", test_reaches_target_and_runs},
        {"ramp_is_rate_limited", test_ramp_is_rate_limited},
        {"below_min_duty_is_zero", test_below_min_duty_is_zero},
        {"pot_to_zero_stops_and_does_not_restart", test_pot_to_zero_stops_and_does_not_restart},
        {"start_with_pot_at_zero_does_nothing", test_start_with_pot_at_zero_does_nothing},
        {"soft_stop_cuts_output_at_min_duty", test_soft_stop_cuts_output_at_min_duty},
        {"restart_during_soft_stop", test_restart_during_soft_stop},
        {"emergency_stop_is_immediate", test_emergency_stop_is_immediate},
        {"tick_wraparound", test_tick_wraparound},
    };

    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++)
    {
        const int before = g_failures;
        tests[i].run();
        printf("%s %s\n", (g_failures == before) ? "PASS" : "FAIL", tests[i].name);
    }

    printf("\n%d failure(s)\n", g_failures);
    return (g_failures == 0) ? 0 : 1;
}
