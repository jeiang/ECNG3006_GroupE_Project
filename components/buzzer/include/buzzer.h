/**
 * @file buzzer.h
 * @brief Library to interface with a buzzer to sound an alarm.
 *
 * @author Aidan Pinard (jeiang)
 * @bug No known bugs.
 */

#ifndef BUZZER_H
#define BUZZER_H

#define configUSE_TASK_NOTIFICATIONS 1

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp8266/gpio_register.h"
#include "esp8266/pin_mux_register.h"
#include "driver/pwm.h"

#define BUZZER_CTX_WAIT_TIME         ((TickType_t)1)
#define BUZZER_PLAY_TASK_STACK_SIZE  1024
#define BUZZER_PLAY_TASK_PRIORITY    5
#define BUZZER_PWM_CHANNEL_COUNT     1
#define BUZZER_ALARM_NOTE_COUNT      10
#define BUZZER_ALARM_NOTE_PLAY_TICKS (pdMS_TO_TICKS(150))
#define BUZZER_A4_PERIOD_US          2273
#define BUZZER_B4_PERIOD_US          2025
#define BUZZER_C4_PERIOD_US          3822
#define BUZZER_C5_PERIOD_US          1911
#define BUZZER_E4_PERIOD_US          3034
#define BUZZER_F4_PERIOD_US          2863
#define BUZZER_G4_PERIOD_US          2551

#define BUZZER_ESP_CHECK(X)       \
    {                             \
        esp_err_t __err_rc = (X); \
        if (__err_rc != ESP_OK)   \
        {                         \
            return __err_rc;      \
        }                         \
    }

typedef enum
{
    BUZZER_OUTPUT_PIN0 = 0,
    BUZZER_OUTPUT_PIN1 = 1,
    BUZZER_OUTPUT_PIN2 = 2,
    BUZZER_OUTPUT_PIN3 = 3,
} buzzer_output_pin_t;

typedef enum
{
    BUZZER_ALARM_HEART = 0,
    BUZZER_ALARM_OXYGEN,
} buzzer_alarm_reason_t;

static const char *
buzzer_period_us_to_note(uint32_t period_us);

esp_err_t buzzer_play_alarm(buzzer_alarm_reason_t reason);

#endif // BUZZER_H
