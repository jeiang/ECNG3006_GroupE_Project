/**
 * @file buzzer.c
 * @brief Entrypoint for default operation. Should be called from app_main.
 *
 * @author Aidan Pinard (jeiang)
 * @bug No known bugs.
 */

#include "buzzer.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "buzzer";

/**
 * @brief Notes for each alarm
 */
static const uint32_t buzzer_alarm_pattern[][BUZZER_ALARM_NOTE_COUNT] = { 
    [BUZZER_ALARM_HEART] = {
        BUZZER_C4_PERIOD_US,
        BUZZER_E4_PERIOD_US,
        BUZZER_G4_PERIOD_US,
        BUZZER_G4_PERIOD_US,
        BUZZER_C5_PERIOD_US,
        BUZZER_C4_PERIOD_US,
        BUZZER_E4_PERIOD_US,
        BUZZER_G4_PERIOD_US,
        BUZZER_G4_PERIOD_US,
        BUZZER_C5_PERIOD_US,
    },
    [BUZZER_ALARM_OXYGEN] = {
        BUZZER_C5_PERIOD_US,
        BUZZER_B4_PERIOD_US,
        BUZZER_A4_PERIOD_US,
        BUZZER_G4_PERIOD_US,
        BUZZER_F4_PERIOD_US,
        BUZZER_C5_PERIOD_US,
        BUZZER_B4_PERIOD_US,
        BUZZER_A4_PERIOD_US,
        BUZZER_G4_PERIOD_US,
        BUZZER_F4_PERIOD_US,
    } };

/**
 * @brief Timings between each note that is played.
 */
static const uint32_t buzzer_alarm_note_off_ms[BUZZER_ALARM_NOTE_COUNT]
    = { 150, 150, 400, 150, 2000, 150, 150, 400, 150, 500 };

static const char *
buzzer_period_us_to_note(uint32_t period_us)
{
    switch (period_us)
    {
        case BUZZER_A4_PERIOD_US:
            return "A4";
        case BUZZER_B4_PERIOD_US:
            return "B4";
        case BUZZER_C4_PERIOD_US:
            return "C4";
        case BUZZER_C5_PERIOD_US:
            return "C5";
        case BUZZER_E4_PERIOD_US:
            return "E4";
        case BUZZER_F4_PERIOD_US:
            return "F4";
        case BUZZER_G4_PERIOD_US:
            return "G4";
        default:
            return "";
    }
}

esp_err_t
buzzer_play_alarm(buzzer_alarm_reason_t reason)
{
    uint32_t duties[1]  = { 0 };
    uint32_t pwm_period = 0;
    for (uint32_t note_num = 0; note_num < BUZZER_ALARM_NOTE_COUNT; note_num++)
    {
        pwm_period = buzzer_alarm_pattern[reason][note_num];
        duties[0]  = pwm_period / 2;
        BUZZER_ESP_CHECK(pwm_set_period(pwm_period));
        BUZZER_ESP_CHECK(pwm_set_duties(duties));
        ESP_LOGI(TAG, "Playing %s.", buzzer_period_us_to_note(pwm_period));
        BUZZER_ESP_CHECK(pwm_start());
        vTaskDelay(BUZZER_ALARM_NOTE_PLAY_TICKS);
        BUZZER_ESP_CHECK(pwm_stop(0x00));
        vTaskDelay(pdMS_TO_TICKS(buzzer_alarm_note_off_ms[note_num]));
    }
    return ESP_OK;
}
