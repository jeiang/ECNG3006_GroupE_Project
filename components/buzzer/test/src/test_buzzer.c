/**
 * @file test_user_main.c
 * @brief Unit tests for user_main component.
 *
 * @author Aidan Pinard (jeiang)
 * @bug No known bugs.
 */

#include "test_buzzer.h"

static const char *TAG = "test_buzzer";

void
test_buzzer_play_alarm(buzzer_alarm_reason_t reason)
{
    esp_err_t rc;
    char      buffer[100];
    uint32_t  pwm_period = 11; // Minimum is 11, just to init pwm
    uint32_t  duties[1]  = { 0 };
    float     phase[1]   = { 0.0 };
    uint32_t  pin_num    = BUZZER_OUTPUT_PIN2;

    rc = pwm_init(pwm_period, duties, 1, &pin_num);
    sprintf(buffer, "Expected ESP_OK, got %s instead.", esp_err_to_name(rc));
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, rc, buffer);
    ESP_LOGI(TAG, "Successfully init PWM.");
    pwm_set_phases(phase);
    for (uint32_t count = 0; count < 3; count++)
    {
        rc = buzzer_play_alarm(reason);
        sprintf(
            buffer, "Expected ESP_OK, got %s instead.", esp_err_to_name(rc));
        TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, rc, buffer);
        ESP_LOGI(TAG, "Successfully played alarm using PWM.");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    rc = pwm_deinit();
    sprintf(buffer, "Expected ESP_OK, got %s instead.", esp_err_to_name(rc));
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, rc, buffer);
    ESP_LOGI(TAG, "Successfully deinit PWM.");
}

TEST_CASE("play heart alarm on buzzer (GPIO2)", "[buzzer]")
{
    ESP_LOGI(TAG, "Testing Heart Alarm");
    test_buzzer_play_alarm(BUZZER_ALARM_HEART);
}

TEST_CASE("play oxygen alarm on buzzer (GPIO2)", "[buzzer]")
{
    ESP_LOGI(TAG, "Testing Oxygen Alarm");
    test_buzzer_play_alarm(BUZZER_ALARM_OXYGEN);
}
