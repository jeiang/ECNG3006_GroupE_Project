/**
 * @file measurement.c
 * @brief Task to handle getting measurements from the ADC.
 *
 * @author Aidan Pinard (jeiang)
 * @bug No known bugs.
 */

#include "measurement.h"
#include "ads1115.h"

static void
measurement_configure_ads1115(const i2c_config_t     *config,
                              const ads1115_handle_t *handle)
{
    ESP_ERROR_CHECK(i2c_driver_install(MEASUREMENT_I2C_PORT, config->mode));
    ESP_ERROR_CHECK(i2c_param_config(MEASUREMENT_I2C_PORT, config));

    ads1115_set_full_scale_gain(handle, ADS1115_PGAFSR_4096);

    i2c_driver_delete(MEASUREMENT_I2C_PORT);
}

static esp_err_t
measurement_read_from_ads1115(const ads1115_handle_t *handle,
                              ads1115_mux_t           mux_selection,
                              int16_t                *result)
{
    ads1115_op_status_t conversion_status;
    ads1115_set_input_pins(handle, mux_selection);
    ads1115_start_single_shot_conversion(handle);

    do
    {
        ads1115_get_conversion_status(handle, &conversion_status);
    } while (conversion_status != ADS1115_OS_FREE_START);

    return ads1115_read_conversion_result(handle, result);
}

static void
measurement_measure_task(void *pvParameters)
{
    QueueHandle_t *queue          = (QueueHandle_t *)pvParameters;
    TickType_t     last_wake_time = xTaskGetTickCount();

    const i2c_config_t config = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = MEASUREMENT_I2C_SDA,
        .sda_pullup_en    = 1,
        .scl_io_num       = MEASUREMENT_I2C_SCL,
        .scl_pullup_en    = 1,
        .clk_stretch_tick = 300, // 300 ticks, Clock stretch is about 210us
    };

    const ads1115_handle_t handle = { .ads1115_address = ADS1115_ADDR_GND,
                                      .i2c_port        = MEASUREMENT_I2C_PORT };

    measurement_configure_ads1115(&config, &handle);
    measurement_value_t reading = {
        .infrared_reading = 0,
        .red_reading      = 0,
    };
    for (;;)
    {
        last_wake_time = xTaskGetTickCount();
        ESP_ERROR_CHECK(i2c_driver_install(MEASUREMENT_I2C_PORT, config.mode));
        ESP_ERROR_CHECK(i2c_param_config(MEASUREMENT_I2C_PORT, &config));

        measurement_read_from_ads1115(
            &handle, ADS1115_MUX_AIN1_GND, &(reading.infrared_reading));
        measurement_read_from_ads1115(
            &handle, ADS1115_MUX_AIN3_GND, &(reading.red_reading));

        i2c_driver_delete(MEASUREMENT_I2C_PORT);
        vTaskDelayUntil(&last_wake_time, MEASUREMENT_PERIOD);
    }
}

TaskHandle_t
measurement_init(uint32_t queue_size, QueueHandle_t *queue)
{
    *queue = xQueueCreate(queue_size, sizeof(measurement_value_t));

    TaskHandle_t handle;
    xTaskCreate(measurement_measure_task,
                "measurement_measure_task",
                MEASUREMENT_STACK_SIZE,
                (void *)queue,
                MEASUREMENT_PRIORITY,
                &handle);
    return handle;
}
