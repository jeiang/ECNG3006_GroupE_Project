/**
 * @file measurement.h
 * @brief Task to handle getting measurements from the ADC.
 *
 * @author Aidan Pinard (jeiang)
 * @bug No known bugs.
 */

#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "ads1115.h"

/**
 * @brief Max frequency ~3Hz (180bpm). Resolution of wave will be 1/10 the
 * wavelength.
 */
#define MEASUREMENT_PERIOD     (pdMS_TO_TICKS(33))
#define MEASUREMENT_PRIORITY   11
#define MEASUREMENT_STACK_SIZE 2048
#define MEASUREMENT_I2C_SDA    0
#define MEASUREMENT_I2C_SCL    2
#define MEASUREMENT_I2C_PORT   I2C_NUM_0

typedef struct
{
    int16_t red_reading;
    int16_t infrared_reading;
} measurement_value_t;

static void measurement_configure_ads1115(const i2c_config_t     *config,
                                          const ads1115_handle_t *handle);

static esp_err_t measurement_read_from_ads1115(const ads1115_handle_t *handle,
                                               ads1115_mux_t mux_selection,
                                               int16_t      *result);

static void measurement_measure_task(void *pvParameters);

TaskHandle_t measurement_init(uint32_t queue_size, QueueHandle_t *queue);

#endif // MEASUREMENT_H
