/**
 * @file app_main.h
 * @brief Entrypoint of program.
 *
 * @author Aidan Pinard (jeiang)
 * @bug No known bugs.
 */

#ifndef APPMAIN_H
#define APPMAIN_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "unity.h"
#include "unity_config.h"
#include "user_main.h"

#ifdef CONFIG_RUN_UNIT_TESTS

/**
 * @brief Launches the unity test menu.
 * @param pvParameters parameters passed by xTaskCreate
 */
static void unityTask(void *pvParameters);

#endif // CONFIG_RUN_UNIT_TESTS

/**
 * @brief Entrypoint.
 */
void app_main(void);

#endif // APPMAIN_H
