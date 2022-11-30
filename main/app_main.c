/**
 * @file app_main.c
 * @brief Entrypoint of program.
 *
 * @author Aidan Pinard (jeiang)
 * @bug No known bugs.
 */

#include "app_main.h"

#ifdef CONFIG_RUN_UNIT_TESTS
static void
unityTask(void *pvParameters)
{
    vTaskDelay(2);    /* Delay a bit to let the main task be deleted */
    unity_run_menu(); /* Doesn't return */
}
#endif // CONFIG_RUN_UNIT_TESTS

void
app_main(void)
{
#ifdef CONFIG_RUN_UNIT_TESTS
    // Note: if unpinning this task, change the way run times are calculated in
    // unity_platform
    xTaskCreatePinnedToCore(unityTask,
                            "unityTask",
                            UNITY_FREERTOS_STACK_SIZE,
                            NULL,
                            UNITY_FREERTOS_PRIORITY,
                            NULL,
                            UNITY_FREERTOS_CPU);
#else
    user_main();
#endif // CONFIG_RUN_UNIT_TESTS
}
