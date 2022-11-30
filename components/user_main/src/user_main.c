/**
 * @file user_main.c
 * @brief Entrypoint for default operation. Should be called from app_main.
 *
 * @author Aidan Pinard (jeiang)
 * @bug No known bugs.
 */

#include "user_main.h"

static const char    *TAG           = "user_main";
static const uint32_t FIRST_NUMBER  = 1;
static const uint32_t SECOND_NUMBER = 2;

uint32_t
user_main_add_two_numbers(uint32_t first, uint32_t second)
{
    return first + second;
}

void
user_main()
{
    uint32_t sum = user_main_add_two_numbers(FIRST_NUMBER, SECOND_NUMBER);
    ESP_LOGI(
        TAG, "The sum of %i and %i is %i.", FIRST_NUMBER, SECOND_NUMBER, sum);
}
