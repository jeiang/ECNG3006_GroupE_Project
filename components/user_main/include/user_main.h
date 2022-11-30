/**
 * @file user_main.h
 * @brief Entrypoint for default operation. Should be called from app_main.
 *
 * @author Aidan Pinard (jeiang)
 * @bug No known bugs.
 */

#ifndef USERMAIN_H
#define USERMAIN_H

#include <stdint.h>
#include <esp_log.h>

/**
 * @brief Adds two numbers together and returns the sum
 * @param first one of the numbers to sum
 * @param second the other number to sum
 * @return the sum of first and second
 */
uint32_t user_main_add_two_numbers(uint32_t first, uint32_t second);

/**
 * @brief User defined main function.
 */
void user_main(void);

#endif // USERMAIN_H
