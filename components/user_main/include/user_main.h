/**
 * @file user_main.h
 * @brief Entrypoint for default operation. Should be called from app_main
 * (assuming not in unit mode).
 *
 * @author Aidan Pinard (jeiang)
 * @bug No known bugs.
 */

#ifndef USERMAIN_H
#define USERMAIN_H

#include <stdint.h>
#include <esp_log.h>

/**
 * @brief User defined main function.
 */
void user_main(void);

#endif // USERMAIN_H
