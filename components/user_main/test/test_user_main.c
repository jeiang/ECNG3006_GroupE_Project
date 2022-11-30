/**
 * @file test_user_main.c
 * @brief Unit tests for user_main component.
 *
 * @author Aidan Pinard (jeiang)
 * @bug No known bugs.
 */

//#include "test_user_main.h"

#include "unity.h"
#include "user_main.h"
#include "unity_config.h"
#include "test_utils.h"
#include <stdint.h>
#include "esp_log.h"

TEST_CASE("test adding 1 and 2", "[user_main]")
{
    uint32_t sum = user_main_add_two_numbers(1, 2);
    TEST_ASSERT_EQUAL_INT32(3, sum);
}

TEST_CASE("test adding 10000000 and 100000000", "[user_main]")
{
  uint32_t sum = user_main_add_two_numbers(10000000, 100000000);
  TEST_ASSERT_EQUAL_UINT32(110000000, sum);
}
