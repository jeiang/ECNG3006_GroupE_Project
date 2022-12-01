/**
 * @file ads1115.c
 * @brief Library to use ADS1115 ADCs over the I2C interface.
 *
 * @author Aidan Pinard (jeiang)
 * @bug No known bugs.
 */

#include "ads1115.h"

/**
 * @brief buffer used to cache 2 bytes for a 16 bit integer
 */
static uint8_t ads1115_read_uint16_buffer[2] = { 0, 0 };

/**
 * @brief conversion array from ads1115_full_scale_t to voltages.
 */
static const double ads1115_full_scale_values[]
    = { [ADS1115_PGAFSR_6144] = 6.144,  [ADS1115_PGAFSR_4096] = 4.096,
        [ADS1115_PGAFSR_2048] = 2.048,  [ADS1115_PGAFSR_1024] = 1.024,
        [ADS1115_PGAFSR_512] = 0.512,   [ADS1115_PGAFSR_256] = 0.256,
        [ADS1115_PGAFSR_256_2] = 0.256, [ADS1115_PGAFSR_256_3] = 0.256 };

static esp_err_t
ads1115_read_register(ads1115_handle_t  *handle,
                      ads1115_register_t reg,
                      uint16_t          *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,
                          (handle->ads1115_address << 1) | I2C_MASTER_WRITE,
                          ADS1115_CHECK_ACK_ENABLE);
    i2c_master_write_byte(cmd, ADS1115_CONFIG_REG, ADS1115_CHECK_ACK_ENABLE);
    i2c_master_stop(cmd);
    esp_err_t ret
        = i2c_master_cmd_begin(handle->i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK)
    {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd,
                              (handle->ads1115_address << 1) | I2C_MASTER_READ,
                              ADS1115_CHECK_ACK_ENABLE);
        i2c_master_read(
            cmd, ads1115_read_uint16_buffer, 2, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(
            handle->i2c_port, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK)
        {
            *data = (((uint16_t)ads1115_read_uint16_buffer[1]) << 8)
                    | ((uint16_t)ads1115_read_uint16_buffer[0]);
        }
    }
    return ret;
}

static esp_err_t
ads1115_write_register(ads1115_handle_t  *handle,
                       ads1115_register_t reg,
                       uint16_t           data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,
                          handle->ads1115_address << 1 | I2C_MASTER_WRITE,
                          ADS1115_CHECK_ACK_ENABLE);
    i2c_master_write_byte(cmd, ADS1115_CONFIG_REG, ADS1115_CHECK_ACK_ENABLE);
    i2c_master_write_byte(cmd, (data >> 8) & 0x00FF, ADS1115_CHECK_ACK_ENABLE);
    i2c_master_write_byte(cmd, data & 0x00FF, ADS1115_CHECK_ACK_ENABLE);
    i2c_master_stop(cmd);
    esp_err_t ret
        = i2c_master_cmd_begin(handle->i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t
ads1115_read_conf_reg_bits(ads1115_handle_t *handle,
                           uint8_t           offset,
                           uint8_t           mask,
                           uint8_t          *data)
{
    uint16_t  result;
    esp_err_t ret = ads1115_read_register(handle, ADS1115_CONFIG_REG, &result);
    if (ret == ESP_OK)
    {
        *data = (result >> offset) & mask;
    }
    return ret;
}

static esp_err_t
ads1115_write_conf_reg_bits(ads1115_handle_t *handle,
                            uint8_t           offset,
                            uint8_t           mask,
                            uint8_t           data)
{
    uint16_t  result;
    esp_err_t ret = ads1115_read_register(handle, ADS1115_CONFIG_REG, &result);
    if (ret == ESP_OK)
    {
        uint16_t conf_value = result & ~(mask << offset);
        conf_value |= data << offset;
        ret = ads1115_write_register(handle, ADS1115_CONFIG_REG, conf_value);
    }
    return ret;
}

esp_err_t
ads1115_start_single_shot_conversion(ads1115_handle_t *handle)
{
    return ads1115_write_conf_reg_bits(
        handle, ADS1115_OS_OFFSET, ADS1115_OS_MASK, ADS1115_OS_FREE_START);
}
esp_err_t
ads1115_get_conversion_status(ads1115_handle_t    *handle,
                              ads1115_op_status_t *status)
{
    uint8_t   data;
    esp_err_t ret = ads1115_read_conf_reg_bits(
        handle, ADS1115_OS_OFFSET, ADS1115_OS_MASK, &data);
    *status = (ads1115_op_status_t)data;
    return ret;
}

esp_err_t
ads1115_set_input_pins(ads1115_handle_t *handle, ads1115_mux_t selected_pins)
{
    return ads1115_write_conf_reg_bits(
        handle, ADS1115_MUX_OFFSET, ADS1115_MUX_MASK, (uint8_t)selected_pins);
}
esp_err_t
ads1115_get_input_pins(ads1115_handle_t *handle, ads1115_mux_t *selected_pins)
{
    uint8_t   pins;
    esp_err_t ret = ads1115_read_conf_reg_bits(
        handle, ADS1115_MUX_OFFSET, ADS1115_MUX_MASK, &pins);
    *selected_pins = (ads1115_mux_t)pins;
    return ret;
}

esp_err_t
ads1115_set_full_scale_gain(ads1115_handle_t    *handle,
                            ads1115_full_scale_t full_scale_gain)
{
    return ads1115_write_conf_reg_bits(
        handle, ADS1115_MUX_OFFSET, ADS1115_MUX_MASK, (uint8_t)full_scale_gain);
}
esp_err_t
ads1115_get_full_scale_gain(ads1115_handle_t     *handle,
                            ads1115_full_scale_t *full_scale_gain)
{
    uint8_t   gain;
    esp_err_t ret = ads1115_read_conf_reg_bits(
        handle, ADS1115_MUX_OFFSET, ADS1115_MUX_MASK, &gain);
    *full_scale_gain = (ads1115_full_scale_t)gain;
    return ret;
}

esp_err_t
ads1115_set_mode(ads1115_handle_t *handle, ads1115_op_mode_t mode)
{
    return ads1115_write_conf_reg_bits(
        handle, ADS1115_MUX_OFFSET, ADS1115_MUX_MASK, (uint8_t)mode);
}
esp_err_t
ads1115_get_mode(ads1115_handle_t *handle, ads1115_op_mode_t *mode)
{
    uint8_t   raw_mode;
    esp_err_t ret = ads1115_read_conf_reg_bits(
        handle, ADS1115_MUX_OFFSET, ADS1115_MUX_MASK, &raw_mode);
    *mode = (ads1115_op_mode_t)raw_mode;
    return ret;
}

esp_err_t
ads1115_set_data_rate(ads1115_handle_t *handle, ads1115_data_rate_t data_rate)
{
    return ads1115_write_conf_reg_bits(
        handle, ADS1115_MUX_OFFSET, ADS1115_MUX_MASK, (uint8_t)data_rate);
}
esp_err_t
ads1115_get_data_rate(ads1115_handle_t *handle, ads1115_data_rate_t *data_rate)
{
    uint8_t   raw_data_rate;
    esp_err_t ret = ads1115_read_conf_reg_bits(
        handle, ADS1115_MUX_OFFSET, ADS1115_MUX_MASK, &raw_data_rate);
    *data_rate = (ads1115_data_rate_t)raw_data_rate;
    return ret;
}

esp_err_t
ads1115_set_comparator_mode(ads1115_handle_t *handle, ads1115_comp_mode_t mode)
{
    return ads1115_write_conf_reg_bits(
        handle, ADS1115_MUX_OFFSET, ADS1115_MUX_MASK, (uint8_t)mode);
}
esp_err_t
ads1115_get_comparator_mode(ads1115_handle_t *handle, ads1115_comp_mode_t *mode)
{
    uint8_t   raw_mode;
    esp_err_t ret = ads1115_read_conf_reg_bits(
        handle, ADS1115_MUX_OFFSET, ADS1115_MUX_MASK, &raw_mode);
    *mode = (ads1115_comp_mode_t)raw_mode;
    return ret;
}

esp_err_t
ads1115_set_comparator_polarity(ads1115_handle_t       *handle,
                                ads1115_comp_polarity_t polarity)
{
    return ads1115_write_conf_reg_bits(
        handle, ADS1115_MUX_OFFSET, ADS1115_MUX_MASK, (uint8_t)polarity);
}
esp_err_t
ads1115_get_comparator_polarity(ads1115_handle_t        *handle,
                                ads1115_comp_polarity_t *polarity)
{
    uint8_t   raw_polarity;
    esp_err_t ret = ads1115_read_conf_reg_bits(
        handle, ADS1115_MUX_OFFSET, ADS1115_MUX_MASK, &raw_polarity);
    *polarity = (ads1115_comp_polarity_t)raw_polarity;
    return ret;
}

esp_err_t
ads1115_set_comparator_latch(ads1115_handle_t    *handle,
                             ads1115_comp_latch_t latch)
{
    return ads1115_write_conf_reg_bits(
        handle, ADS1115_MUX_OFFSET, ADS1115_MUX_MASK, (uint8_t)latch);
}
esp_err_t
ads1115_get_comparator_latch(ads1115_handle_t     *handle,
                             ads1115_comp_latch_t *latch)
{
    uint8_t   raw_latch;
    esp_err_t ret = ads1115_read_conf_reg_bits(
        handle, ADS1115_MUX_OFFSET, ADS1115_MUX_MASK, &raw_latch);
    *latch = (ads1115_comp_latch_t)raw_latch;
    return ret;
}

esp_err_t
ads1115_set_comparator_queue(ads1115_handle_t    *handle,
                             ads1115_comp_queue_t queue)
{
    return ads1115_write_conf_reg_bits(
        handle, ADS1115_MUX_OFFSET, ADS1115_MUX_MASK, (uint8_t)queue);
}
esp_err_t
ads1115_get_comparator_queue(ads1115_handle_t     *handle,
                             ads1115_comp_queue_t *queue)
{
    uint8_t   raw_queue;
    esp_err_t ret = ads1115_read_conf_reg_bits(
        handle, ADS1115_MUX_OFFSET, ADS1115_MUX_MASK, &raw_queue);
    *queue = (ads1115_comp_queue_t)raw_queue;
    return ret;
}

esp_err_t
ads1115_read_conversion_result(ads1115_handle_t *handle, int16_t *result)
{
    uint16_t  raw_result;
    esp_err_t ret
        = ads1115_read_register(handle, ADS1115_CONVERSION_REG, &raw_result);
    if (raw_result <= 0x7FFF) // positive two's complement
    {
        *result = (int16_t)raw_result;
    }
    else // negative two's complement
    {
        // manual conversion to negative, not sure of internal implementation
        *result = -((uint16_t)(~raw_result + 1));
    }
    return ret;
}

esp_err_t
ads1115_estimate_absolute_reading(ads1115_handle_t *handle, double *result)
{
    ads1115_full_scale_t full_scale_setting;
    esp_err_t ret = ads1115_get_full_scale_gain(handle, &full_scale_setting);
    if (ret == ESP_OK)
    {
        int16_t raw_result;
        ret     = ads1115_read_conversion_result(handle, &raw_result);
        *result = ((double)raw_result)
                  * ads1115_full_scale_values[full_scale_setting];
    }
    return ret;
}

esp_err_t
ads1115_get_low_threshold(ads1115_handle_t *handle, int16_t *result)
{
    uint16_t  raw_result;
    esp_err_t ret
        = ads1115_read_register(handle, ADS1115_LO_THRESH_REG, &raw_result);
    if (raw_result <= 0x7FFF) // positive two's complement
    {
        *result = (int16_t)raw_result;
    }
    else // negative two's complement
    {
        // manual conversion to negative, not sure of internal implementation
        *result = -((uint16_t)(~raw_result + 1));
    }
    return ret;
}

esp_err_t
ads1115_set_low_threshold(ads1115_handle_t *handle, int16_t result)
{
    uint16_t two_comp_result;
    if (result < 0)
    {
        // convert to two's complement
        two_comp_result = ~((uint16_t)(abs(result))) + 1;
    }
    else
    {
        two_comp_result = (uint16_t)result;
    }
    return ads1115_write_register(
        handle, ADS1115_LO_THRESH_REG, two_comp_result);
}

esp_err_t
ads1115_get_high_threshold(ads1115_handle_t *handle, int16_t *result)
{
    uint16_t  raw_result;
    esp_err_t ret
        = ads1115_read_register(handle, ADS1115_HI_THRESH_REG, &raw_result);
    if (raw_result <= 0x7FFF) // positive two's complement
    {
        *result = (int16_t)raw_result;
    }
    else // negative two's complement
    {
        // manual conversion to negative, not sure of internal implementation
        *result = -((uint16_t)(~raw_result + 1));
    }
    return ret;
}

esp_err_t
ads1115_set_high_threshold(ads1115_handle_t *handle, int16_t result)
{
    uint16_t two_comp_result;
    if (result < 0)
    {
        // convert to two's complement
        two_comp_result = ~((uint16_t)(abs(result))) + 1;
    }
    else
    {
        two_comp_result = (uint16_t)result;
    }
    return ads1115_write_register(
        handle, ADS1115_HI_THRESH_REG, two_comp_result);
}
