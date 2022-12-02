/**
 * @file ads1115.h
 * @brief Library to use ADS1115 ADCs over the I2C interface.
 *
 * @author Aidan Pinard (jeiang)
 * @bug No known bugs.
 */

#ifndef ADS111X_H
#define ADS111X_H

#include "esp_err.h"
#include "driver/i2c.h"

#define ADS1115_CHECK_ACK_ENABLE 1
#define ADS1115_COMP_QUE_OFFSET  1
#define ADS1115_COMP_QUE_MASK    0x03
#define ADS1115_COMP_LAT_OFFSET  2
#define ADS1115_COMP_LAT_MASK    0x01
#define ADS1115_COMP_POL_OFFSET  3
#define ADS1115_COMP_POL_MASK    0x01
#define ADS1115_COMP_MODE_OFFSET 4
#define ADS1115_COMP_MODE_MASK   0x01
#define ADS1115_DATA_RATE_OFFSET 5
#define ADS1115_DATA_RATE_MASK   0x07
#define ADS1115_MODE_OFFSET      8
#define ADS1115_MODE_MASK        0x01
#define ADS1115_PGA_OFFSET       9
#define ADS1115_PGA_MASK         0x07
#define ADS1115_MUX_OFFSET       12
#define ADS1115_MUX_MASK         0x07
#define ADS1115_OS_OFFSET        15
#define ADS1115_OS_MASK          0x01

/**
 * @brief I2C address for the ADS1115 based on ADDR pin connection
 * @see ADS1115_ADDR_GND
 * @see ADS1115_ADDR_VDD
 * @see ADS1115_ADDR_SDA
 * @see ADS1115_ADDR_SCL
 */
typedef enum
{
    /**
     * @brief I2C Address used when the ADDR pin is connected to the GND pin
     */
    ADS1115_ADDR_GND = 0b1001000,
    /**
     * @brief I2C Address used when the ADDR pin is connected to the VDD pin
     */
    ADS1115_ADDR_VDD = 0b1001001,
    /**
     * @brief I2C Address used when the ADDR pin is connected to the SDA pin
     */
    ADS1115_ADDR_SDA = 0b1001010,
    /**
     * @brief I2C Address used when the ADDR pin is connected to the SCL pin
     */
    ADS1115_ADDR_SCL = 0b1001011,
} ads1115_addr_t;

/**
 * @brief ADS1115 register values to select a register to read from/write to
 * @see ADS1115_CONVERSION_REG
 * @see ADS1115_CONFIG_REG
 * @see ADS1115_LO_THRESH_REG
 * @see ADS1115_HI_THRESH_REG
 */
typedef enum
{
    /**
     * @brief Register storing conversion output
     */
    ADS1115_CONVERSION_REG = 0x0,
    /**
     * @brief Register for configuration of the ADS1115
     */
    ADS1115_CONFIG_REG = 0x1,
    /**
     * @brief Register for the comparator which unsets the ALERT/RDY pin when
     * the conversion value falls below the value in this register
     */
    ADS1115_LO_THRESH_REG = 0x2,
    /**
     * @brief Register for the comparator which sets the ALERT/RDY pin when the
     * conversion value exceeds the value in this register
     */
    ADS1115_HI_THRESH_REG = 0x3,
} ads1115_register_t;

/**
 * @brief When reading, has current operating status. When writing, starts a
 * conversion in single shot mode.
 * @see ADS1115_OS_DISABLED
 * @see ADS1115_OS_ENABLED
 */
typedef enum
{
    /**
     * @brief When reading, device is performing a conversion. When writing,
     * no effect.
     */
    ADS1115_OS_BUSY = 0,
    /**
     * @brief When reading, device is not performing a conversion. When writing,
     * starts a conversion in single shot mode.
     */
    ADS1115_OS_FREE_START,
} ads1115_op_status_t;

/**
 * @brief Configure the input multiplexer of the ADS1115
 * @see ADS1115_MUX_AIN0_AIN1
 * @see ADS1115_MUX_AIN0_AIN3
 * @see ADS1115_MUX_AIN1_AIN3
 * @see ADS1115_MUX_AIN2_AIN3
 * @see ADS1115_MUX_AIN0_GND
 * @see ADS1115_MUX_AIN1_GND
 * @see ADS1115_MUX_AIN2_GND
 * @see ADS1115_MUX_AIN3_GND
 */
typedef enum
{
    /**
     * @brief Input is measured across AIN0 (positive) and AIN1 (negative).
     */
    ADS1115_MUX_AIN0_AIN1 = 0,
    /**
     * @brief Input is measured across AIN0 (positive) and AIN3 (negative).
     */
    ADS1115_MUX_AIN0_AIN3,
    /**
     * @brief Input is measured across AIN1 (positive) and AIN3 (negative).
     */
    ADS1115_MUX_AIN1_AIN3,
    /**
     * @brief Input is measured across AIN2 (positive) and AIN3 (negative).
     */
    ADS1115_MUX_AIN2_AIN3,
    /**
     * @brief Input is measured across AIN0 (positive) and GND (negative).
     */
    ADS1115_MUX_AIN0_GND,
    /**
     * @brief Input is measured across AIN1 (positive) and GND (negative).
     */
    ADS1115_MUX_AIN1_GND,
    /**
     * @brief Input is measured across AIN2 (positive) and GND (negative).
     */
    ADS1115_MUX_AIN2_GND,
    /**
     * @brief Input is measured across AIN3 (positive) and GND (negative).
     */
    ADS1115_MUX_AIN3_GND,
} ads1115_mux_t;

/**
 * @brief Set the full scale range of the programmable gain amplifier. Does not
 * affect the maximum input of the ADS1115. It changes the scaling of the
 * converted 16 bit value.
 * @see ADS1115_PGAFSR_6144
 * @see ADS1115_PGAFSR_4096
 * @see ADS1115_PGAFSR_2048
 * @see ADS1115_PGAFSR_1024
 * @see ADS1115_PGAFSR_512
 * @see ADS1115_PGAFSR_256
 */
typedef enum
{
    /**
     * @brief Full scale range = +- 6.144 V
     */
    ADS1115_PGAFSR_6144 = 0,
    /**
     * @brief Full scale range = +- 4.096 V
     */
    ADS1115_PGAFSR_4096,
    /**
     * @brief Full scale range = +- 2.048 V
     */
    ADS1115_PGAFSR_2048,
    /**
     * @brief Full scale range = +- 1.024 V
     */
    ADS1115_PGAFSR_1024,
    /**
     * @brief Full scale range = +- 512 mV
     */
    ADS1115_PGAFSR_512,
    /**
     * @brief Full scale range = +- 256 mV
     */
    ADS1115_PGAFSR_256,
    /**
     * @brief Full scale range = +- 256 mV, alternative option, same as @see
     * ADS1115_PGAFSR_256
     */
    ADS1115_PGAFSR_256_2,
    /**
     * @brief Full scale range = +- 256 mV, alternative option, same as @see
     * ADS1115_PGAFSR_256
     */
    ADS1115_PGAFSR_256_3,
} ads1115_full_scale_t;

/**
 * @brief Selects when conversions are performed by the ADS1115.
 */
typedef enum
{
    /**
     * @brief The ADS1115 will continuously read the input analog value, and
     * update the conversion register each time.
     */
    ADS1115_MODE_CONTINUOUS = 0,
    /**
     * @brief The ADS1115 will only read the input analog value when the
     * operating status is set.
     */
    ADS1115_MODE_SINGLE = 1,
} ads1115_op_mode_t;

/**
 * @brief The data rate. Controls the continuous conversion rate and the number
 * of samples for single shot.
 */
typedef enum
{
    /**
     * @brief 8 samples per second
     */
    ADS1115_DATA_RATE_8SPS = 0,
    /**
     * @brief 16 samples per second
     */
    ADS1115_DATA_RATE_16SPS,
    /**
     * @brief 32 samples per second
     */
    ADS1115_DATA_RATE_32SPS,
    /**
     * @brief 64 samples per second
     */
    ADS1115_DATA_RATE_64SPS,
    /**
     * @brief 128 samples per second
     */
    ADS1115_DATA_RATE_128SPS,
    /**
     * @brief 250 samples per second
     */
    ADS1115_DATA_RATE_250SPS,
    /**
     * @brief 475 samples per second
     */
    ADS1115_DATA_RATE_475SPS,
    /**
     * @brief 860 samples per second
     */
    ADS1115_DATA_RATE_860SPS,
} ads1115_data_rate_t;

/**
 * @brief Selects the comparator mode of the ADS1115.
 */
typedef enum
{
    /**
     * @brief The ALERT/RDY pin is set when the conversion result is between the
     * value in LO_THRESH and HI_THRESH.
     */
    ADS1115_COMP_TRADITIONAL = 0,
    /**
     * @brief The ALERT/RDY pin is set when the conversion result is either
     * below value in LO_THRESH or above the value in HI_THRESH.
     */
    ADS1115_COMP_WINDOW,
} ads1115_comp_mode_t;

/**
 * @brief Selects whether the ALERT/RDY pin is active high or active low.
 */
typedef enum
{
    /**
     * @brief Select active low ALERT/RDY pin.
     */
    ADS1115_COMP_ACTIVE_LOW = 0,
    /**
     * @brief Select active high ALERT/RDY pin.
     */
    ADS1115_COMP_ACTIVE_HIGH,
} ads1115_comp_polarity_t;

/**
 * @brief Selects whether the ALERT/RDY pin latches when set.
 */
typedef enum
{
    /**
     * @brief The ALERT/RDY pin does not latch when asserted.
     */
    ADS1115_COMP_LATCH_ENABLED = 0,
    /**
     * @brief The asserted ALERT/RDY pin remains latched until conversion data
     * are read by the master or an appropriate SMBus alert response is sent by
     * the master.
     */
    ADS1115_COMP_LATCH_DISABLED,
} ads1115_comp_latch_t;

/**
 * @brief Select how many conversions are required before ALERT/RDY is set.
 */
typedef enum
{
    /**
     * @brief Set after 1 conversion.
     */
    ADS1115_COMP_QUEUE_1 = 0,
    /**
     * @brief Set after 2 conversions.
     */
    ADS1115_COMP_QUEUE_2,
    /**
     * @brief Set after 4 conversions.
     */
    ADS1115_COMP_QUEUE_3,
    /**
     * @brief Disable comparator and do not set ALERT/RDY.
     */
    ADS1115_COMP_QUEUE_DISABLED,
} ads1115_comp_queue_t;

/**
 * @struct ads1115_handle_t
 * @brief Handle for the ADS1115.
 * @var ads1115_handle_t::i2c_port I2C on the ESP8266 port which is configured
 * already.
 * @var ads1115_handle_t::ads1115_address I2C address of the ADS1115
 */
typedef struct
{
    i2c_port_t     i2c_port;
    ads1115_addr_t ads1115_address;
} ads1115_handle_t;

/**
 * @brief Read a value from a register on the ADS1115
 * @param handle handle to the port and address for the ADS1115
 * @param reg register to read from
 * @param data data stored in the register
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
static esp_err_t ads1115_read_register(const ads1115_handle_t *handle,
                                       ads1115_register_t      reg,
                                       uint16_t               *data);

/**
 * @brief Write to a register on the ADS1115
 * @param handle handle to the port and address for the ADS1115
 * @param reg register to write to
 * @param value data to write to the register
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
static esp_err_t ads1115_write_register(const ads1115_handle_t *handle,
                                        ads1115_register_t      reg,
                                        uint16_t                data);

/**
 * @brief Read only certain bits from the config register
 * @param handle handle to the port and address for the ADS1115
 * @param offset offset of the lsb of the desired config setting
 * @param mask mask to select the number of bits of the config setting
 * @param data the value of the config setting
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
static esp_err_t ads1115_read_conf_reg_bits(const ads1115_handle_t *handle,
                                            uint8_t                 offset,
                                            uint8_t                 mask,
                                            uint8_t                *data);

/**
 * @brief Write to only certain bits of the config register
 * @param handle handle to the port and address for the ADS1115
 * @param offset offset of the lsb of the desired config setting
 * @param mask mask to set which bits will be overwritten by data
 * @param data bits to write to the config register
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
static esp_err_t ads1115_write_conf_reg_bits(const ads1115_handle_t *handle,
                                             uint8_t                 offset,
                                             uint8_t                 mask,
                                             uint8_t                 data);

/**
 * @brief Start a one shot conversion
 * @param handle handle to the port and address for the ADS1115
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t ads1115_start_single_shot_conversion(const ads1115_handle_t *handle);

/**
 * @brief Check whether the ADS1115 is currently converting an analog reading.
 * @param handle handle to the port and address for the ADS1115
 * @param status current status of the conversion
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t ads1115_get_conversion_status(const ads1115_handle_t *handle,
                                        ads1115_op_status_t    *status);

/**
 * @brief Set the input pins to ADS1115
 * @param handle handle to the port and address for the ADS1115
 * @param selected_pins pins mux option to select which pins are to be used as
 * input
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t ads1115_set_input_pins(const ads1115_handle_t *handle,
                                 ads1115_mux_t           selected_pins);

/**
 * @brief Reads which pins are set as input
 * @param handle handle to the port and address for the ADS1115
 * @param selected_pins pins that are set to input
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t ads1115_get_input_pins(const ads1115_handle_t *handle,
                                 ads1115_mux_t          *selected_pins);

/**
 * @brief Set the full scale gain of the programmable gain amplifier of the
 * ADS1115
 * @param handle handle to the port and address for the ADS1115
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t ads1115_set_full_scale_gain(const ads1115_handle_t *handle,
                                      ads1115_full_scale_t    full_scale_gain);

/**
 * @brief Read the full scale gain
 * @param handle handle to the port and address for the ADS1115
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t ads1115_get_full_scale_gain(const ads1115_handle_t *handle,
                                      ads1115_full_scale_t   *full_scale_gain);

/**
 * @brief Set the ADS1115 to either continuous operating mode or single shot
 * mode
 * @param handle handle to the port and address for the ADS1115
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t ads1115_set_mode(const ads1115_handle_t *handle,
                           ads1115_op_mode_t       mode);

/**
 * @brief Check whether the ADS1115 is in single shot mode or continuous mode
 * @param handle handle to the port and address for the ADS1115
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t ads1115_get_mode(const ads1115_handle_t *handle,
                           ads1115_op_mode_t      *mode);

/**
 * @brief Set the sampling rate of the ADS1115
 * @param handle handle to the port and address for the ADS1115
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t ads1115_set_data_rate(const ads1115_handle_t *handle,
                                ads1115_data_rate_t     data_rate);

/**
 * @brief Check the sampling rate of the ADS1115
 * @param handle handle to the port and address for the ADS1115
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t ads1115_get_data_rate(const ads1115_handle_t *handle,
                                ads1115_data_rate_t    *data_rate);

/**
 * @brief Set the comparator to either windowed or traditional
 * @param handle handle to the port and address for the ADS1115
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t ads1115_set_comparator_mode(const ads1115_handle_t *handle,
                                      ads1115_comp_mode_t     mode);

/**
 * @brief
 * @param handle handle to the port and address for the ADS1115
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t ads1115_get_comparator_mode(const ads1115_handle_t *handle,
                                      ads1115_comp_mode_t    *mode);

/**
 * @brief
 * @param handle handle to the port and address for the ADS1115
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t ads1115_set_comparator_polarity(const ads1115_handle_t *handle,
                                          ads1115_comp_polarity_t polarity);

/**
 * @brief
 * @param handle handle to the port and address for the ADS1115
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t ads1115_get_comparator_polarity(const ads1115_handle_t  *handle,
                                          ads1115_comp_polarity_t *polarity);

/**
 * @brief
 * @param handle handle to the port and address for the ADS1115
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t ads1115_set_comparator_latch(const ads1115_handle_t *handle,
                                       ads1115_comp_latch_t    latch);

/**
 * @brief
 * @param handle handle to the port and address for the ADS1115
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t ads1115_get_comparator_latch(const ads1115_handle_t *handle,
                                       ads1115_comp_latch_t   *latch);

/**
 * @brief
 * @param handle handle to the port and address for the ADS1115
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t ads1115_set_comparator_queue(const ads1115_handle_t *handle,
                                       ads1115_comp_queue_t    queue);

/**
 * @brief
 * @param handle handle to the port and address for the ADS1115
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t ads1115_get_comparator_queue(const ads1115_handle_t *handle,
                                       ads1115_comp_queue_t   *queue);

/**
 * @brief
 * @param handle handle to the port and address for the ADS1115
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t ads1115_read_conversion_result(const ads1115_handle_t *handle,
                                         int16_t                *result);

/**
 * @brief
 * @param handle handle to the port and address for the ADS1115
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t ads1115_estimate_absolute_reading(const ads1115_handle_t *handle,
                                            double                 *result);

/**
 * @brief
 * @param handle handle to the port and address for the ADS1115
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t ads1115_get_low_threshold(const ads1115_handle_t *handle,
                                    int16_t                *result);

/**
 * @brief
 * @param handle handle to the port and address for the ADS1115
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t ads1115_set_low_threshold(const ads1115_handle_t *handle,
                                    int16_t                 result);

/**
 * @brief
 * @param handle handle to the port and address for the ADS1115
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t ads1115_get_high_threshold(const ads1115_handle_t *handle,
                                     int16_t                *result);

/**
 * @brief
 * @param handle handle to the port and address for the ADS1115
 * @return
 *  - ESP_OK Success
 *  - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t ads1115_set_high_threshold(const ads1115_handle_t *handle,
                                     int16_t                 result);

#endif // ADS111X_H
