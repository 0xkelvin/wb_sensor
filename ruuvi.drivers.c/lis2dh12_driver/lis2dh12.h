/**
@addtogroup LIS2DH12Driver LIS2DH12 Acceleration Sensor Driver
@{
@file       LIS2DH12.h
Hardware Driver for the LIS2DH12 Acceleration Sensor
* @}
***************************************************************************************************/
#ifndef LIS2DH12_H
#define LIS2DH12_H

/* INCLUDES ***************************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include "app_error.h"
#include "app_scheduler.h"
#include "nordic_common.h"
#include "nrf_drv_twi.h"
//#include "app_timer_appsh.h"
#include "lis2dh12_registers.h"

/* CONSTANTS **************************************************************************************/
#define LIS2DH12_FIFO_MAX_LENGTH 32
#define LIS2DH12_NO_INTERRUPTS 0
#define LIS2DH12_TWI_ADDR1 0x19
#define LIS2DH12_TWI_ADDR0 0x18

//TODO remove non-relevant macro definitions if need to 


/* CONSTANTS **************************************************************************************/
/** Maximum Size of SPI Addresses */
#define ADR_MAX 0x3FU

/** Size of raw sensor data for all 3 axis */
#define SENSOR_DATA_SIZE 6U
//
///** Bit Mask to enable auto address incrementation for multi read */
#define TWI_ADR_INC 0x80U

/* TYPES ******************************************************************************************/
/** Structure containing sensor data for all 3 axis */
typedef struct __attribute__((packed))
{
    int16_t x;
    int16_t y;
    int16_t z;
}acceleration_t;

/** Union to split raw data to values for each axis */
typedef union
{
  uint8_t raw[sizeof(acceleration_t)];
  acceleration_t sensor;
}lis2dh12_sensor_buffer_t;

/* MACROS *****************************************************************************************/

/* TYPES ******************************************************************************************/
/** Available Scales */
typedef enum{
        LIS2DH12_SCALE2G = LIS2DH12_FS_2G,  /**< Scale Selection: +/- 2g */
        LIS2DH12_SCALE4G = LIS2DH12_FS_4G,	/**< Scale Selection: +/- 4g */
        LIS2DH12_SCALE8G = LIS2DH12_FS_8G,	/**< Scale Selection: +/- 8g */
        LIS2DH12_SCALE16G = LIS2DH12_FS_16G	/**< Scale Selection: +/- 16g */
}lis2dh12_scale_t;

/** Available Resolutions */
typedef enum{
        LIS2DH12_RES8BIT = 8,		/**< 8 extra bits */
        LIS2DH12_RES10BIT = 6,		/**< 6 extra bits */
        LIS2DH12_RES12BIT = 4		/**< 4 extra bits */
}lis2dh12_resolution_t;

/** Available sample rates */
typedef enum{
        LIS2DH12_RATE_0   = 0,		/**< Power down */
        LIS2DH12_RATE_1   = 1<<4,	/**< 1 Hz */
        LIS2DH12_RATE_10  = 2<<4,	/**< 10 Hz*/
        LIS2DH12_RATE_25  = 3<<4,		
        LIS2DH12_RATE_50  = 4<<4,		
        LIS2DH12_RATE_100 = 5<<4,		
        LIS2DH12_RATE_200 = 6<<4,		
        LIS2DH12_RATE_400 = 7<<4    /** 1k+ rates not implemented */		
}lis2dh12_sample_rate_t;

/** Available modes */
typedef enum{
        LIS2DH12_MODE_BYPASS = 0,                                              /**< FIFO off */
        LIS2DH12_MODE_FIFO   = LIS2DH12_FM_FIFO,                               /**< FIFO on */
        LIS2DH12_MODE_STREAM = LIS2DH12_FM_STREAM,                             /**< Stream on */
        LIS2DH12_MODE_STREAM_TO_FIFO = (LIS2DH12_FM_FIFO | LIS2DH12_FM_STREAM) /**< Stream until trigger */
}lis2dh12_fifo_mode_t;

typedef struct {
	uint8_t	addr;
        lis2dh12_scale_t scale;
        lis2dh12_resolution_t resolution;
        lis2dh12_sample_rate_t sample_rate;
        lis2dh12_fifo_mode_t fifo_mode;
} lis2dh12_twi_config_t;

/** Data Ready Event Callback Type */
typedef void (*LIS2DH12_drdy_event_t)(void);

/* PROTOTYPES *************************************************************************************/

/**
 *  Initializes LIS2DH12, enables X-Y-Z axes, goes to sleep.
 *  Checks communication by reading WHO_AM_I register, returns error if WHO_AM_I does not match
 */
void lis2dh12_twi_init(nrf_drv_twi_t const *         p_twi,
                       lis2dh12_twi_config_t const * p_config );

/**
 *  Reboots memory to power-up defaults
 *  Wait 10ms after calling reset before using sensor.
 *  Returns status from SPI write
 */
static void lis2dh12_reset(void);

/**
 *  Enables X-Y-Z axes after reboot. Does not start sampling.
 *  Returns status from SPI write
 */
static void lis2dh12_enable(void);

/**
 *  Select scale. Higher scale results in lower resolution
 *  Returns error code from SPI write
 */
static void lis2dh12_set_scale(lis2dh12_scale_t scale);

/**
 * Return full scale in mg for current state
 */
int lis2dh12_get_full_scale();

/**
 *  Select resolution. FIFO is 10 bits, 12 bit mode has lower bandwidth than 10 and 8 bit modes.
 *  Higher resolution consumes more power.
 *  Returns error code from SPI write
 */
static void lis2dh12_set_resolution(lis2dh12_resolution_t resolution);

/**
 *  Select sample rate. 
 *  Returns error code from SPI write
 */
static void lis2dh12_set_sample_rate(lis2dh12_sample_rate_t sample_rate);

/**
 * Get sample rate.
 */
static void lis2dh12_get_sample_rate(lis2dh12_sample_rate_t *sample_rate);

/**
 * Convert sample_rate values to frequency.
 */
int lis2dh12_odr_to_hz(lis2dh12_sample_rate_t sample_rate);

/**
 *  Select FIFO mode. Bypass: FIFO not used. FIFO: FIFO fills and "freezes" until FIFO reset by setting mode to bypass.
 *  Stream: Oldest element is discarded if FIFO is full. Stream-to-FIFO: Oldest element is discarded until defined event occurs,
 *  after which buffer switches to FIFO mode discarding new samples. 
 *  Returns error code from SPI write
 */
static void lis2dh12_set_fifo_mode(lis2dh12_fifo_mode_t mode);

/**
 *  Read specified number of samples into given buffer. values are read as
 *  buffer[index].x|y|z. Values are in mg, data type is int16_t. 
 *  Usage with FIFO: size_t count = 0; 
 *  lis2dh12_get_fifo_sample_number(&count);
 *  lis2dh12_read_samples(buffer, count);
 */
static void lis2dh12_read_samples(lis2dh12_sensor_buffer_t* buffer, size_t count);

/**
 *  Get number of samples waiting in buffer into count.
 *  Returns error code from SPI write
 */
static void lis2dh12_get_fifo_sample_number(size_t* count);

/**
 *  Sets FIFO watermark level, up to 32. After FIFO has number of samples
 *  defined by watermark interrupt occurs. Remember to set interrupts using lis2dh12_set_inhterrupts
 *  Returns error code from SPI write
 */
static ret_code_t lis2dh12_set_fifo_watermark(size_t count);

/**
 *  Set interrupt on pin. Write "0" To disable interrupt on pin. 
 *
 *  @param interrupts interrupts, see registers.h
 *  @param pin 1 or, others are invalid
 *  @return LIS2DH12_RET_INVALID if pin was not valid, error code from SPI stack otherwise.
 */
static ret_code_t lis2dh12_set_interrupts(uint8_t interrupts, uint8_t pin);

/**
 * Setup interrupt configuration: AND/OR of events, X-Y-Z Hi/Lo, 6-direction detection
 *
 * @param cfg, configuration. See registers.h for description
 * @param function number of interrupt, 1 or 2. Others are invalid
 *
 * @return error code from SPI write or LIS2DH12_RET_INVALID if pin was invalid. 0 on success.
 */
static ret_code_t lis2dh12_set_interrupt_configuration(uint8_t cfg, uint8_t function);

/**
 *  Setup number of LSBs needed to trigger AOI interrupt function. 
 *  Note: this targets function 1 or 2, not pin. 
 *
 *  @param bits number of LSBs required to trigger the interrupt
 *  @param function 1 or 2, others are invalid
 *
 *  @return error code from stack
 */
static ret_code_t lis2dh12_set_threshold(uint8_t bits, uint8_t pin);


/**
 * Enable activity detection interrupt on pin 2. Interrupt is high for samples where high-passed acceleration exceeds mg
 */
static void  lis2dh12_set_activity_interrupt_pin_2(uint16_t mg);

/**
 * Enable tap detection interrupt on given pin with click_cfg, threshold and time limit, window and latency.
 */
static ret_code_t  lis2dh12_set_tap_interrupt(uint8_t click_cfg, int threshold_mg, int timelimit_ms, int latency_ms, int window_ms, uint8_t pin);

/**
 *  Internal functions for reading/writing registers. 
 */
static void lis2dh12_read_register(int8_t address, uint8_t* const p_toRead, size_t count);
static void lis2dh12_write_register(int8_t address, int8_t* const value, size_t count);


 
/**
 *  Functions for getting measurements from the lis2dh12.
 */
void lis2dh12_twi_measurement_get(lis2dh12_sensor_buffer_t *buffer);
#endif  /* LIS2DH12_H */