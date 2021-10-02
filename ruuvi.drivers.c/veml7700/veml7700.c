
#include <stdio.h>
#include "veml7700.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "boards.h"
#include "app_util_platform.h"
#include "lis2dh12.h"
#include "lis2dh12_registers.h"

#define LIS2DH12_TWI_ADDR                     LIS2DH12_TWI_ADDR1
#define LIS2DH12_SCALE                        LIS2DH12_SCALE4G
#define LIS2DH12_RESOLUTION                   LIS2DH12_RES12BIT
#define LIS2DH12_SAMPLE_RATE                  LIS2DH12_RATE_10

//#define NRF_LOG_MODULE_NAME "VEML7700"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_delay.h"
/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    //NRF_LOG_INFO("twi_veml7700_handler p_event->type %d\r\n", p_event->type);
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            //NRF_LOG_INFO("NRF_DRV_TWI_EVT_DONE\r\n");
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                //data_handler(m_sample);
            }
            m_xfer_done = true;
            if(p_event->xfer_desc.address == LIS2DH12_TWI_ADDR)
            {
               lis2dh12_twi_evt_handler(p_event, p_context);
            }
            break;
        default:
            break;
    }
}

/**
 * @brief TWI initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = SCL_PIN,
       .sda                = SDA_PIN,
       .frequency          = TWI_FREQUENCY_FREQUENCY_K400,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

int veml_config(uint8_t reg, uint8_t reg_lsb, uint8_t reg_msb)
{
    NRF_LOG_INFO("veml_config\r\n");
    ret_code_t err_code;
    uint8_t reg_data[3] = {0x00, 0x00, 0x00};
 
    err_code = nrf_drv_twi_tx(&m_twi, VEML7700_ADDR, reg_data, sizeof(reg_data), false);
    APP_ERROR_CHECK(err_code);

    while (m_xfer_done == false);
    NRF_LOG_INFO("veml_config done\r\n");
    return 0;
}

/**
 * @brief LIS2DH12 initialization.
 */
void lis2dh12_init(void) {
    const lis2dh12_twi_config_t lis2dh12_twi_config = {
            .addr = LIS2DH12_TWI_ADDR,
            .scale = LIS2DH12_SCALE,
            .resolution = LIS2DH12_RESOLUTION,
            .sample_rate = LIS2DH12_SAMPLE_RATE,
    };

    lis2dh12_twi_init(&m_twi, &lis2dh12_twi_config);
}

uint16_t veml_read_luminosity(uint8_t reg)
{
    // send slave addr/Wr + command code + no_stop
    ret_code_t err_code;
    uint8_t reg_data = reg;
    uint8_t luminosity[2];
    uint16_t result = 0;
    
    err_code = nrf_drv_twi_tx(&m_twi, VEML7700_ADDR, &reg_data, 1, true);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
    // Rx 2 bytes 
    m_xfer_done = false;
    // Read 2 byte from the specified address
    nrf_delay_ms(5);
    err_code = nrf_drv_twi_rx(&m_twi, VEML7700_ADDR, luminosity, sizeof(luminosity));
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
    m_xfer_done = false;
    nrf_delay_ms(5); // don't know why need delay here to get correct value
    result = luminosity[1];
    result =  (result << 8) + luminosity[0];
    //NRF_LOG_INFO("veml_read_luminosity result %d \r\n", result);
    return result;
}



lis2dh12_sensor_buffer_t lis2dh12_data;
/**
 * @brief Function for reading acceleration data from LIS2DH12.
 */
void log_lis2dh12_sensors(void) //to get the measurement data from lis2dh12 (x,y,z).
{
    int lis2dh12_number_of_samples_avg = 2;
    lis2dh12_sensor_buffer_t raw_data[lis2dh12_number_of_samples_avg]; // array of raw data for averaging, set size to 1 if averaging is not
    
    //next few lines are for data averaging on acceleration values.
    //five samples are taken with a delay of 5 micro seconds.
    for(int i = 0; i < lis2dh12_number_of_samples_avg; i++)
    {
        lis2dh12_twi_measurement_get(&raw_data[i]);
        nrf_delay_us(5);
    }
    
    lis2dh12_data.sensor.x = 0;    
    lis2dh12_data.sensor.y = 0;    
    lis2dh12_data.sensor.z = 0;
    
    for(int i = 0; i < lis2dh12_number_of_samples_avg; i++)
    {
      lis2dh12_data.sensor.x = lis2dh12_data.sensor.x + raw_data[i].sensor.x;
      lis2dh12_data.sensor.y = lis2dh12_data.sensor.y + raw_data[i].sensor.y;
      lis2dh12_data.sensor.z = lis2dh12_data.sensor.z + raw_data[i].sensor.z;
    }
    
    lis2dh12_data.sensor.x = lis2dh12_data.sensor.x/lis2dh12_number_of_samples_avg;    
    lis2dh12_data.sensor.y = lis2dh12_data.sensor.y/lis2dh12_number_of_samples_avg;    
    lis2dh12_data.sensor.z = lis2dh12_data.sensor.z/lis2dh12_number_of_samples_avg;
}