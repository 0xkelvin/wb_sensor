#include "ruuvi_endpoint_3.h"
#include "ruuvi_endpoints.h"
#include <stddef.h>
#include <stdbool.h>
#include <math.h>
#include "nrf_log.h"
static void ruuvi_endpoint_3_encode_acceleration(uint8_t* const buffer, const float acceleration, const float invalid)
{
   int16_t decimal = (int16_t) (acceleration*1000);
   buffer[0] = decimal >> 8;
   buffer[1] = decimal & 0xFF;
}

ruuvi_endpoint_status_t ruuvi_endpoint_3_encode(uint8_t* const buffer, const ruuvi_endpoint_3_data_t* data, const float invalid)
{
  uint16_t temp_adc = 0;
  if(NULL == buffer  || NULL == data) { return RUUVI_ENDPOINT_ERROR_NULL; }

  buffer[RUUVI_ENDPOINT_3_OFFSET_HEADER] = RUUVI_ENDPOINT_3_DESTINATION;

  // HUMIDITY
  if(invalid != data->humidity_rh)
  {
    //Humidity (one lsb is 0.5%, e.g. 128 is 64%). Round the value
    buffer[RUUVI_ENDPOINT_3_OFFSET_HUMIDITY] = (uint8_t)((data->humidity_rh*2) + 0.5);
  }
  else
  {
    buffer[RUUVI_ENDPOINT_3_OFFSET_HUMIDITY] = RUUVI_ENDPOINT_3_INVALID_DATA;
  }

  // Temperature
  if(invalid != data->temperature_c)
  {
    //Temperature (MSB is sign, next 7 bits are decimal value)
    float temperature = data->temperature_c;
    bool sign = (temperature < 0) ? 1 : 0;
    // abs value
    if(0 > temperature) { temperature = 0 - temperature; }
    // cap the temperature
    if(127 < temperature) {temperature = 127; }

    buffer[RUUVI_ENDPOINT_3_OFFSET_TEMPERATURE_DECIMAL] = (uint8_t)temperature | (sign<<7);
    uint8_t temperature_fraction = (uint8_t)((temperature - floor(temperature)) * 100);
    buffer[RUUVI_ENDPOINT_3_OFFSET_TEMPERATURE_FRACTION] = temperature_fraction;
  }
  else
  {
    buffer[RUUVI_ENDPOINT_3_OFFSET_TEMPERATURE_DECIMAL]  = RUUVI_ENDPOINT_3_INVALID_DATA;
    buffer[RUUVI_ENDPOINT_3_OFFSET_TEMPERATURE_FRACTION] = RUUVI_ENDPOINT_3_INVALID_DATA;
  }

  // Pressure
  if(invalid != data->pressure_pa)
  {
    uint32_t pressure = data->pressure_pa;
    pressure -= 50000;
    buffer[RUUVI_ENDPOINT_3_OFFSET_PRESSURE_MSB] = pressure>>8;
    buffer[RUUVI_ENDPOINT_3_OFFSET_PRESSURE_LSB] = pressure&0xFF;
  }
  else
  {
    buffer[RUUVI_ENDPOINT_3_OFFSET_PRESSURE_MSB] = RUUVI_ENDPOINT_3_INVALID_DATA;
    buffer[RUUVI_ENDPOINT_3_OFFSET_PRESSURE_LSB] = RUUVI_ENDPOINT_3_INVALID_DATA;
  }

  // acceleration
  //ruuvi_endpoint_3_encode_acceleration(&buffer[RUUVI_ENDPOINT_3_OFFSET_ACCELERATIONX_MSB], data->rec1_adc, invalid);
  buffer[RUUVI_ENDPOINT_3_OFFSET_ACCELERATIONX_MSB] = data->accelerationx_g>>8;
  buffer[RUUVI_ENDPOINT_3_OFFSET_ACCELERATIONX_LSB] = data->accelerationx_g&0xFF;
  //ruuvi_endpoint_3_encode_acceleration(&buffer[RUUVI_ENDPOINT_3_OFFSET_ACCELERATIONY_MSB], data->rec2_adc, invalid);
  buffer[RUUVI_ENDPOINT_3_OFFSET_ACCELERATIONY_MSB] = data->accelerationy_g>>8;
  buffer[RUUVI_ENDPOINT_3_OFFSET_ACCELERATIONY_LSB] = data->accelerationy_g&0xFF;
  //ruuvi_endpoint_3_encode_acceleration(&buffer[RUUVI_ENDPOINT_3_OFFSET_ACCELERATIONZ_MSB], data->pm_adc, invalid);
  buffer[RUUVI_ENDPOINT_3_OFFSET_ACCELERATIONZ_MSB] = data->accelerationz_g>>8;
  buffer[RUUVI_ENDPOINT_3_OFFSET_ACCELERATIONZ_LSB] = data->accelerationz_g&0xFF;
  // voltage
  buffer[RUUVI_ENDPOINT_3_OFFSET_VOLTAGE_MSB] = data->light>>8;
  buffer[RUUVI_ENDPOINT_3_OFFSET_VOLTAGE_LSB] = data->light&0xFF;


  uint32_t voltage = (data->battery_v*1000);
  buffer[RUUVI_ENDPOINT_3_OFFSET_SUPERCAP_MSB] = voltage>>8;
  buffer[RUUVI_ENDPOINT_3_OFFSET_SUPERCAP_LSB] = voltage&0xFF;

  ruuvi_endpoint_3_encode_acceleration(&buffer[RUUVI_ENDPOINT_3_OFFSET_REC1_ADC], data->rec1_adc, invalid);
  ruuvi_endpoint_3_encode_acceleration(&buffer[RUUVI_ENDPOINT_3_OFFSET_REC2_ADC], data->rec2_adc, invalid);
  ruuvi_endpoint_3_encode_acceleration(&buffer[RUUVI_ENDPOINT_3_OFFSET_PM_ADC], data->pm_adc, invalid);
 
 
  // End

  return RUUVI_ENDPOINT_SUCCESS;
}
