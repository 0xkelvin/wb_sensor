/**
 * Ruuvi Endpoint 3 helper. Defines necessary data for creating a Ruuvi data format 3 broadcast.
 *
 * License: BSD-3
 * Author: Otso Jousimaa <otso@ojousima.net>
 */

#ifndef RUUVI_ENDPOINT_3_H
#define RUUVI_ENDPOINT_3_H
#include "ruuvi_endpoints.h"
#include <stdint.h>

#define RUUVI_ENDPOINT_3_DESTINATION 0x03
#define RUUVI_ENDPOINT_3_INVALID_DATA 0
#define RUUVI_ENDPOINT_3_DATA_LENGTH 22

#define RUUVI_ENDPOINT_3_OFFSET_HEADER               0
#define RUUVI_ENDPOINT_3_OFFSET_HUMIDITY             1
#define RUUVI_ENDPOINT_3_OFFSET_TEMPERATURE_DECIMAL  2
#define RUUVI_ENDPOINT_3_OFFSET_TEMPERATURE_FRACTION 3
#define RUUVI_ENDPOINT_3_OFFSET_PRESSURE_MSB         4
#define RUUVI_ENDPOINT_3_OFFSET_PRESSURE_LSB         5
#define RUUVI_ENDPOINT_3_OFFSET_ACCELERATIONX_MSB    6
#define RUUVI_ENDPOINT_3_OFFSET_ACCELERATIONX_LSB    7
#define RUUVI_ENDPOINT_3_OFFSET_ACCELERATIONY_MSB    8
#define RUUVI_ENDPOINT_3_OFFSET_ACCELERATIONY_LSB    9
#define RUUVI_ENDPOINT_3_OFFSET_ACCELERATIONZ_MSB    10
#define RUUVI_ENDPOINT_3_OFFSET_ACCELERATIONZ_LSB    11
#define RUUVI_ENDPOINT_3_OFFSET_VOLTAGE_MSB          12
#define RUUVI_ENDPOINT_3_OFFSET_VOLTAGE_LSB          13
#define RUUVI_ENDPOINT_3_OFFSET_SUPERCAP_MSB         14
#define RUUVI_ENDPOINT_3_OFFSET_SUPERCAP_LSB         15
#define RUUVI_ENDPOINT_3_OFFSET_REC1_ADC             16
#define RUUVI_ENDPOINT_3_OFFSET_REC2_ADC             18
#define RUUVI_ENDPOINT_3_OFFSET_PM_ADC               20

typedef struct{
  float humidity_rh;
  float pressure_pa;
  float temperature_c;
  int16_t accelerationx_g;
  int16_t accelerationy_g;
  int16_t accelerationz_g;
  float battery_v;
  float rec1_adc;
  float rec2_adc;
  float pm_adc;
  int   light;
}ruuvi_endpoint_3_data_t;

ruuvi_endpoint_status_t ruuvi_endpoint_3_encode(uint8_t* const buffer, const ruuvi_endpoint_3_data_t* data, const float invalid);


#endif
