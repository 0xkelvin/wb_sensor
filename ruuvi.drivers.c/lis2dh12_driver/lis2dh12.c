#include "lis2dh12.h"
#include "nrf_log.h"

//////////////////////////////////////RUUVI FUNCTIONS /////////////////////////////////////////////////////

/* MACROS *****************************************************************************************/


/* PROTOTYPES *************************************************************************************/
static ret_code_t selftest(void);
void timer_lis2dh12_event_handler(void* p_context);
static int16_t rawToMg(int16_t raw_acceleration);
static uint8_t scale_interrupt_threshold(int16_t threshold_mg);

/* VARIABLES **************************************************************************************/
static lis2dh12_scale_t       state_scale      = LIS2DH12_SCALE4G;
static lis2dh12_resolution_t  state_resolution = LIS2DH12_RES12BIT;
static lis2dh12_sample_rate_t sample_rate      = LIS2DH12_RATE_10;

static volatile bool    m_xfer_done = false;
static const nrf_drv_twi_t *m_twi;
static uint8_t	m_addr; // I2C Address


/**
 *  Initializes LIS2DH12, and puts it in sleep mode.
 *  
 */
void lis2dh12_twi_init(nrf_drv_twi_t const *         p_twi,
                       lis2dh12_twi_config_t const * p_config ) // TODO complete the config
{
    ret_code_t err_code;
    
    m_twi = p_twi;
    m_addr = p_config->addr;
    state_scale = p_config->scale;
    state_resolution = p_config->resolution;
    sample_rate = p_config->sample_rate;
    /* Start Selftest */
    lis2dh12_enable();
    lis2dh12_reset();
    err_code = selftest();
    APP_ERROR_CHECK(err_code);

    lis2dh12_set_scale(state_scale);
    lis2dh12_set_resolution(state_resolution);
    lis2dh12_set_sample_rate(sample_rate);

    
}

//FLAG static void of just void
/** Reboots memory to default settings **/
static void lis2dh12_reset(void)
{
  uint8_t ctrl[3] = {0x10, 0x00, 0x2F}; //Default values, only first and third are non-zero
  lis2dh12_write_register(0x1E, &ctrl[0], 1);
  lis2dh12_write_register(0x1F, &ctrl[1], 1);
  lis2dh12_write_register(0x20, &ctrl[2], 1);
  lis2dh12_write_register(0x21, &ctrl[1], 1);
  lis2dh12_write_register(0x22, &ctrl[1], 1);
  lis2dh12_write_register(0x23, &ctrl[1], 1);
  lis2dh12_write_register(0x24, &ctrl[1], 1);
  lis2dh12_write_register(0x25, &ctrl[1], 1);
  lis2dh12_write_register(0x26, &ctrl[1], 1);
  lis2dh12_write_register(0x2E, &ctrl[1], 1);
  lis2dh12_write_register(0x30, &ctrl[1], 1);
  lis2dh12_write_register(0x32, &ctrl[1], 1);
  lis2dh12_write_register(0x33, &ctrl[1], 1);
  lis2dh12_write_register(0x34, &ctrl[1], 1);
  lis2dh12_write_register(0x36, &ctrl[1], 1);
  lis2dh12_write_register(0x37, &ctrl[1], 1);
  lis2dh12_write_register(0x38, &ctrl[1], 1);
  lis2dh12_write_register(0x3A, &ctrl[1], 1);
  lis2dh12_write_register(0x3B, &ctrl[1], 1);
  lis2dh12_write_register(0x3C, &ctrl[1], 1);
  lis2dh12_write_register(0x3D, &ctrl[1], 1);
  lis2dh12_write_register(0x3E, &ctrl[1], 1);
  lis2dh12_write_register(0x3F, &ctrl[1], 1);
}

/**
 *  Enables X-Y-Z axes
 */
static void lis2dh12_enable(void)
{
    ret_code_t err_code;
    /* Enable XYZ axes */
    uint8_t ctrl[1] = {0};
    ctrl[0] = LIS2DH12_XYZ_EN_MASK;
    lis2dh12_write_register(LIS2DH12_CTRL_REG1, ctrl, 1);
}

/**
 *  
 */
static void lis2dh12_set_scale(lis2dh12_scale_t scale)
{
    //Read current value of CTRL4 Register
    uint8_t ctrl4[1] = {0};
    lis2dh12_read_register(LIS2DH12_CTRL_REG4, ctrl4, 1);

    //Reset scale bits
    ctrl4[0] &= ~LIS2DH12_FS_MASK;
    ctrl4[0] |= scale;

    //Write register value back to lis2dh12
    lis2dh12_write_register(LIS2DH12_CTRL_REG4, ctrl4, 1);
    state_scale = scale; 
  
}

/**
 * Return full scale in mg for current state
 */
int lis2dh12_get_full_scale()
{
    switch(state_scale)
    {
        case LIS2DH12_SCALE2G:  return 2000;
        case LIS2DH12_SCALE4G:  return 4000;
        case LIS2DH12_SCALE8G:  return 8000;
        case LIS2DH12_SCALE16G: return 16000;
        default:                return 0;
    }
}

/**
 *  Sets resolution to lis2dh12in bits
 *  valid values are in enum lis2dh12_resolution_t
 *  Invalid resolution will put device to normal power (10 bit) mode.
 *
 *  returns error code, 0 on success.
 */
static void lis2dh12_set_resolution(lis2dh12_resolution_t resolution)
{
    ret_code_t err_code;

    uint8_t ctrl1[1] = {0};
    uint8_t ctrl4[1] = {0};

    //Read registers 3 & 4
    lis2dh12_read_register(LIS2DH12_CTRL_REG1, ctrl1, 1);
    lis2dh12_read_register(LIS2DH12_CTRL_REG4, ctrl4, 1);

    //Reset Low-power, high-resolution masks
    ctrl1[0] &= ~LIS2DH12_LPEN_MASK;
    ctrl4[0] &= ~LIS2DH12_HR_MASK;
    switch(resolution)
    {
        case LIS2DH12_RES12BIT:
              ctrl4[0] |= LIS2DH12_HR_MASK;
             break;

        //No action needed
        case LIS2DH12_RES10BIT:
             break;

        case LIS2DH12_RES8BIT:
             ctrl1[0] |= LIS2DH12_LPEN_MASK;
             break;
        //Writing normal power to lis2dh12 is safe
        default:
             err_code |= 1;
             APP_ERROR_CHECK(err_code);
             break;
    }
    lis2dh12_write_register(LIS2DH12_CTRL_REG1, ctrl1, 1);
    lis2dh12_write_register(LIS2DH12_CTRL_REG4, ctrl4, 1);
    state_resolution = resolution;   
}

/**
 *  
 * Note: By design, when the device from high-resolution configuration (HR) is set to power-down 
 * mode (PD), it is recommended to read register REFERENCE (26h)
 * for a complete reset of the filtering block before switching to normal/high-performance mode again for proper 
 * device functionality.
 */
static void lis2dh12_set_sample_rate(lis2dh12_sample_rate_t sample_rate)
{
    uint8_t ctrl[1] = {0};
    lis2dh12_read_register(LIS2DH12_CTRL_REG1, ctrl, 1);
    NRF_LOG_DEBUG("Read samplerate %x \n", ctrl[0]);

    // Clear sample rate bits
    ctrl[0] &= ~LIS2DH12_ODR_MASK;

    // Setup sample rate
    ctrl[0] |= sample_rate;
    lis2dh12_write_register(LIS2DH12_CTRL_REG1, ctrl, 1);
    NRF_LOG_DEBUG("Wrote samplerate %x \n", ctrl[0]);

    //Always read REFERENCE register when powering down to reset filter.
    if(LIS2DH12_RATE_0 == sample_rate)
    {
        lis2dh12_read_register(LIS2DH12_CTRL_REG6, ctrl, 1);
    }
}

/**
 * Get sample rate.
 */
static void lis2dh12_get_sample_rate(lis2dh12_sample_rate_t *sample_rate)
{
    uint8_t ctrl[1] = {0};
    lis2dh12_read_register(LIS2DH12_CTRL_REG1, ctrl, 1);
    NRF_LOG_DEBUG("Read samplerate %x \n", ctrl[0]);
    ctrl[0] &= LIS2DH12_ODR_MASK;
    *sample_rate = ctrl[0];
}

/**
 * Convert sample_rate values to frequency.
 */
int lis2dh12_odr_to_hz(lis2dh12_sample_rate_t sample_rate)
{
    sample_rate &= LIS2DH12_ODR_MASK;
    switch(sample_rate) 
    {
        case LIS2DH12_RATE_0: return 0;
        case LIS2DH12_RATE_1: return 1;
        case LIS2DH12_RATE_10: return 10;
        case LIS2DH12_RATE_25: return 25;
        case LIS2DH12_RATE_50: return 50;
        case LIS2DH12_RATE_100: return 100;
        case LIS2DH12_RATE_200: return 200;
        case LIS2DH12_RATE_400: return 400;
        default: return 0; /** 1k+ rates not implemented */
    }
}

/**
 *  Select FIFO mode
 *
 *  Bypass:         FiFo not in use. Setting Bypass resets FiFo
 *  FIFO:           FiFo is in use. Accumulates data until full or reset
 *  Stream:         FiFo is in use. Accumulates data, discarding oldest sample on overflow
 *  Stream-to-FIFO: FiFo is in use. Starts in stream, switches to FIFO on interrupt. Remember to configure the interrupt source.
 *
 */
static void lis2dh12_set_fifo_mode(lis2dh12_fifo_mode_t mode)
{
    uint8_t ctrl_fifo[1] = {0};
    uint8_t ctrl5[1] = {0};

      lis2dh12_read_register(LIS2DH12_CTRL_REG5, ctrl5, 1);
      lis2dh12_read_register(LIS2DH12_FIFO_CTRL_REG, ctrl_fifo, 1);

      // Clear FiFo bits
      ctrl_fifo[0] &= ~LIS2DH12_FM_MASK;
      //Clear enable bit
      ctrl5[0] &= ~LIS2DH12_FIFO_EN_MASK;
      // Setup FiFo rate
      ctrl_fifo[0] |= mode;
      //Enable FiFo if appropriate
      if(LIS2DH12_MODE_BYPASS != mode)
      { 
          ctrl5[0] |= LIS2DH12_FIFO_EN_MASK; 
      }
      //FIFO must be enabled before setting mode
      lis2dh12_write_register(LIS2DH12_CTRL_REG5, ctrl5, 1);
      lis2dh12_write_register(LIS2DH12_FIFO_CTRL_REG, ctrl_fifo, 1);
}

static void lis2dh12_read_samples(lis2dh12_sensor_buffer_t* buffer, size_t count)
{
     size_t bytes_to_read = count*sizeof(lis2dh12_sensor_buffer_t);

     lis2dh12_read_register(LIS2DH12_OUT_X_L, (uint8_t*)buffer, count*sizeof(lis2dh12_sensor_buffer_t));
     // Use constant bitshift, so we don't have to adjust mgpb with resolution
     for(int ii = 0; ii < count; ii++)
     {
        buffer[ii].sensor.x = rawToMg(buffer[ii].sensor.x);
        buffer[ii].sensor.y = rawToMg(buffer[ii].sensor.y);
        buffer[ii].sensor.z = rawToMg(buffer[ii].sensor.z);
     }
}

// put number of samples in HW FIFO to count
static void lis2dh12_get_fifo_sample_number(size_t* count)
{ 
    uint8_t ctrl[1] = {0};
    lis2dh12_read_register(LIS2DH12_FIFO_SRC_REG, ctrl, 1);
    *count = ctrl[0] & LIS2DH12_FSS_MASK;
}

// Generate watermark interrupt when FIFO reaches certain level
static ret_code_t lis2dh12_set_fifo_watermark(size_t count)
{
    if(count > 32) return NRF_ERROR_DRV_TWI_ERR_ANACK;

    uint8_t ctrl[1] = {0};
    lis2dh12_read_register(LIS2DH12_FIFO_CTRL_REG, ctrl, 1);
    ctrl[0] &= ~LIS2DH12_FTH_MASK;
    ctrl[0] += count;
    lis2dh12_write_register(LIS2DH12_FIFO_CTRL_REG, ctrl, 1);
}

/**
 * Enable activity detection interrupt on pin 2. Interrupt is high for samples where high-passed acceleration exceeds mg
 */
static void lis2dh12_set_activity_interrupt_pin_2(uint16_t mg)
{
    uint8_t cfg = 0;

    // // Configure activity interrupt - TODO: Implement in driver, add tests.
    // uint8_t ctrl[1];
    // // Enable high-pass for Interrupt function 2.
    // //CTRLREG2 = 0x02
    // ctrl[0] = LIS2DH12_HPIS2_MASK;
    // lis2dh12_write_register(LIS2DH12_CTRL_REG2, ctrl, 1);
    lis2dh12_read_register(LIS2DH12_CTRL_REG2, &cfg, 1);
    cfg |= LIS2DH12_HPIS2_MASK;
    lis2dh12_write_register(LIS2DH12_CTRL_REG2, &cfg, 1);

    // Enable interrupt 2 on X-Y-Z HI/LO.
    // INT2_CFG = 0x7F
    // ctrl[0] = 0x7F;
    // lis2dh12_write_register(LIS2DH12_INT2_CFG, ctrl, 1);
    cfg = 0;
    cfg |= LIS2DH12_6D_MASK | LIS2DH12_ZHIE_MASK | LIS2DH12_ZLIE_MASK;
    cfg |= LIS2DH12_YHIE_MASK | LIS2DH12_YLIE_MASK;
    cfg |= LIS2DH12_XHIE_MASK | LIS2DH12_XLIE_MASK;
    lis2dh12_set_interrupt_configuration(cfg, 2);
    // Interrupt on 64 mg+ (highpassed, +/-).
    //INT2_THS= 0x04 // 4 LSB = 64 mg @2G scale
    // ctrl[0] = LIS2DH12_ACTIVITY_THRESHOLD;
    // lis2dh12_write_register(LIS2DH12_INT2_THS, ctrl, 1);

    uint8_t threshold = scale_interrupt_threshold(mg);
    if(0 == threshold)
    {
      threshold = 1;
    }

    lis2dh12_set_threshold(threshold, 2);

    // Enable Interrupt function 2 on LIS interrupt pin 2 (stays high for 1/ODR).
    lis2dh12_set_interrupts(LIS2DH12_I2C_INT2_MASK, 2);
}

/**
 *  Set interrupt on pin. Write "0" To disable interrupt on pin. 
 *  NOTE: pin 1 and pin 2 DO NOT support identical configurations.
 *
 *  @param interrupts interrupts, see registers.h
 *  @param function 1 or 2, others are invalid
 */
static ret_code_t lis2dh12_set_interrupts(uint8_t interrupts, uint8_t function)
{
    if(1 != function && 2 != function)
    { 
        return NRF_ERROR_DRV_TWI_ERR_ANACK; 
    }
    uint8_t ctrl[1]; 
    ctrl[0] = interrupts;
    uint8_t target_reg = LIS2DH12_CTRL_REG3;
    if( 2 == function ) 
    { 
        target_reg = LIS2DH12_CTRL_REG6; 
    }
    lis2dh12_write_register(target_reg, ctrl, 1);
    return 0;
}

/**
 * Setup interrupt configuration: AND/OR of events, X-Y-Z Hi/Lo, 6-direction detection
 *
 * @param cfg, configuration. See registers.h for description
 * @param function number of interrupt, 1 or 2. Others are invalid
 *
 * @return error code from SPI write or LIS2DH12_RET_INVALID if pin was invalid. 0 on success.
 */
static ret_code_t lis2dh12_set_interrupt_configuration(uint8_t cfg, uint8_t function)
{
    if(1 != function && 2 != function)
    { 
        return NRF_ERROR_DRV_TWI_ERR_ANACK; 
    }
    uint8_t ctrl[1]; 
    ctrl[0] = cfg;
    uint8_t target_reg = LIS2DH12_INT1_CFG;
    if( 2 == function ) 
    { 
        target_reg = LIS2DH12_INT2_CFG; 
    }
    lis2dh12_write_register(target_reg, ctrl, 1);
    return 0;
}

/**
 *  Setup number of LSBs needed to trigger activity interrupt. 
 *
 *  @param bits number of LSBs required to trigger the interrupt
 *  @param pin 1 or 2, others are invalid
 *
 *  @return error code from stack
 */
static ret_code_t lis2dh12_set_threshold(uint8_t bits, uint8_t pin)
{
    if((1 != pin && 2 != pin) || bits > 0x7F)
    { 
        return NRF_ERROR_DRV_TWI_ERR_ANACK; 
    }

    uint8_t ctrl[1];
    ctrl[0] = bits;
    uint8_t target_reg = LIS2DH12_INT1_THS;
    if(2 == pin) 
    { 
        target_reg = LIS2DH12_INT2_THS;
    } 

    lis2dh12_write_register(target_reg, ctrl, 1);
    return 0;
}

/**
 * Enable tap detection interrupt with click_cfg, threshold, time limit,
 * window, latency and interrupt pin (1 or 2).
 *
 * STM App note (en.DM00365457.pdf) advices that (double) tap detection
 * does not work well with data rates lower than 400 Hz. LIS2DH12 consumes
 * 4 uA at 10 Hz in normal mode and 73 uA at 400 Hz. For some applications
 * you may get away with a lower sampling rate combined with lower
 * acceleration threshold, but both false positives and false negatives
 * increase.
 *
 * These parameters detect double taps well (no false positives on moving,
 * reliable detection) on Ruuvitag with LIS2DH12 data rate set to 400 Hz:
 * lis2dh12_set_tap_interrupt(LIS2DH12_ZD_MASK, 1000, 100, 100, 400, 1)
 */
static ret_code_t lis2dh12_set_tap_interrupt(uint8_t click_cfg, int threshold_mg, int timelimit_ms, int latency_ms, int window_ms, uint8_t pin)
{
    int reg;
    uint8_t value;
    lis2dh12_sample_rate_t sample_rate;
    ret_code_t err_code;

    if (pin == 1) {
        value = LIS2DH12_I1_CLICK;
    } else if (pin == 2) {
        value = LIS2DH12_I2C_CCK_EN_MASK;
    } else {
        return 1;
    }
    /* Enable click interrupt on pin */
    err_code = lis2dh12_set_interrupts(value, pin);

    /* Turn on high pass filter for click detection */
    lis2dh12_read_register(LIS2DH12_CTRL_REG2, &value, 1);
    value |= LIS2DH12_HPCLICK_MASK;
    value &= ~LIS2DH12_HPCF_MASK; // Largest highpass cutoff frequency
    lis2dh12_write_register(LIS2DH12_CTRL_REG2, &value, 1);

    value = click_cfg;
    lis2dh12_write_register(LIS2DH12_CLICK_CFG, &value, 1);

    /* Set threshold */
    reg = (128*threshold_mg) / lis2dh12_get_full_scale();
    value = (uint8_t) (reg > LIS2DH12_CLK_THS_MASK) ? LIS2DH12_CLK_THS_MASK : reg; // clip to max value (7 bits)
    value = (value < 1) ? 1 : value; // at least 1
    // LIR_CLICK bit will be (implicitly) set to 0
    lis2dh12_write_register(LIS2DH12_CLICK_THS, &value, 1);

    /* Set time limit */
    lis2dh12_get_sample_rate(&sample_rate);
    reg = (timelimit_ms * lis2dh12_odr_to_hz(sample_rate)) / 1e3; // time limit in measurement cycles
    value = (uint8_t) (reg > LIS2DH12_TLI_MASK) ? LIS2DH12_TLI_MASK : reg; // clip to max value (7 bits)
    value = (value < 1) ? 1 : value; // at least 1 cycle
    lis2dh12_write_register(LIS2DH12_TIME_LIMIT, &value, 1);

    /* Set latency */
    reg = (latency_ms * lis2dh12_odr_to_hz(sample_rate)) / 1e3;
    value = (uint8_t) (reg > 0xFF) ? 0xFF : reg ; // clip to max value (8 bits)
    lis2dh12_write_register(LIS2DH12_TIME_LATENCY, &value, 1);

    /* Set window */
    reg = (window_ms * lis2dh12_odr_to_hz(sample_rate)) / 1e3;
    value = (uint8_t) (reg > 0xFF) ? 0xFF : reg ; // clip to max value (8 bits)
    lis2dh12_write_register(LIS2DH12_TIME_WINDOW, &value, 1);

    return err_code;
}

/* INTERNAL FUNCTIONS *****************************************************************************/

/**
 * Execute LIS2DH12 Selftest
 * TODO: Run the self-test internal to device
 *
 * @return LIS2DH12_RET_OK Selftest passed
 * @return LIS2DH12_RET_ERROR_SELFTEST Selftest failed
 */
static ret_code_t selftest(void)
{
    uint8_t value[1] = {0};
    //lis2dh12_read_register((uint8_t)LIS2DH12_WHO_AM_I, value, 1);
    lis2dh12_read_register(0x0F, value, 1);
    return (LIS2DH12_I_AM_MASK == value[0]) ? 0 : 1;
}

/**
 * Conversion functions from 
 * https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/3e3b7528dfacb223aea250daf4e512e335f17509/lis2dh12_STdC/driver/lis2dh12_reg.c#L104
 */
static inline int16_t lis2dh12_from_fs2_hr_to_mg(int16_t lsb)
{
  return (lsb / 16 ) * 1;
}

static inline int16_t lis2dh12_from_fs4_hr_to_mg(int16_t lsb)
{
  return (lsb / 16 ) * 2;
}

static inline int16_t lis2dh12_from_fs8_hr_to_mg(int16_t lsb)
{
  return (lsb / 16 ) * 4;
}

static inline int16_t lis2dh12_from_fs16_hr_to_mg(int16_t lsb)
{
  return (lsb / 16) * 12;
}

static inline int16_t lis2dh12_from_fs2_nm_to_mg(int16_t lsb)
{
  return (lsb / 64) * 4;
}

static inline int16_t lis2dh12_from_fs4_nm_to_mg(int16_t lsb)
{
  return (lsb / 64) * 8;
}

static inline int16_t lis2dh12_from_fs8_nm_to_mg(int16_t lsb)
{
  return (lsb / 64) * 16;
}

static inline int16_t lis2dh12_from_fs16_nm_to_mg(int16_t lsb)
{
  return (lsb / 64) * 48;
}

static inline int16_t lis2dh12_from_fs2_lp_to_mg(int16_t lsb)
{
  return (lsb / 256) * 16;
}

static inline int16_t lis2dh12_from_fs4_lp_to_mg(int16_t lsb)
{
  return (lsb / 256) * 32;
}

static inline int16_t lis2dh12_from_fs8_lp_to_mg(int16_t lsb)
{
  return (lsb / 256) * 64;
}

static inline int16_t lis2dh12_from_fs16_lp_to_mg(int16_t lsb)
{
  return (lsb / 256) * 192;
}


/**
 * Convert raw value to acceleration in mg. Reads scale and resolution from state variables.
 *
 * @param raw_acceleration raw ADC value from LIS2DH12
 * @return int16_t representing acceleration in milli-G. 
 */
static int16_t rawToMg(int16_t raw_acceleration)
{
  switch(state_scale)
  {
    case LIS2DH12_SCALE2G:
      switch(state_resolution)
      {
        case LIS2DH12_RES8BIT:
          return lis2dh12_from_fs2_lp_to_mg(raw_acceleration);

        case LIS2DH12_RES10BIT:
          return  lis2dh12_from_fs2_nm_to_mg(raw_acceleration);

        case LIS2DH12_RES12BIT:
          return  lis2dh12_from_fs2_hr_to_mg(raw_acceleration);

        default:
          break;
      }
      break;

    case LIS2DH12_SCALE4G:
      switch(state_resolution)
      {
        case LIS2DH12_RES8BIT:
          return  lis2dh12_from_fs4_lp_to_mg(raw_acceleration);

        case LIS2DH12_RES10BIT:
          return  lis2dh12_from_fs4_nm_to_mg(raw_acceleration);

        case LIS2DH12_RES12BIT:
          return  lis2dh12_from_fs4_hr_to_mg(raw_acceleration);
      }
      break;

    case LIS2DH12_SCALE8G:
      switch(state_resolution)
      {
        case LIS2DH12_RES8BIT:
          return  lis2dh12_from_fs8_lp_to_mg(raw_acceleration);

        case LIS2DH12_RES10BIT:
          return  lis2dh12_from_fs8_nm_to_mg(raw_acceleration);

        case LIS2DH12_RES12BIT:
          return  lis2dh12_from_fs8_hr_to_mg(raw_acceleration);

        default:
          break;
      }
    break;

    case LIS2DH12_SCALE16G:
      switch(state_resolution)
      {
        case LIS2DH12_RES8BIT:
          return  lis2dh12_from_fs16_lp_to_mg(raw_acceleration);

        case LIS2DH12_RES10BIT:
          return  lis2dh12_from_fs16_nm_to_mg(raw_acceleration);

        case LIS2DH12_RES12BIT:
          return  lis2dh12_from_fs16_hr_to_mg(raw_acceleration);

        default:
          break;
      }
    break;

    default:
      break;
  }
  // reached only in case of an error, return "smallest representable value"
  return 0x8000;
}

/**
 * Return correct threshold setting for activity interrupt at
 * given threshold. Scales threshold upwards to next value.
 *
 * @param threshold_mg desired threshold. Will be converted to positive value if negative value is given.
 * @return threshold to set or 0x7F (max) if given threshold is not possible.
 */
static uint8_t scale_interrupt_threshold(int16_t threshold_mg)
{
  // Adjust for scale
  // 1 LSb = 16 mg @ FS = 2 g
  // 1 LSb = 32 mg @ FS = 4 g
  // 1 LSb = 62 mg @ FS = 8 g
  // 1 LSb = 186 mg @ FS = 16 g
  uint8_t divisor;
  switch(state_scale)
  {
    case LIS2DH12_FS_2G:
      divisor = 16;
      break;

    case LIS2DH12_FS_4G:
      divisor = 32;
      break;

    case LIS2DH12_FS_8G:
      divisor = 62;
      break;

    case LIS2DH12_FS_16G:
      divisor = 186;
      break;

    default:
      divisor = 16;
      break;
  }

  if(threshold_mg < 0) { threshold_mg = 0 - threshold_mg; }
  uint8_t threshold = (threshold_mg/divisor) + 1;
  if(threshold > 0x7F) { threshold = 0x7F; }
  return threshold;
}


////////////////////////////////////// TRANSFERFI FUNCTION ////////////////////////////////////////////////



static void lis2dh12_read_register(const int8_t address, uint8_t* const p_toRead, const size_t count)
{

    ret_code_t err_code; 
//    uint8_t* read_buff   = calloc(count+1, sizeof(uint8_t));
//    uint8_t  write_buff[1];
    uint8_t* read_buff   = calloc(count+1, sizeof(uint8_t));
    uint8_t* write_buff   = calloc(count+1, sizeof(uint8_t));

    write_buff[0] = address | TWI_ADR_INC;    // this allow for multiple read/write

    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(m_twi, m_addr, write_buff, 1, true);
    APP_ERROR_CHECK(err_code);
    do {
		__WFE();
	}while (m_xfer_done == false);


    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(m_twi, m_addr, read_buff, count);
    APP_ERROR_CHECK(err_code);
    do {
		__WFE();
	}while (m_xfer_done == false);


    memcpy(p_toRead, &(read_buff[0]), count);

    free(read_buff);
    free(write_buff);
}


static void lis2dh12_write_register(int8_t address, int8_t* const value, size_t count)
{
    m_xfer_done = false;

    ret_code_t err_code;
    uint8_t* buff = calloc(count+1, sizeof(uint8_t));

    buff[0] = address | TWI_ADR_INC;
    memcpy(&(buff[1]), value, 1);
        
    err_code = nrf_drv_twi_tx(m_twi, m_addr, buff, 2, false);
    do {
		__WFE();
	}while (m_xfer_done == false);

    APP_ERROR_CHECK(err_code);

    //free(buff);
}

void lis2dh12_twi_measurement_get(lis2dh12_sensor_buffer_t *buffer){
        int32_t acc[3] = {0};
        lis2dh12_read_samples(buffer, 1);
        acc[0] = buffer->sensor.x;
        acc[1] = buffer->sensor.y;
        acc[2] = buffer->sensor.z;
}


void lis2dh12_twi_evt_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
        switch (p_event->type) {
		case NRF_DRV_TWI_EVT_DONE:
			m_xfer_done = true;
			break;
		default:
			break;
	}
}
