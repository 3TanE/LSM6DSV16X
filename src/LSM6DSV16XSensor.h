/**
 ******************************************************************************
 * @file    LSM6DSV16XSensor.h
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    July 2022
 * @brief   Abstract Class of a LSM6DSV16X inertial measurement sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2022 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *notice, this list of conditions and the following disclaimer in the
 *documentation and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Prevent recursive inclusion -----------------------------------------------*/

#ifndef __LSM6DSV16XSensor_H__
#define __LSM6DSV16XSensor_H__

/* Includes ------------------------------------------------------------------*/

#ifdef PICO

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#define HIGH 1
#define LOW 0 

static void delay(int a)
{
    sleep_ms(a);
}

#else
#include "SPI.h"
#include "Wire.h"
#endif
#include "lsm6dsv16x_reg.h"
#include <cstring>

/* Defines -------------------------------------------------------------------*/
/* For compatibility with ESP32 platforms */
#ifdef ESP32
#ifndef MSBFIRST
#define MSBFIRST SPI_MSBFIRST
#endif
#endif

#define LSM6DSV16X_ACC_SENSITIVITY_FS_2G 0.061f
#define LSM6DSV16X_ACC_SENSITIVITY_FS_4G 0.122f
#define LSM6DSV16X_ACC_SENSITIVITY_FS_8G 0.244f
#define LSM6DSV16X_ACC_SENSITIVITY_FS_16G 0.488f

#define LSM6DSV16X_GYRO_SENSITIVITY_FS_125DPS 4.375f
#define LSM6DSV16X_GYRO_SENSITIVITY_FS_250DPS 8.750f
#define LSM6DSV16X_GYRO_SENSITIVITY_FS_500DPS 17.500f
#define LSM6DSV16X_GYRO_SENSITIVITY_FS_1000DPS 35.000f
#define LSM6DSV16X_GYRO_SENSITIVITY_FS_2000DPS 70.000f
#define LSM6DSV16X_GYRO_SENSITIVITY_FS_4000DPS 140.000f

#define LSM6DSV16X_QVAR_GAIN 78.000f

#define LSM6DSV16X_MIN_ST_LIMIT_mg 50.0f
#define LSM6DSV16X_MAX_ST_LIMIT_mg 1700.0f
#define LSM6DSV16X_MIN_ST_LIMIT_mdps 150000.0f
#define LSM6DSV16X_MAX_ST_LIMIT_mdps 700000.0f

#define LSM6DSV16X_ACC_USR_OFF_W_HIGH_LSB (float)(pow(2, -6))
#define LSM6DSV16X_ACC_USR_OFF_W_LOW_LSB (float)(pow(2, -10))
#define LSM6DSV16X_ACC_USR_OFF_W_HIGH_MAX                                      \
  LSM6DSV16X_ACC_USR_OFF_W_HIGH_LSB *INT8_MAX
#define LSM6DSV16X_ACC_USR_OFF_W_LOW_MAX                                       \
  LSM6DSV16X_ACC_USR_OFF_W_LOW_LSB *INT8_MAX

/* Typedefs ------------------------------------------------------------------*/

typedef enum {
  LSM6DSV16X_OK = 0,
  LSM6DSV16X_ERROR = -1
} LSM6DSV16XStatusTypeDef;

typedef enum {
  LSM6DSV16X_INT1_PIN,
  LSM6DSV16X_INT2_PIN,
} LSM6DSV16X_SensorIntPin_t;

typedef struct {
  unsigned int FreeFallStatus : 1;
  unsigned int TapStatus : 1;
  unsigned int DoubleTapStatus : 1;
  unsigned int WakeUpStatus : 1;
  unsigned int StepStatus : 1;
  unsigned int TiltStatus : 1;
  unsigned int D6DOrientationStatus : 1;
  unsigned int SleepStatus : 1;
} LSM6DSV16X_Event_Status_t;

typedef union {
  int16_t i16bit[3];
  uint8_t u8bit[6];
} lsm6dsv16x_axis3bit16_t;

typedef union {
  int16_t i16bit;
  uint8_t u8bit[2];
} lsm6dsv16x_axis1bit16_t;

typedef enum {
  LSM6DSV16X_RESET_GLOBAL = 0x1,
  LSM6DSV16X_RESET_CAL_PARAM = 0x2,
  LSM6DSV16X_RESET_CTRL_REGS = 0x4,
} LSM6DSV16X_Reset_t;

typedef enum {
  LSM6DSV16X_ACC_HIGH_PERFORMANCE_MODE,
  LSM6DSV16X_ACC_HIGH_ACCURACY_MODE,
  LSM6DSV16X_ACC_NORMAL_MODE,
  LSM6DSV16X_ACC_LOW_POWER_MODE1,
  LSM6DSV16X_ACC_LOW_POWER_MODE2,
  LSM6DSV16X_ACC_LOW_POWER_MODE3
} LSM6DSV16X_ACC_Operating_Mode_t;

typedef enum {
  LSM6DSV16X_GYRO_HIGH_PERFORMANCE_MODE,
  LSM6DSV16X_GYRO_HIGH_ACCURACY_MODE,
  LSM6DSV16X_GYRO_SLEEP_MODE,
  LSM6DSV16X_GYRO_LOW_POWER_MODE
} LSM6DSV16X_GYRO_Operating_Mode_t;

/* Class Declaration ---------------------------------------------------------*/

/**
 * Abstract class of a LSM6DSV16X sensor.
 */
class LSM6DSV16XSensor {
public:
#ifdef PICO
  LSM6DSV16XSensor(i2c_inst_t *i2c, uint8_t address = LSM6DSV16X_I2C_ADD_H);
  LSM6DSV16XSensor(spi_inst_t *spi, int cs_pin, uint32_t spi_speed = 2000000);
#else
  LSM6DSV16XSensor(TwoWire *i2c, uint8_t address = LSM6DSV16X_I2C_ADD_H);
  LSM6DSV16XSensor(SPIClass *spi, int cs_pin, uint32_t spi_speed = 2000000);
#endif
  LSM6DSV16XStatusTypeDef begin();
  LSM6DSV16XStatusTypeDef end();
  LSM6DSV16XStatusTypeDef read_whoami(uint8_t *Id);

  LSM6DSV16XStatusTypeDef enable_x();
  LSM6DSV16XStatusTypeDef disbale_x();
  LSM6DSV16XStatusTypeDef get_x_sensitivity(float *Sensitivity);
  LSM6DSV16XStatusTypeDef get_x_odr(float *Odr);
  LSM6DSV16XStatusTypeDef set_x_odr(float Odr,
                                    LSM6DSV16X_ACC_Operating_Mode_t Mode =
                                        LSM6DSV16X_ACC_HIGH_PERFORMANCE_MODE);
  LSM6DSV16XStatusTypeDef get_x_fs(int32_t *FullScale);
  LSM6DSV16XStatusTypeDef set_x_fs(int32_t FullScale);
  LSM6DSV16XStatusTypeDef get_x_axes_raw(int16_t *Value);
  LSM6DSV16XStatusTypeDef get_x_axes(int32_t *Acceleration);
  LSM6DSV16XStatusTypeDef get_x_drdy_status(uint8_t *Status);
  LSM6DSV16XStatusTypeDef get_x_event_status(LSM6DSV16X_Event_Status_t *Status);
  LSM6DSV16XStatusTypeDef set_x_power_mode(uint8_t PowerMode);
  LSM6DSV16XStatusTypeDef set_x_filter_mode(uint8_t LowHighPassFlag,
                                            uint8_t FilterMode);
  LSM6DSV16XStatusTypeDef enable_x_user_offset();
  LSM6DSV16XStatusTypeDef disable_x_user_offset();
  LSM6DSV16XStatusTypeDef set_x_user_offset(float x, float y, float z);

  LSM6DSV16XStatusTypeDef enable_g();
  LSM6DSV16XStatusTypeDef disable_g();
  LSM6DSV16XStatusTypeDef get_g_sensitivity(float *Sensitivity);
  LSM6DSV16XStatusTypeDef get_g_odr(float *Odr);
  LSM6DSV16XStatusTypeDef set_g_odr(float Odr,
                                    LSM6DSV16X_GYRO_Operating_Mode_t Mode =
                                        LSM6DSV16X_GYRO_HIGH_PERFORMANCE_MODE);
  LSM6DSV16XStatusTypeDef get_g_fs(int32_t *FullScale);
  LSM6DSV16XStatusTypeDef set_g_fs(int32_t FullScale);
  LSM6DSV16XStatusTypeDef get_g_axes_raw(int16_t *Value);
  LSM6DSV16XStatusTypeDef get_g_axes(int32_t *AngularRate);
  LSM6DSV16XStatusTypeDef get_g_drdy_status(uint8_t *Status);
  LSM6DSV16XStatusTypeDef set_g_power_mode(uint8_t PowerMode);
  LSM6DSV16XStatusTypeDef set_g_filter_mode(uint8_t LowHighPassFlag,
                                            uint8_t FilterMode);

  LSM6DSV16XStatusTypeDef get_temp_odr(float *Odr);
  LSM6DSV16XStatusTypeDef set_temp_odr(float Odr);
  LSM6DSV16XStatusTypeDef get_temp_odr(int16_t *value);

  LSM6DSV16XStatusTypeDef test_imu(uint8_t XTestType, uint8_t GTestType);
  LSM6DSV16XStatusTypeDef test_x_imu(uint8_t TestType);
  LSM6DSV16XStatusTypeDef test_g_imu(uint8_t TestType);

  LSM6DSV16XStatusTypeDef enable_6d_orientation(LSM6DSV16X_SensorIntPin_t IntPin);
  LSM6DSV16XStatusTypeDef disable_6d_orientation();
  LSM6DSV16XStatusTypeDef set_6d_orientation_threshold(uint8_t Threshold);
  LSM6DSV16XStatusTypeDef get_6d_orientation_xl(uint8_t *XLow);
  LSM6DSV16XStatusTypeDef get_6d_orientation_xh(uint8_t *XHigh);
  LSM6DSV16XStatusTypeDef get_6d_orientation_yl(uint8_t *YLow);
  LSM6DSV16XStatusTypeDef get_6d_orientation_yh(uint8_t *YHigh);
  LSM6DSV16XStatusTypeDef get_6d_orientation_zl(uint8_t *ZLow);
  LSM6DSV16XStatusTypeDef get_6d_orientation_zh(uint8_t *ZHigh);

  LSM6DSV16XStatusTypeDef enable_free_fall_detection(LSM6DSV16X_SensorIntPin_t IntPin);
  LSM6DSV16XStatusTypeDef disable_free_fall_detection();
  LSM6DSV16XStatusTypeDef set_free_fall_threshold(uint8_t Threshold);
  LSM6DSV16XStatusTypeDef set_free_fall_duration(uint8_t Duration);

  LSM6DSV16XStatusTypeDef enable_wake_up_detection(LSM6DSV16X_SensorIntPin_t IntPin);
  LSM6DSV16XStatusTypeDef disable_wake_up_detection();
  LSM6DSV16XStatusTypeDef set_wake_up_threshold(uint32_t Threshold);
  LSM6DSV16XStatusTypeDef set_wake_up_duration(uint8_t Duration);

  LSM6DSV16XStatusTypeDef enable_single_tap_detection(LSM6DSV16X_SensorIntPin_t IntPin);
  LSM6DSV16XStatusTypeDef disable_single_tap_detection();
  LSM6DSV16XStatusTypeDef enable_double_tap_detection(LSM6DSV16X_SensorIntPin_t IntPin);
  LSM6DSV16XStatusTypeDef disable_double_tap_detection();
  LSM6DSV16XStatusTypeDef set_tap_threshold(uint8_t Threshold);
  LSM6DSV16XStatusTypeDef set_tap_shock_time(uint8_t Time);
  LSM6DSV16XStatusTypeDef set_tap_quit_time(uint8_t Time);
  LSM6DSV16XStatusTypeDef set_tap_duration_time(uint8_t Time);

  LSM6DSV16XStatusTypeDef enable_pedometer(LSM6DSV16X_SensorIntPin_t IntPin);
  LSM6DSV16XStatusTypeDef disable_pedometer();
  LSM6DSV16XStatusTypeDef get_step_count(uint16_t *StepCount);
  LSM6DSV16XStatusTypeDef step_count_reset();

  LSM6DSV16XStatusTypeDef enable_tilt_detection(LSM6DSV16X_SensorIntPin_t IntPin);
  LSM6DSV16XStatusTypeDef disable_tilt_detection();

  LSM6DSV16XStatusTypeDef fifo_get_nom_samples(uint16_t *NumSamples);
  LSM6DSV16XStatusTypeDef fifo_get_full_status(uint8_t *Status);
  LSM6DSV16XStatusTypeDef fifo_set_int1_fifo_full(uint8_t Status);
  LSM6DSV16XStatusTypeDef fifo_set_int2_fifo_full(uint8_t Status);
  LSM6DSV16XStatusTypeDef fifo_set_watermark_level(uint8_t Watermark);
  LSM6DSV16XStatusTypeDef fifo_set_stop_on_fth(uint8_t Status);
  LSM6DSV16XStatusTypeDef fifo_set_mode(uint8_t Mode);
  LSM6DSV16XStatusTypeDef fifo_get_tag(uint8_t *Tag);
  LSM6DSV16XStatusTypeDef fifo_get_data(uint8_t *Data);
  LSM6DSV16XStatusTypeDef fifo_get_x_axes(int32_t *Acceleration);
  LSM6DSV16XStatusTypeDef fifo_set_x_bdr(float Bdr);
  LSM6DSV16XStatusTypeDef fifo_get_g_axes(int32_t *AngularVelocity);
  LSM6DSV16XStatusTypeDef fifo_set_g_bdr(float Bdr);
  LSM6DSV16XStatusTypeDef fifo_get_status(lsm6dsv16x_fifo_status_t *Status);
  LSM6DSV16XStatusTypeDef fifo_get_rotation_vector(float *rvec);
  LSM6DSV16XStatusTypeDef fifo_get_gravity_vector(float *gvec);
  LSM6DSV16XStatusTypeDef fifo_get_gyroscope_bias(float *gbias);
  LSM6DSV16XStatusTypeDef fifo_enable_timestamp();
  LSM6DSV16XStatusTypeDef fifo_disable_timestamp();
  LSM6DSV16XStatusTypeDef fifo_set_timestamp_decimation(uint8_t decimation);
  LSM6DSV16XStatusTypeDef fifo_get_timestamp(uint32_t *timestamp);
  LSM6DSV16XStatusTypeDef fifo_reset();

  LSM6DSV16XStatusTypeDef qvar_enable();
  LSM6DSV16XStatusTypeDef qvar_disable();
  LSM6DSV16XStatusTypeDef qvar_get_status(uint8_t *val);
  LSM6DSV16XStatusTypeDef qvar_get_impedance(uint16_t *val);
  LSM6DSV16XStatusTypeDef qvar_set_impedance(uint16_t val);
  LSM6DSV16XStatusTypeDef qvar_get_data(float *Data);

  LSM6DSV16XStatusTypeDef get_mlc_status(lsm6dsv16x_mlc_status_mainpage_t *status);
  LSM6DSV16XStatusTypeDef get_mlc_output(lsm6dsv16x_mlc_out_t *output);

  LSM6DSV16XStatusTypeDef enable_rotation_vector();
  LSM6DSV16XStatusTypeDef disable_rotation_vector();
  LSM6DSV16XStatusTypeDef enable_gravity_vector();
  LSM6DSV16XStatusTypeDef disable_gravity_vector();
  LSM6DSV16XStatusTypeDef enable_gyroscope_bias();
  LSM6DSV16XStatusTypeDef disable_gyroscope_bias();
  LSM6DSV16XStatusTypeDef set_sflp_batch(bool GameRotation, bool Gravity,
                                         bool gBias);
  LSM6DSV16XStatusTypeDef set_sflp_odr(float Odr);
  LSM6DSV16XStatusTypeDef set_sflp_gbias(float x, float y, float z);
  LSM6DSV16XStatusTypeDef reset_sflp();

  LSM6DSV16XStatusTypeDef read_reg(uint8_t Reg, uint8_t *Data);
  LSM6DSV16XStatusTypeDef write_reg(uint8_t Reg, uint8_t Data);

  LSM6DSV16XStatusTypeDef enable_block_data_update();
  LSM6DSV16XStatusTypeDef disable_block_data_update();
  LSM6DSV16XStatusTypeDef enable_auto_increment();
  LSM6DSV16XStatusTypeDef disable_auto_increment();
  LSM6DSV16XStatusTypeDef device_reset(LSM6DSV16X_Reset_t flags = LSM6DSV16X_RESET_GLOBAL);

/**
 * @brief Utility function to read data.
 * @param  pBuffer: pointer to data to be read.
 * @param  RegisterAddr: specifies internal address register to be read.
 * @param  NumByteToRead: number of bytes to be read.
 * @retval 0 if ok, an error code otherwise.
 */
#ifdef PICO
  uint8_t IO_Read(uint8_t *pBuffer, uint8_t RegisterAddr,
                  uint16_t NumByteToRead) {
    if (dev_spi) {
      // SPI transaction
      uint8_t readAdress = (RegisterAddr | 0x80);// |0x80 to enable read bit 
      gpio_put(cs_pin, 0); // CS low
      spi_write_blocking(dev_spi,&readAdress,1) ;
      spi_read_blocking(dev_spi,0x00,pBuffer,NumByteToRead);
      gpio_put(cs_pin, 1);        // CS high

      return 0;
    }

    if (dev_i2c) {
      // I2C transaction
      uint8_t addr = (address >> 1) & 0x7F;
      i2c_write_blocking(dev_i2c, addr, &RegisterAddr, 1,
                         false); // Write register address
      i2c_read_blocking(dev_i2c, addr, pBuffer, NumByteToRead,
                        false); // Read data

      return 0;
    }

    return 1;
  }

#else
  uint8_t IO_Read(uint8_t *pBuffer, uint8_t RegisterAddr,
                  uint16_t NumByteToRead) {
    if (dev_spi) {
      dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));

      digitalWrite(cs_pin, LOW);

      /* Write Reg Address */
      dev_spi->transfer(RegisterAddr | 0x80);
      /* Read the data */
      for (uint16_t i = 0; i < NumByteToRead; i++) {
        *(pBuffer + i) = dev_spi->transfer(0x00);
      }

      digitalWrite(cs_pin, HIGH);

      dev_spi->endTransaction();

      return 0;
    }

    if (dev_i2c) {
      dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));
      dev_i2c->write(RegisterAddr);
      dev_i2c->endTransmission(false);

      dev_i2c->requestFrom(((uint8_t)(((address) >> 1) & 0x7F)),
                           (uint8_t)NumByteToRead);

      int i = 0;
      while (dev_i2c->available()) {
        pBuffer[i] = dev_i2c->read();
        i++;
      }

      return 0;
    }

    return 1;
  }
#endif

#ifdef PICO
  uint8_t IO_Write(const uint8_t *pBuffer, uint8_t RegisterAddr,
                   uint16_t NumByteToWrite) {
    if (dev_spi) {
      // SPI transaction
      gpio_put(cs_pin, 0);                           // CS low
      spi_write_blocking(dev_spi, &RegisterAddr, 1); // Write register address
      spi_write_blocking(dev_spi, pBuffer, NumByteToWrite); // Write data
      gpio_put(cs_pin, 1);                                  // CS high

      return 0;
    }

    if (dev_i2c) {
      // I2C transaction
      uint8_t addr = (address >> 1) & 0x7F;
      uint8_t
          txBuffer[NumByteToWrite + 1]; // Buffer for register address + data
      txBuffer[0] = RegisterAddr;
      memcpy(&txBuffer[1], pBuffer, NumByteToWrite);

      i2c_write_blocking(dev_i2c, addr, txBuffer, NumByteToWrite + 1,
                         true); // Write register + data

      return 0;
    }

    return 1;
  }

#else
  /**
   * @brief Utility function to write data.
   * @param  pBuffer: pointer to data to be written.
   * @param  RegisterAddr: specifies internal address register to be written.
   * @param  NumByteToWrite: number of bytes to write.
   * @retval 0 if ok, an error code otherwise.
   */
  uint8_t IO_Write(const uint8_t *pBuffer, uint8_t RegisterAddr,
                   uint16_t NumByteToWrite) {
    if (dev_spi) {
      dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));

      digitalWrite(cs_pin, LOW);

      /* Write Reg Address */
      dev_spi->transfer(RegisterAddr);
      /* Write the data */
      for (uint16_t i = 0; i < NumByteToWrite; i++) {
        dev_spi->transfer(pBuffer[i]);
      }

      digitalWrite(cs_pin, HIGH);

      dev_spi->endTransaction();

      return 0;
    }

    if (dev_i2c) {
      dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));

      dev_i2c->write(RegisterAddr);
      for (uint16_t i = 0; i < NumByteToWrite; i++) {
        dev_i2c->write(pBuffer[i]);
      }

      dev_i2c->endTransmission(true);

      return 0;
    }

    return 1;
  }
#endif

private:
  LSM6DSV16XStatusTypeDef set_x_odr_when_enabled(float Odr);
  LSM6DSV16XStatusTypeDef set_x_odr_when_disabled(float Odr);
  LSM6DSV16XStatusTypeDef set_g_odr_when_enabled(float Odr);
  LSM6DSV16XStatusTypeDef set_g_odr_when_disabled(float Odr);
  LSM6DSV16XStatusTypeDef get_x_axes_raw_when_aval(int16_t *Value);
  LSM6DSV16XStatusTypeDef get_g_axes_raw_when_aval(int16_t *Value);
  LSM6DSV16XStatusTypeDef npy_halfbits_to_floatbits(uint16_t h, uint32_t *f);
  LSM6DSV16XStatusTypeDef npy_half_to_float(uint16_t h, float *f);
  LSM6DSV16XStatusTypeDef sflp2q(float quat[4], uint16_t sflp[3]);

  float convert_x_sensitivity(lsm6dsv16x_xl_full_scale_t full_scale);
  float convert_g_sensitivity(lsm6dsv16x_gy_full_scale_t full_scale);

  /* Helper classes. */

#ifdef PICO
  i2c_inst_t *dev_i2c; // For I2C
  spi_inst_t *dev_spi; // For SPI
#else
  TwoWire *dev_i2c;
  SPIClass *dev_spi;
#endif
  /* Configuration */
  uint8_t address;
  int cs_pin;
  uint32_t spi_speed;

  lsm6dsv16x_data_rate_t acc_odr;
  lsm6dsv16x_data_rate_t gyro_odr;
  lsm6dsv16x_xl_full_scale_t acc_fs;
  lsm6dsv16x_gy_full_scale_t gyro_fs;
  lsm6dsv16x_fifo_mode_t fifo_mode;
  uint8_t acc_is_enabled;
  uint8_t gyro_is_enabled;
  uint8_t initialized;
  lsm6dsv16x_ctx_t reg_ctx;
};

#ifdef __cplusplus
extern "C" {
#endif
int32_t LSM6DSV16X_io_write(void *handle, uint8_t WriteAddr,
                            const uint8_t *pBuffer, uint16_t nBytesToWrite);
int32_t LSM6DSV16X_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer,
                           uint16_t nBytesToRead);
void LSM6DSV16X_sleep(uint32_t ms);
#ifdef __cplusplus
}
#endif

#endif /* __LSM6DSV16XSensor_H__ */
