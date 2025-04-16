/*
   @file    LSM6DSV16X_Sensor_Fusion.ino
   @author  STMicroelectronics
   @brief   Example to use the LSM6DSV16X library with Sensor Fusion Low Power.
 *******************************************************************************
   Copyright (c) 2022, STMicroelectronics
   All rights reserved.

   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause

 *******************************************************************************
*/

/*
 * You can display the quaternion values with a 3D model connecting for example to this link:
 * https://adafruit.github.io/Adafruit_WebSerial_3DModelViewer/
 */

#include <LSM6DSV16XSensor.h>

#define ALGO_FREQ  120U /* Algorithm frequency 120Hz */
#define ALGO_PERIOD  (1000U / ALGO_FREQ) /* Algorithm period [ms] */
unsigned long startTime, elapsedTime;

LSM6DSV16XSensor AccGyr(&Wire);
uint8_t status = 0;
uint32_t k = 0;

uint8_t tag = 0;
float quaternions[4] = {0};

void setup()
{

  Serial.begin(115200);
  while (!Serial) {
    yield();
  }

  Wire.begin();
  // Initialize LSM6DSV16X.
  AccGyr.begin();
  AccGyr.enable_x();
  AccGyr.enable_g();

  // Enable Sensor Fusion
  status |= AccGyr.set_x_fs(4);
  status |= AccGyr.set_g_fs(2000);
  status |= AccGyr.set_x_odr(120.0f);
  status |= AccGyr.set_g_odr(120.0f);
  status |= AccGyr.set_sflp_odr(120.0f);
  status |= AccGyr.enable_rotation_vector();
  status |= AccGyr.fifo_set_mode(LSM6DSV16X_STREAM_MODE);

  if (status != LSM6DSV16X_OK) {
    Serial.println("LSM6DSV16X Sensor failed to init/configure");
    while (1);
  }
  Serial.println("LSM6DSV16X SFLP Demo");
}

void loop()
{
  uint16_t fifo_samples;
  // Get start time of loop cycle
  startTime = millis();

  // Check the number of samples inside FIFO
  if (AccGyr.fifo_get_nom_samples(&fifo_samples) != LSM6DSV16X_OK) {
    Serial.println("LSM6DSV16X Sensor failed to get number of samples inside FIFO");
    while (1);
  }

  // Read the FIFO if there is one stored sample
  if (fifo_samples > 0) {
    for (int i = 0; i < fifo_samples; i++) {
      AccGyr.fifo_get_tag(&tag);
      if (tag == 0x13u) {
        AccGyr.fifo_get_rotation_vector(&quaternions[0]);

        // Print Quaternion data
        Serial.print("Quaternion: ");
        Serial.print(quaternions[3], 4);
        Serial.print(", ");
        Serial.print(quaternions[0], 4);
        Serial.print(", ");
        Serial.print(quaternions[1], 4);
        Serial.print(", ");
        Serial.println(quaternions[2], 4);

        // Compute the elapsed time within loop cycle and wait
        elapsedTime = millis() - startTime;

        if ((long)(ALGO_PERIOD - elapsedTime) > 0) {
          delay(ALGO_PERIOD - elapsedTime);
        }
      }
    }
  }
}
