/*
   @file    LSM6DSV16X_Qvar.ino
   @author  STMicroelectronics
   @brief   Example to use LSM6DSV16X Qvar features
 *******************************************************************************
   Copyright (c) 2022, STMicroelectronics
   All rights reserved.

   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause

 *******************************************************************************
*/
#include <LSM6DSV16XSensor.h>
LSM6DSV16XSensor AccGyr(&Wire);

void setup()
{

  Serial.begin(115200);
  Wire.begin();

  // Initialize LSM6DSV16X.
  AccGyr.begin();

  // Enable accelerometer and gyroscope.
  AccGyr.enable_x();
  AccGyr.enable_g();

  // Enable QVAR
  if (AccGyr.qvar_enable() != 0) {
    Serial.println("Error during initialization of QVAR");
    while (1);
  }
  Serial.println("LSM6DSV16X QVAR Demo");
}

void loop()
{
  uint8_t qvar_status;
  float qvar_data;
  // Check if QVAR data is ready
  AccGyr.qvar_get_status(&qvar_status);
  if (qvar_status) {
    // Get QVAR data
    AccGyr.qvar_get_data(&qvar_data);
    Serial.println(qvar_data);
  }
}