/*
   @file    LSM6DSV16X_Single_Tap_Detection.ino
   @author  STMicroelectronics
   @brief   Example to use the LSM6DSV16X Single Tap Detection
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

#define INT1_pin PA4

LSM6DSV16XSensor LSM6DSV16X(&Wire);
//Interrupts.
volatile int mems_event = 0;

void INT1Event_cb();

void setup()
{
  Serial.begin(115200);
  delay(1000);
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();

  //Interrupts.
  attachInterrupt(INT1_pin, INT1Event_cb, RISING);

  // Initlialize components.
  LSM6DSV16X.begin();
  LSM6DSV16X.enable_x();

  // Enable Single Tap Detection.
  LSM6DSV16X.enable_single_tap_detection(LSM6DSV16X_INT1_PIN);
}

void loop()
{
  if (mems_event) {
    mems_event = 0;
    LSM6DSV16X_Event_Status_t status;
    LSM6DSV16X.get_x_event_status(&status);
    if (status.TapStatus) {

      // Led blinking.
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      Serial.println("Single Tap Detected!");
    }
  }
}

void INT1Event_cb()
{
  mems_event = 1;
}
