/**
 * Copyright 2018 Robot Garden, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * A set of sensor service routines to manage:
 * - Wheel encoder
 * - Touch sensor
 * - Serial com with upstream host
 */

#include "PRSG.h"

//
#define USE_I2C
#define SERIAL_STREAM Serial
#define DEBUG_SERIAL_STREAM Serial

#define TOUCH_SENSORS true

void setup() {
  Serial.begin(115200);
  initCommand();

  initWheelEncoder();

  #ifdef TOUCH_SENSORS
    initTouchSensor(TOUCH_SWITCH_QTY);
  #endif
}

/**
 * 
 */
unsigned long timer = 0;
const unsigned long _delay = 50;

void loop() {
  // Process input
  unsigned long newtime = millis();
  unsigned long elapsed = newtime - timer;

  processCommandChar();
  if (elapsed > _delay) {
    timer = newtime;

    #ifdef TOUCH_SENSORS
    // Display touch sensor state
    Serial.print("B ");
    Serial.print(!digitalRead(TOUCH_SW_1));
    Serial.print(' ');
    Serial.print(!digitalRead(TOUCH_SW_2));
    Serial.print(' ');
    Serial.println(!digitalRead(TOUCH_SW_3));
    #endif

    // Show left and right encoder ticks.
    Serial.print("E ");
    Serial.print(elapsed);
    Serial.print(' ');
    Serial.print(readTicks(LEFT_ENCODER));
    Serial.print(' ');
    Serial.print(readTicks(RIGHT_ENCODER));
    Serial.println();
  }
}


