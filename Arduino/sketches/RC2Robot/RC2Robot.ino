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
 * - Bumper
 * - Serial com with upstream host
 */

#include "PRSG.h"

//
#define USE_I2C
#define SERIAL_STREAM Serial
#define DEBUG_SERIAL_STREAM Serial

void setup() {
  Serial.begin(115200);
  initCommand();
  //
  initWheelEncoder();
  //
  initBumper();
}
/**
 * 
 */
void dispPinVal() {
    // Display pin vals
    int pinVal;
    pinVal = digitalRead(LEFT_ENC_PIN_A);
    Serial.print(pinVal);
    //
    Serial.print("-");
    pinVal = digitalRead(LEFT_ENC_PIN_B);
    Serial.print(pinVal);
    //
    Serial.print("-");
    pinVal = digitalRead(RIGHT_ENC_PIN_A);
    Serial.print(pinVal);
    //
    Serial.print("-");
    pinVal = digitalRead(RIGHT_ENC_PIN_B);
    Serial.print(pinVal);
    //
    Serial.println();
}

/**
 * 
 */
unsigned long timer = 0;
#define _delay 1000

void loop() {
  // Process input
  int newtime = millis();
  processCommandChar();
  if (newtime>(timer+_delay)) {
    timer = newtime;
    //
//    dispPinVal();
    //
    double vel;
    vel = readSpeed(LEFT);
    Serial.print("L:");
    Serial.print(vel);
    Serial.print(" R:");
    vel = readSpeed(RIGHT);
    Serial.println(vel);
  }
}


