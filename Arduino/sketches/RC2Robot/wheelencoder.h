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
 */

#ifndef wheelencoder_h
#define wheelencoder_h
#include "Arduino.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// The following pin deffinitions can be changed.
// If changed, additional changes in the code are required

// UNO
#ifdef ARDUINO_AVR_UNO
  #define LEFT_ENCODER_PIN_A 2
  #define LEFT_ENC_PIN_A PB4
  #define LEFT_ENCODER_PIN_B 3
  #define LEFT_ENC_PIN_B PB5

  //#define RIGHT_ENCODER_PIN_A 10
  //#define RIGHT_ENC_PIN_A PB6
  //#define RIGHT_ENCODER_PIN_B 11
  //#define RIGHT_ENC_PIN_B PB7

  #define LED_ENC_A 11
  #define LED_ENC_B 10
#endif

// 32U4: ATMega
//  pololu A-Star 32U4 Micro
//  See https://www.pololu.com/product/3101
#ifdef ARDUINO_AVR_A_STAR_32U4
  #define LEFT_ENCODER_PIN_A 8
  #define LEFT_ENC_PIN_A PB4
  #define LEFT_ENCODER_PIN_B 9
  #define LEFT_ENC_PIN_B PB5

  #define RIGHT_ENCODER_PIN_A 10
  #define RIGHT_ENC_PIN_A PB6
  #define RIGHT_ENCODER_PIN_B 11
  #define RIGHT_ENC_PIN_B PB7
#endif

// ESP32
#ifdef ESP32
  #define LEFT_ENCODER_PIN_A 15
  #define LEFT_ENC_PIN_A GPIO15
  #define LEFT_ENCODER_PIN_B 2
  #define LEFT_ENC_PIN_B GPIO2

  #define RIGHT_ENCODER_PIN_A 27
  #define RIGHT_ENC_PIN_A GPIO12
  #define RIGHT_ENCODER_PIN_B 26
  #define RIGHT_ENC_PIN_B GPIO14
#endif


// Left/right encoder
enum ENCODER {  LEFT_ENCODER,
  RIGHT_ENCODER
};

void initWheelEncoder();
void resetEncoders();
long readTicks(int i);

#endif
