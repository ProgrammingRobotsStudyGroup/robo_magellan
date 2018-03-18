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

#include "wheelencoder.h"
#include <FastGPIO.h>
#include <EnableInterrupt.h>

static const long MIN_CHANGE_TIME = 1000; // 1000us = 1ms is minimum time between ticks.

static volatile long left_ticks = 0;
static volatile long right_ticks = 0;

static volatile int last_left_state;
static volatile long last_left_time;

static volatile int last_right_state;
static volatile long last_right_time;

static const signed char ENC_STATES [] = {
  0,  // 00 -> 00
  1,  // 00 -> 01
  -1, // 00 -> 10
  0,  // 00 -> 11 (shouldn't happen)
  -1, // 01 -> 00
  0,  // 01 -> 01
  0,  // 01 -> 10 (shouldn't happen)
  1,  // 01 -> 11
  1,  // 10 -> 00
  0,  // 10 -> 01 (shouldn't happen)
  0,  // 10 -> 10
  -1, // 10 -> 11
  0,  // 11 -> 00 (shouldn't happen)
  -1, // 11 -> 01
  1,  // 11 -> 10
  0   // 11 -> 11
};

static void updateTicks(volatile long &ticks, volatile int &last_state, volatile long &last_time,
  int new_state) {

  long new_time = micros();
  if (new_time-last_time > MIN_CHANGE_TIME) {
    ticks += ENC_STATES[last_state<<2 | new_state];
    last_state = new_state;
    last_time = new_time;
  }
}

// Update the left encoder when either encoder pin changes.
static void leftISR() {
  int new_state = FastGPIO::Pin<LEFT_ENCODER_PIN_A>::isInputHigh()<<1
     | FastGPIO::Pin<LEFT_ENCODER_PIN_B>::isInputHigh();
  updateTicks(left_ticks, last_left_state, last_left_time, new_state);
}

// Update the encoder when the right A pin changes.
static void rightISR() {
  int new_state = FastGPIO::Pin<RIGHT_ENCODER_PIN_A>::isInputHigh()<<1
     | FastGPIO::Pin<RIGHT_ENCODER_PIN_B>::isInputHigh();
  updateTicks(right_ticks, last_right_state, last_right_time, new_state);
}

/**
 * 
 */
void initWheelEncoder() {
  FastGPIO::Pin<LEFT_ENCODER_PIN_A>::setInputPulledUp();
  FastGPIO::Pin<LEFT_ENCODER_PIN_B>::setInputPulledUp();

  FastGPIO::Pin<RIGHT_ENCODER_PIN_A>::setInputPulledUp();
  FastGPIO::Pin<RIGHT_ENCODER_PIN_B>::setInputPulledUp();

  enableInterrupt(LEFT_ENCODER_PIN_A, leftISR, CHANGE);
  enableInterrupt(LEFT_ENCODER_PIN_B, leftISR, CHANGE);
  enableInterrupt(RIGHT_ENCODER_PIN_A, rightISR, CHANGE);
  enableInterrupt(RIGHT_ENCODER_PIN_B, rightISR, CHANGE);

  resetEncoders();
}

/* Wrap the encoder reading function */
double readTicks(int i) {
  if (i == LEFT_ENCODER) {
    return left_ticks;
  } else {
    return right_ticks;
  }
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
  if (i == LEFT_ENCODER) {
    left_ticks = 0;
    last_left_state = FastGPIO::Pin<LEFT_ENCODER_PIN_A>::isInputHigh()<<1
      | FastGPIO::Pin<LEFT_ENCODER_PIN_B>::isInputHigh();
    last_left_time = micros();
  } else {
    right_ticks = 0;
    last_right_state = FastGPIO::Pin<RIGHT_ENCODER_PIN_A>::isInputHigh()<<1
      | FastGPIO::Pin<RIGHT_ENCODER_PIN_B>::isInputHigh();
    last_right_time = micros();
  }
}

void resetEncoders() {
  resetEncoder(LEFT_ENCODER);
  resetEncoder(RIGHT_ENCODER);
}

