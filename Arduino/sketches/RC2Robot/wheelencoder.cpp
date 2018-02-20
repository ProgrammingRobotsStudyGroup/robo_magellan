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

volatile double left_speed = 0.0;
volatile double right_speed = 0.0;

static const signed char ENC_STATES [] = {0,1,-1,0, -1,0,0,1, 1,0,0,-1, 0,-1,1,0};

#ifndef opt0
/* Interrupt routine for LEFT encoder, taking care of actual counting */
ISR (PCINT0_vect) {
  static unsigned long last_time = 0;
  static long last_dt = 0;
  static unsigned char last_enc = 0;

  // Shift previous state two places
  last_enc <<= 2;

  // Read the current state into lowest 2 bits
  last_enc |= (PINB & (3 << 4)) >> 4;

  signed char stat = ENC_STATES[(last_enc & 0x0f)];

  if (stat != 0) {
    unsigned long t = micros();
    long dt = t - last_time;

    last_time = t;

    if (1000 < dt && dt < 150000) {
      dt *= stat;

      if (last_dt < 0 && dt > 0 || last_dt > 0 && dt < 0) {
        last_dt = 0;
      }

      last_dt = ((last_dt * 4) + dt) / 5;

      left_speed = 25000.0 / (double)last_dt;
    }
  }
}
#endif

/* Interrupt routine for RIGHT encoder, taking care of actual counting */
// ISR (PCINT1_vect){
//   static unsigned long last_time = 0;
//   static long last_dt = 0;
//   static unsigned char last_enc = 0;
//
//   last_enc <<= 2; //shift previous state two places
//
//   last_enc |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits
//
//   signed char stat = ENC_STATES[(last_enc & 0x0f)];
//
//   if (stat != 0) {
//     unsigned long t = micros();
//     long dt = t - last_time;
//
//     last_time = t;
//
//     if (1000 < dt && dt < 150000) {
//       dt *= stat;
//
//       if (last_dt < 0 && dt > 0 || last_dt > 0 && dt < 0) {
//         last_dt = 0;
//       }
//
//       last_dt = ((last_dt * 4) + dt) / 5;
//
//       right_speed = 25000.0 / (double)last_dt;
//     }
//   }
// }

/**
 * 
 */
void initWheelEncoder() {
  //set as inputs
  DDRB &= ~(1 << LEFT_ENC_PIN_A);
  DDRB &= ~(1 << LEFT_ENC_PIN_B);
  DDRB &= ~(1 << RIGHT_ENC_PIN_A);
  DDRB &= ~(1 << RIGHT_ENC_PIN_B);
  //enable pull up resistors
  PORTB |= (1 << LEFT_ENC_PIN_A);
  PORTB |= (1 << LEFT_ENC_PIN_B);
  PORTB |= (1 << RIGHT_ENC_PIN_A);
  PORTB |= (1 << RIGHT_ENC_PIN_B);
  // tell pin change mask to listen to left encoder pins
  PCMSK0 |= (1 << LEFT_ENC_PIN_A) | (1 << LEFT_ENC_PIN_B);
  // tell pin change mask to listen to right encoder pins
  PCMSK0 |= (1 << RIGHT_ENC_PIN_A) | (1 << RIGHT_ENC_PIN_B);

  // enable PCINT0 interrupt in the general interrupt mask
  PCICR |= 1 << PCIE0;
  resetEncoders();
}

/* Wrap the encoder reading function */
double readSpeed(int i) {
  if (i == LEFT) {
    return left_speed;
  }
  else {
    return right_speed;
  }
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
  if (i == LEFT) {
    left_speed = 0.0;
    return;
  }
  else {
    right_speed = 0.0;
    return;
  }
}

void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

