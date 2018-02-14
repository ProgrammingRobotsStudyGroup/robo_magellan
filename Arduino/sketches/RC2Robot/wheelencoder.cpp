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
#define opt0

#include "wheelencoder.h"
#ifdef opt0
#include <AStar32U4.h>
//#include <EnableInterrupt.h>
#else
#endif

/**
 * 
 */
void processWheelEncoder() {

}


// below can be changed, but should be PORTD pins;
// otherwise additional changes in the code are required
#define LEFT_ENC_PIN_A PD2  // pololu mini a* pin 0
#define LEFT_ENC_PIN_B PD3  // pololu mini a* pin 1

// below can be changed, but should be PORTC pins
// #define RIGHT_ENC_PIN_A PC4  //pin A4
// #define RIGHT_ENC_PIN_B PC5  //pin A5

volatile double left_speed = 0.0;
volatile double right_speed = 0.0;

static const signed char ENC_STATES [] = {0,1,-1,0, -1,0,0,1, 1,0,0,-1, 0,-1,1,0};

#ifdef opt0
#else
/* Interrupt routine for LEFT encoder, taking care of actual counting */
//ISR (PCINT0_vect) {
//  Serial.println("X");
//  static unsigned long last_time = 0;
//  static long last_dt = 0;
//  static unsigned char last_enc = 0;
//
//  last_enc <<= 2; //shift previous state two places
//  last_enc |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits
//
//  signed char stat = ENC_STATES[(last_enc & 0x0f)];
//
//  if (stat != 0) {
//    unsigned long t = micros();
//    long dt = t - last_time;
//
//    last_time = t;
//
//    if (1000 < dt && dt < 150000) {
//      dt *= stat;
//
//      if (last_dt < 0 && dt > 0 || last_dt > 0 && dt < 0) {
//        last_dt = 0;
//      }
//
//      last_dt = ((last_dt * 4) + dt) / 5;
//
//      left_speed = 25000.0 / (double)last_dt;
//    }
//  }
//}
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
void isr1A(){
  Serial.println("1A");
  static unsigned long last_time = 0;
  static long last_dt = 0;
  static unsigned char last_enc = 0;

  last_enc <<= 2; //shift previous state two places
  
  last_enc |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits
  
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
      
      right_speed = 25000.0 / (double)last_dt;
    }
  }
}
/**
 * 
 */

void isr1B(){
  Serial.println("1B");
  static unsigned long last_time = 0;
  static long last_dt = 0;
  static unsigned char last_enc = 0;
}

void isr2A(){
  Serial.println("2A");
}
void isr2B(){
  Serial.println("2B");
}/**
 * 
 */
void initWheelEncoder() {
  FastGPIO::Pin<hallPin1a>::setInputPulledUp();
  FastGPIO::Pin<hallPin1b>::setInputPulledUp();
  FastGPIO::Pin<hallPin2a>::setInputPulledUp();
  FastGPIO::Pin<hallPin2b>::setInputPulledUp();
//  pinMode(digitalPinToInterrupt(hallPin1), INPUT_PULLUP);
//  pinMode(digitalPinToInterrupt(hallPin2), INPUT_PULLUP);
#ifdef opt0
  attachInterrupt(digitalPinToInterrupt(hallPin1a), isr1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(hallPin1b), isr1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(hallPin2a), isr2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(hallPin2b), isr2B, CHANGE);
#else
  //set as inputs
  DDRD &= ~(1 << LEFT_ENC_PIN_A);
  DDRD &= ~(1 << LEFT_ENC_PIN_B);
//   DDRC &= ~(1 << RIGHT_ENC_PIN_A);
//   DDRC &= ~(1 << RIGHT_ENC_PIN_B);

  //enable pull up resistors
  PORTD |= (1 << LEFT_ENC_PIN_A);
  PORTD |= (1 << LEFT_ENC_PIN_B);
//   PORTC |= (1 << RIGHT_ENC_PIN_A);
//   PORTC |= (1 << RIGHT_ENC_PIN_B);

  // tell pin change mask to listen to left encoder pins
//*  PCMSK2 |= (1 << LEFT_ENC_PIN_A) | (1 << LEFT_ENC_PIN_B);
  PCMSK0 |= (1 << LEFT_ENC_PIN_A) | (1 << LEFT_ENC_PIN_B);
  // tell pin change mask to listen to right encoder pins
//   PCMSK1 |= (1 << RIGHT_ENC_PIN_A) | (1 << RIGHT_ENC_PIN_B);

  // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
//  PCICR |= (1 << PCIE1) | (1 << PCIE2);
//*  PCICR |= 1 << PCIE2;
  PCICR |= 1 << PCIE0;
#endif
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

