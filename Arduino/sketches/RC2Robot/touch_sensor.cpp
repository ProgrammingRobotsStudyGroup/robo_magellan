/*! \file PRSG.h
 *
 * This is the main header file for the PRSG library.
 * It includes all the other header files provided by the library.
 */

#pragma once
#include "touch_sensor.h"
#include <stdint.h>

/*! \brief Returns true if USB power is detected.

This function returns true if power is detected on the board's USB port and
returns false otherwise.  It uses the ATmega32U4's VBUS line, which is directly
connected to the power pin of the USB connector.

\sa A method for detecting whether the board's virtual COM port is open:
  http://arduino.cc/en/Serial/IfSerial */

boolean touchSw1 = false;
boolean touchSw2 = false;
boolean touchSw3 = false;

/**
 * Initialize touch switches
 * touchSwitchQty
 */
void initTouchSensor(int touchSwitchQty) {
  touchSw1 = false;
  touchSw2 = false;
  touchSw3 = false;
  pinMode(TOUCH_SW_1, INPUT_PULLUP);
  enableInterrupt(TOUCH_SW_1, touchSw1ISR, CHANGE);
  pinMode(TOUCH_SW_2, INPUT_PULLUP);
  enableInterrupt(TOUCH_SW_2, touchSw2ISR, CHANGE);
  pinMode(TOUCH_SW_3, INPUT_PULLUP);
  enableInterrupt(TOUCH_SW_3, touchSw3ISR, CHANGE);

  Serial.print("Initialized touch sensors. Number of switches: ");
  Serial.println(touchSwitchQty);
}

static exampleSwitch(int touchSwitchQty) {
  switch (touchSwitchQty) {
    case 1: // Center Only
      break;
    case 3: // Center, Left and Right
      pinMode(TOUCH_SW_3, INPUT_PULLUP);
      enableInterrupt(TOUCH_SW_3, touchSw3ISR, CHANGE);
    case 2: // Left and Right
    default:
    case 0:
      break;
  }
}

/**
 * 
 */
static void touchSw1ISR() {
  Serial.println("Sw1");
}

/**
 * 
 */
static void touchSw2ISR() {
  Serial.println("Sw2");
}

/**
 * 
 */
static void touchSw3ISR() {
  Serial.println("Sw3");
}


