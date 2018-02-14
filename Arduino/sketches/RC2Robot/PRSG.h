/*! \file PRSG.h
 *
 * This is the main header file for the PRSG library.
 * It includes all the other header files provided by the library.
 */

//#pragma once
#include "Arduino.h"

#include "command.h"
#include "wheelencoder.h"
#include "bumper.h"

#include <stdint.h>

/*! \brief Returns true if USB power is detected.

This function returns true if power is detected on the board's USB port and
returns false otherwise.  It uses the ATmega32U4's VBUS line, which is directly
connected to the power pin of the USB connector.

\sa A method for detecting whether the board's virtual COM port is open:
  http://arduino.cc/en/Serial/IfSerial */


