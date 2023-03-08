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

#ifndef ESP32
  #if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
  #else
    #include "WProgram.h"
  #endif
#endif

#include "command.h"
#include "wheelencoder.h"

#define SERIAL_STREAM Serial
#define DEBUG_SERIAL_STREAM Serial


#define CHAR_CR 0x0d
#define CHAR_SP ' '
// String definitions
// Note: 
//  Use definition instead of string literal to ensure consistency.
//  Uses the same space as compiler merges strings. const uses MORE memory
#define STR_OK "OK"


// Forward declarations
int executeCommand(int cmd, int arg1, int arg2);

/**
 * Global command processing variables
 */
char cmd;                 // command
char argv1[16];           // First and second arguments
char argv2[16];
int arg = 0;
int _index = 0;

/**
 * Read a character from a serial connection
 */
void processCommandChar() {
  static char chr;          // Input character

  // If we have a character...
  while (SERIAL_STREAM.available() > 0) {
    // Read the next character
    chr = SERIAL_STREAM.read();

    // Terminate a command with a CR
    if (chr == CHAR_CR) {
      if (arg == 1) {
        argv1[_index] = NULL;
      }
      else if (arg == 2) {
        argv2[_index] = NULL;
      }
      executeCommand(cmd, atoi(argv1), atoi(argv2));
      initCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == CHAR_SP) {
      switch (arg) {
        case 1:
          argv1[_index] = NULL;
          _index = 0;
          // Fall through
        case 0:
          arg+=1;
      }
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[_index] = chr;
        _index++;
      }
      else if (arg == 2) {
        argv2[_index] = chr;
        _index++;
      }
    }
  }
}

/**
 * Clear the current command parameters
 */
void initCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg = 0;
  _index = 0;
}

/**
 * Execute command from an external communication channel.
 * Return: 1 = Command processed;
 *         0 = Bad command
 */
int executeCommand(int cmd, int arg1, int arg2) {
  int retVal = 1;

  switch(cmd) {
  case ANALOG_READ:
    SERIAL_STREAM.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    SERIAL_STREAM.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    SERIAL_STREAM.println(STR_OK);
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) {
      digitalWrite(arg1, LOW);
    }
    else if (arg2 == 1) {
      digitalWrite(arg1, HIGH);
    }
    SERIAL_STREAM.println(STR_OK);
    break;
  case PIN_MODE:
    if (arg2 == 0)
      pinMode(arg1, INPUT);
    else if (arg2 == 1)
      pinMode(arg1, OUTPUT);
    SERIAL_STREAM.println(STR_OK);
    break;
//  case PING:
//    SERIAL_STREAM.println(Ping(arg1));
//    break;

//#ifdef USE_BASE
//  case READ_ENCODERS:
//    SERIAL_STREAM.print(readEncoder(LEFT));
//    SERIAL_STREAM.print(" ");
//    SERIAL_STREAM.println(readEncoder(RIGHT));
//    break;
   case RESET_ENCODERS:
    resetEncoders();
//    resetPID();
    SERIAL_STREAM.println("Reset encoders");
    break;
//  case MOTOR_SPEEDS:
//    /* Reset the auto stop timer */
//    lastMotorCommand = millis();
//    if (arg1 == 0 && arg2 == 0) {
//      setMotorSpeeds(0, 0);
//      moving = 0;
//    }
//    else moving = 1;
//    leftPID.TargetTicksPerFrame = arg1;
//    rightPID.TargetTicksPerFrame = arg2;
//    SERIAL_STREAM.println(STR_OK);
//    break;
//  case UPDATE_PID:
//	char *str;
//	int pid_args[4];
//    while ((str = strtok_r(p, ":", &p)) != '\0') {
//       pid_args[i] = atoi(str);
//       i++;
//    }
//    Kp = pid_args[0];
//    Kd = pid_args[1];
//    Ki = pid_args[2];
//    Ko = pid_args[3];
//    SERIAL_STREAM.println(STR_OK);
//    break;
//#endif

  // 0=LEFT_ENCODER; 1=RIGHT_ENCODER. See enum.
  case VELOCITY:
    double vel;
    vel = readTicks(arg2);
    SERIAL_STREAM.println(vel);
    break;

  // Bad command=Unrecognized
  default:
    SERIAL_STREAM.println("Bad command");
    retVal = 0;
    break;
  }
  return retVal;
}
