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

/* Define single-letter commands that will be sent
 * by the PC over the serial link.
*/

#ifndef COMMAND_H
#define COMMAND_H

extern "C" {
void initCommand();
void processCommandChar();
}

#define ANALOG_READ    'a'  // Read pin as analog value
//#define GET_BAUDRATE   'b'
#define PIN_MODE       'c'  // Get pin mode
#define DIGITAL_READ   'd'
#define READ_ENCODERS  'e'
#define MOTOR_SPEEDS   'm'
#define PING           'p'
#define RESET_ENCODERS 'r'
#define SERVO_WRITE    's'
#define SERVO_READ     't'
#define UPDATE_PID     'u'
#define DIGITAL_WRITE  'w'
#define ANALOG_WRITE   'x'
#define LEFT            0
#define RIGHT           1

#endif

