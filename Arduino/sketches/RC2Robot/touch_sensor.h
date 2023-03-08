/**
 * touch_sensor.h
 */
#ifndef touch_sensor_h
#define touch_sensor_h
#include "Arduino.h"


// To do:
// _ If touch switches form a bumper, what does it mean if only
//   one of 2 touch switches engage? Debounce may be a consideration
//   for a different layer.
//

// Forward declaration of EnableInterrupt library's enableInterrupt(...) method.
// See EnableInterrupt.h
void enableInterrupt(uint8_t interruptDesignator, void (*userFunction)(void), uint8_t mode);

// The following pin definitions can be changed.
// If changed, additional changes in the code may be required

// Supports 1, 2 or 3 touch sensors.
// Touch sensors are switches to this code

// Use 1, 2 or 3. 
#define TOUCH_SWITCH_QTY 2

#ifdef ESP32
#define TOUCH_SW_1  16
#define TOUCH_SW_2  17
#define TOUCH_SW_3  05
#else
#define TOUCH_SW_1  4
#define TOUCH_SW_2  5
#define TOUCH_SW_3  6
#endif
void initTouchSensor(int qty);

#endif //touch_sensor_h
