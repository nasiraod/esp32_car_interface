#ifndef CAN_UTILS_H
#define CAN_UTILS_H

#include <Arduino.h>
#include <ACAN_ESP32.h>
#include <DebugLog.h>

#define MAX_CAN_FRAME_DATA_LEN 8

// Function prototypes
int can_relay(int can_address, bool state, int pin);

#endif // CAN_UTILS_H
