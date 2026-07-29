#ifndef SWITCH_H
#define SWITCH_H

#include <Arduino.h>
#include "arduinoDefs.h"
#include <DebugLog.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class Switch {
	public:
		String display_name;
		int display_type;
		bool state;
		bool active_low;
		int pin_location;
		int address;
		int input_pin;
		int output_led;
		int output_pin;
		int physical_type;
		String switch_function;
		Switch(String display_name, int display_type, bool state, bool active_low, int pin_location, int address, int input_pin, int output_led, int output_pin, int physical_type, String switch_function);
		bool handle_switch_event(int source);
};

#endif // SWITCH_H
