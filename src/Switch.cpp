#include "Switch.h"
#include "CanUtils.h"

extern struct gpio_defintions gpio_expander_io_map[];
extern Adafruit_MCP23X17 io_expanders[3];

Switch::Switch(String display_name, int display_type, bool state, bool active_low, int pin_location, int address, int input_pin, int output_led, int output_pin, int physical_type, String switch_function) {
    this -> display_name = display_name;
    this -> display_type = display_type;
    this -> state = state;
    this -> active_low = active_low;
    this -> pin_location = pin_location;
    this -> address = address;
    this -> input_pin = input_pin;
    this -> output_led = output_led;
    this -> output_pin = output_pin;
    this -> physical_type = physical_type;
    this -> switch_function = switch_function;
}

bool Switch::handle_switch_event(int source) {
    int ret = 0;
    LOG_DEBUG("Entering");
    bool ok;
    switch (source) {
        case EXT_IO_LOCAL_BUTTON:
        state = !state;
        
        delay(50);
        
        break;
    }
    
    LOG_DEBUG("Switches invoked");
    switch (pin_location) {
    case GPIO_CAN:
        ret = can_relay(address, state ^ active_low, output_pin);
        break;
    case GPIO_LOCAL:
        LOG_DEBUG("GPIO_LOCAL");
        digitalWrite(output_pin, state ^ active_low);
        break;
    }
    if (ret == 0) {
        if (output_led != -1) {
            gpio_defintions temp_gpio;
            temp_gpio = gpio_expander_io_map[output_led];
            io_expanders[temp_gpio.gpio_expander].digitalWrite(temp_gpio.pin_number, state);
        }
    } else { //Failed to set output relay, revert the state variable to original state
        switch (source) {
            case EXT_IO_LOCAL_BUTTON:
            state = !state;
            delay(50);					
            break;
        }
        if (output_led != -1) {
            gpio_defintions temp_gpio;
            temp_gpio = gpio_expander_io_map[output_led];
            bool led_flash = true;
            for(int j = 9; j>=0; --j){
                io_expanders[temp_gpio.gpio_expander].digitalWrite(temp_gpio.pin_number, led_flash);
                led_flash = !led_flash;
                delay(200);
            }
            io_expanders[temp_gpio.gpio_expander].digitalWrite(temp_gpio.pin_number, state);
        }
    }
    
    // Using FreeRTOS native call instead of pthread wrapper if possible, or just standard
    LOG_DEBUG("Stack High Mark:", uxTaskGetStackHighWaterMark(NULL));
    return true;
}
