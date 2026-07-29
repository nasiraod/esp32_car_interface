#ifndef ARDUINO_DEFS_H
#define ARDUINO_DEFS_H

#include <Arduino.h>
#include <Adafruit_MCP23X17.h>

// #define ILLUMINATION_PIN 22
#define IGNITION_PIN 19
#define GPIO_EXPANDER0_RST_PIN 13
#define GPIO_EXPANDER0_INT_PIN 25
#define GPIO_EXPANDER0_ADDRESS 0x22
#define GPIO_EXPANDER0_BASE_ADDRESS 0
#define GPIO_EXPANDER1_BASE_ADDRESS 16
#define GPIO_EXPANDER2_BASE_ADDRESS 32
#define GPIO_I2C_SDA 32
#define GPIO_I2C_SCL 33

struct gpio_expander_param {
    uint8_t RST_PIN;
    uint8_t INT_PIN;
    uint8_t I2C_ADDRESS;
    uint8_t BASE_INDEX;
    bool INTERRUPTABLE;
};

// External declaration
extern struct gpio_expander_param gpio_expander_hw[];

struct gpio_defintions {
    uint8_t gpio_expander;
    char pin_name[5];
    char pin_function[20];
    uint8_t pin_number;
    int pin_direction;
    bool interrupt_pin;
    bool interrupt_high;
    int switch_index;     
};

// External declarations
extern uint8_t led_indecies[];
extern struct gpio_defintions gpio_expander_io_map[];
extern Adafruit_MCP23X17 io_expanders[3];

class Switch; // Forward declaration
extern Switch switches[]; // Extern declaration
extern const int num_switches;
extern const int num_gpio_map_size;
extern const int num_active_gpio_expanders;
extern const int num_led_indecies;

enum switchTypes{
	TOGGLE,
	BUTTON,
	NONE,
	OTHER
};
enum ilumination_states{
	NIGHT,
	MEDIUM,
	DAY
};
enum deviceLocation{
	GPIO_LOCAL,
	GPIO_CAN,
	GPIO_EXP_LOCAL,
	TTL_RELAY_LOCAL,
	TTL_RELAY_CAN
};

enum switchSource{
	GUI_DISPLAY,
    CAN_IO,
	EXT_IO,
    EXT_IO_LOCAL_BUTTON,
    BLUETOOTH_DEVICE
};

enum interrupt_sources {
	GPIO_EXPANDER,
    GPIO_EXPANDER0,
    GPIO_EXPANDER1,
    GPIO_EXPANDER2
};
enum ignition_states{
	IGNITION_INIT,
    IGNITION_OFF,
	IGNITION_ON
};

#endif // ARDUINO_DEFS_H