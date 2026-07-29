#include <Arduino.h>
#include "arduinoDefs.h"
#include <Adafruit_MCP23X17.h>
#include "Switch.h"

// Initialize GPIO Expander Hardware Parameters
struct gpio_expander_param gpio_expander_hw[] = {
    {12, 23, 0x21, 0, true},
    {13, 25, 0x22, 16, true}
};

// Initialize LED Indices
uint8_t led_indecies[] = {10, 11, 12, 13, 14, 15, 26, 27, 28, 29};

// Initialize GPIO Expander IO Map
struct gpio_defintions gpio_expander_io_map[] = {
    {0, "GPA0", "EXT_IN_GPIO1",     0,      INPUT,              false, false, -1},
    {0, "GPA1", "EXT_IN_GPIO2",     1,      INPUT,              false, false, -1},
    {0, "GPA2", "EXT_IN_GPIO3",     2,      INPUT,              false, false, -1},
    {0, "GPA3", "EXT_IN_GPIO4",     3,      INPUT,              false, false, -1},
    {0, "GPA4", "EXT_IN_GPIO5",     4,      INPUT,              false, false, -1},
    {0, "GPA5", "EXT_IN_GPIO6",     5,      INPUT,              false, false, -1},
    {0, "GPA6", "EXT_IN_GPIO7",     6,      INPUT,              false, false, -1},
    {0, "GPA7", "EXT_IN_GPIO8",     7,      INPUT,              false, false, -1},
    {0, "GPB0", "EXT_IN_GPIO9",     8,      INPUT,              false, false, -1},
    {0, "GPB1", "EXT_IN_GPIO10",    9,      INPUT,              false, false, -1},
    {0, "GPB2", "LED_GPIO_SW1",     10,     OUTPUT,             false, false, -1},
    {0, "GPB3", "LED_GPIO_SW2",     11,     OUTPUT,             false, false, -1},
    {0, "GPB4", "LED_GPIO_SW3",     12,     OUTPUT,             false, false, -1},
    {0, "GPB5", "LED_GPIO_SW4",     13,     OUTPUT,             false, false, -1},
    {0, "GPB6", "LED_GPIO_SW5",     14,     OUTPUT,             false, false, -1},
    {0, "GPB7", "LED_GPIO_SW6",     15,     OUTPUT,             false, false, -1},
    {1, "GPA0", "EXT_SW_GPIO1",     0,      INPUT,              true, false, 0},
    {1, "GPA1", "EXT_SW_GPIO2",     1,      INPUT,              true, false, 1},
    {1, "GPA2", "EXT_SW_GPIO3",     2,      INPUT,              true, false, 2},
    {1, "GPA3", "EXT_SW_GPIO4",     3,      INPUT,              true, false, 3},
    {1, "GPA4", "EXT_SW_GPIO5",     4,      INPUT,              true, false, 4},
    {1, "GPA5", "EXT_SW_GPIO6",     5,      INPUT,              true, false, 5},
    {1, "GPA6", "EXT_SW_GPIO7",     6,      INPUT,              true, false, -1},
    {1, "GPA7", "EXT_SW_GPIO8",     7,      INPUT,              true, false, -1},
    {1, "GPB0", "EXT_SW_GPIO9",     8,      INPUT,              true, false, -1},
    {1, "GPB1", "EXT_SW_GPIO10",    9,      INPUT,              true, false, -1},
    {1, "GPB2", "LED_GPIO_SW7",     10,     OUTPUT,             false, false, -1},
    {1, "GPB3", "LED_GPIO_SW8",     11,     OUTPUT,             false, false, -1},
    {1, "GPB4", "LED_GPIO_SW9",     12,     OUTPUT,             false, false, -1},
    {1, "GPB5", "LED_GPIO_SW10",    13,     OUTPUT,             false, false, -1},
    {1, "GPB6", "NA",               14,     OUTPUT,             false, false, -1},
    {1, "GPB7", "NA",               15,     OUTPUT,             false, false, -1}
};

// Global GPIO Expander Objects
Adafruit_MCP23X17 io_expanders[3];

// Global Switches
Switch switches[] = {
	Switch( "P1_SW0", TOGGLE, true, false, GPIO_CAN, 0x7FF, 0, 10, 7, BUTTON, "Roof Lights"),					// Roof Toggle SW
	Switch( "P1_SW1", TOGGLE, false, false, GPIO_CAN, 0x7FF, 1, 11, 6, BUTTON, "Bumper Lights"),					// Bumper Toggle SW
	Switch( "P1_SW2", TOGGLE, false, false, GPIO_CAN, 0x7FF, 2, 12, 5, BUTTON, "Ditch Lights"),					// Ditch Toggle SW
	Switch( "P1_SW3", TOGGLE, false, false, GPIO_CAN, 0x7FF, 3, 13, 4, BUTTON, "Rear Lights"),					// Rear Toggle SW
	Switch( "P1_SW4", TOGGLE, false, false, GPIO_CAN, 0x7FF, 4, 14, 3, BUTTON, "Rock Lights"),					// Rock Toggle SW
	Switch( "P1_SW5", TOGGLE, false, false, GPIO_CAN, 0x7FF, 5, 15, 2, BUTTON, "Rock Lights")					// Rock Toggle SW
};

const int num_switches = sizeof(switches) / sizeof(Switch);
const int num_gpio_map_size = sizeof(gpio_expander_io_map) / sizeof(gpio_defintions);
const int num_active_gpio_expanders = sizeof(gpio_expander_hw) / sizeof(gpio_expander_param);
const int num_led_indecies = sizeof(led_indecies) / sizeof(led_indecies[0]);
