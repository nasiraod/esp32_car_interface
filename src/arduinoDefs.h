#include <Arduino.h>
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
struct gpio_expander_param gpio_expander_hw[] {
    {12, 23, 0x21, 0, true},
    {13, 25, 0x22, 16, true}
};
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
    {1, "GPA1", "EXT_SW_GPIO2",     1,      INPUT,              false, false, 1},
    {1, "GPA2", "EXT_SW_GPIO3",     2,      INPUT,              false, false, 2},
    {1, "GPA3", "EXT_SW_GPIO4",     3,      INPUT,              false, false, 3},
    {1, "GPA4", "EXT_SW_GPIO5",     4,      INPUT,              false, false, 4},
    {1, "GPA5", "EXT_SW_GPIO6",     5,      INPUT,              false, false, -1},
    {1, "GPA6", "EXT_SW_GPIO7",     6,      INPUT,              false, false, -1},
    {1, "GPA7", "EXT_SW_GPIO8",     7,      INPUT,              false, false, -1},
    {1, "GPB0", "EXT_SW_GPIO9",     8,      INPUT,              false, false, -1},
    {1, "GPB1", "EXT_SW_GPIO10",    9,      INPUT,              false, false, -1},
    {1, "GPB2", "LED_GPIO_SW7",     10,     OUTPUT,             false, false, -1},
    {1, "GPB3", "LED_GPIO_SW8",     11,     OUTPUT,             false, false, -1},
    {1, "GPB4", "LED_GPIO_SW9",     12,     OUTPUT,             false, false, -1},
    {1, "GPB5", "LED_GPIO_SW10",    13,     OUTPUT,             false, false, -1},
    {1, "GPB6", "NA",               14,     OUTPUT,             false, false, -1},
    {1, "GPB7", "NA",               15,     OUTPUT,             false, false, -1}//,
    // {2, "GPA0", "EXT_OUT_NEG1",     0,      OUTPUT, false, false, -1},
    // {2, "GPA1", "EXT_OUT_NEG2",     1,      OUTPUT, false, false, -1},
    // {2, "GPA2", "EXT_OUT_NEG3",     2,      OUTPUT, false, false, -1},
    // {2, "GPA3", "EXT_OUT_NEG4",     3,      OUTPUT, false, false, -1},
    // {2, "GPA4", "EXT_OUT_NEG5",     4,      OUTPUT, false, false, -1},
    // {2, "GPA5", "EXT_OUT_NEG6",     5,      OUTPUT, false, false, -1},
    // {2, "GPA6", "EXT_OUT_NEG7",     6,      OUTPUT, false, false, -1},
    // {2, "GPA7", "EXT_OUT_NEG8",     7,      OUTPUT, false, false, -1},
    // {2, "GPB0", "OUT_POS_GPIO1",    8,      OUTPUT, false, false, -1},
    // {2, "GPB1", "OUT_POS_GPIO2",    9,      OUTPUT, false, false, -1},
    // {2, "GPB2", "OUT_POS_GPIO3",    10,     OUTPUT, false, false, -1},
    // {2, "GPB3", "OUT_POS_GPIO4",    11,     OUTPUT, false, false, -1},
    // {2, "GPB4", "OUT_POS_GPIO5",    12,     OUTPUT, false, false, -1},
    // {2, "GPB5", "OUT_POS_GPIO6",    13,     OUTPUT, false, false, -1},
    // {2, "GPB6", "OUT_POS_GPIO7",    14,     OUTPUT, false, false, -1},
    // {2, "GPB7", "OUT_POS_GPIO8",    15,     OUTPUT, false, false, -1}
};




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










/*struct switchStruct{
	String nextion_name;
	int nextion_type;
	bool state;
	bool active_low;
	int pin_location;
	int address;
	int INPUT, false, false_expander;
	int INPUT, false, false_pin;
	int OUTPUT, false, false_pin;
	int physical_type;
};*/

/*struct switchStruct switches[] = {
		{ "P1_SW0", TOGGLE, true, true, GPIO_CAN, 0x7FF, 0, 0, 0, BUTTON },						// Roof Toggle SW
		{ "P1_SW1", TOGGLE, false, true, GPIO_CAN, 0x7FF, 0, 1, 1, BUTTON },					// Bumper Toggle SW
		{ "P1_SW2", TOGGLE, false, true, GPIO_CAN, 0x7FF, 0, 2, 2, BUTTON },					// Ditch Toggle SW
		{ "P1_SW3", TOGGLE, false, true, GPIO_CAN, 0x7FF, 0, 3, 3, BUTTON },					// Rear Toggle SW
		{ "P1_SW4", TOGGLE, false, true, GPIO_CAN, 0x7FF, 0, 4, 4, BUTTON }						// Rock Toggle SW
};*/

/* Following is how the CAN packet is sent in sequence, please refer to CAN_PACKET.png for the scope capture
frame.data[0] = 0x01;
frame.data[1] = 0x23;
frame.data[2] = 0x45;
frame.data[3] = 0x67;
frame.data[4] = 0x89;
frame.data[5] = 0xAB;
frame.data[6] = 0xCD;
frame.data[7] = 0xEF;
frame.data16[0] = 0x2301;
frame.data16[1] = 0x6745;
frame.data16[2] = 0xAB89;
frame.data16[3] = 0xEFCD;
frame.data32[0] = 0x67452301;
frame.data32[1] = 0xEFCDAB89;
frame.data64 = 0xEFCDAB8967452301;*/
// void handleSwitchEvent (int incomingSwitch, int triggerSource) {
// 	CANMessage frame;
// 	bool ok;
// 	frame.id= 0x7FF;
// 	frame.len = 8;
// 	//ACAN_ESP32::can.tryToSend (frame) ;
	
// 	LOG_DEBUG("Switches invoked");
// 	switch (switches[incomingSwitch].pin_location) {
// 	case GPIO_CAN:
// 		frame.id= switches[incomingSwitch].address;
// 		frame.len = MAX_CAN_FRAME_DATA_LEN;
// 		/* Following is how the CAN packet is sent in sequence, please refer to CAN_PACKET.png for the scope capture
// 		frame.data[0] = 0x01;
// 		frame.data[1] = 0x23;
// 		frame.data[2] = 0x45;
// 		frame.data[3] = 0x67;
// 		frame.data[4] = 0x89;
// 		frame.data[5] = 0xAB;
// 		frame.data[6] = 0xCD;
// 		frame.data[7] = 0xEF;
// 		frame.data16[0] = 0x2301;
// 		frame.data16[1] = 0x6745;
// 		frame.data16[2] = 0xAB89;
// 		frame.data16[3] = 0xEFCD;
// 		frame.data32[0] = 0x67452301;
// 		frame.data32[1] = 0xEFCDAB89;
// 		frame.data64 = 0xEFCDAB8967452301;*/

// 		frame.data16[0] = switches[incomingSwitch].state ^ switches[incomingSwitch].active_low;
// 		frame.data16[1] = switches[incomingSwitch].OUTPUT, false, false_pin;
// 		frame.data16[2] = 0x0000;
// 		frame.data16[3] = 0x0000;
		
		


// 		/*CAN_FRAME commandOut;
// 		/*commandOut.id = 0x7FF;
// 		commandOut.length = MAX_CAN_FRAME_DATA_LEN;
// 		commandOut.data.s0 = 0x0123;
// 		commandOut.data.s1 = 0x4567;
// 		commandOut.data.s2 = 0x89AB;
// 		commandOut.data.s3 = 0xCDEF;
// 		commandOut.extended = 0;*/
// 		//Can0.sendFrame(commandOut);*/
// 		ok = ACAN_ESP32::can.tryToSend (frame) ;
// 		if(ok) {
// 			// char temp[] = "";
// 			// sprintf(temp, "CAN MESSAGE SENT: %x", frame.id);
// 			LOG_DEBUG("CAN MESSAGE SENT");
// 			// LOG_DEBUG(temp);

// 		}
		
// 		break;
// 	case GPIO_LOCAL:
// 		digitalWrite(switches[incomingSwitch].OUTPUT, false, false_pin, switches[incomingSwitch].state ^ switches[incomingSwitch].active_low);
// 		break;
// 	}
// }


// void raspSerialListenerSCH() {
// 	// read from port 1, send to port 0:
// 	if (Serial1.available()) {
// 		//String inByte = Serial1.readString();
// 		int numBytes = Serial1.readBytesUntil(TERMINATOR, inByte, BUFFER_SIZE);
// 		//Serial.println(inByte);
// 		//Serial.write(inByte);
// 		//char * inByteCh = inByte;
// 		raspPacketParser(numBytes);

// 	}
// 	// read from port 0, send to port 1:
// 	if (Serial.available()) {
// 		int inByte = Serial.read();
// 		Serial1.write(inByte);
// 	}
// 	yield();
// }
// //Data Packet: <HEADER_TYPE><COMMAND><DATA_TYPE><ELEMENT><NUMBER_DATA_BYTES><DATA><TERMINATOR>

// void raspPacketParser(int numBytes) {
// 	switch (inByte[0]) {
// 	case CONFIG:
// 		Serial.write("Config command received!\n");
// 		if (inByte[1] == GET) {
// 			Serial.write("Config get command received!\n");
// 		}
// 		else if (inByte[1] == SET) {
// 			Serial.write("Config set command received!\n");
// 		}
// 		else
// 			Serial.write("Invalid command!\n");

// 		break;
// 	case STATE:
// 		Serial.write("State command received!\n");
// 		break;
// 	default:
// 		// block of code default: do something when var is not equal to any of above label
// 		break;
// 	}
// 	//Done Parsing, empty buffer
// 	memset(inByte, 0, BUFFER_SIZE);
// }