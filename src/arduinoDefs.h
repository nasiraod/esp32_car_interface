#define ILLUMINATION_PIN 22
#define IGNITION_PIN 35
#define GPIO_EXPANDER0_RST_PIN 23
#define GPIO_EXPANDER0_INT_PIN 18




enum switchTypes{
	TOGGLE,
	BUTTON,
	NONE,
	OTHER
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
	EXT_IO
};


enum interrupt_sources {
	D18,
	D23
};










/*struct switchStruct{
	String nextion_name;
	int nextion_type;
	bool state;
	bool active_low;
	int pin_location;
	int address;
	int input_expander;
	int input_pin;
	int output_pin;
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
// 		frame.data16[1] = switches[incomingSwitch].output_pin;
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
// 		digitalWrite(switches[incomingSwitch].output_pin, switches[incomingSwitch].state ^ switches[incomingSwitch].active_low);
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