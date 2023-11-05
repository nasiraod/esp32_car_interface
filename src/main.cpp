#include <Arduino.h>
//#include "variant.h"
//#include <Scheduler.h>
#include "dispDefsNXT.h"
#include "dataPacket.h"
#include "arduinoDefs.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <EasyNextionLibrary.h>
#include <hp_BH1750.h>
#define DEBUGLOG_DEFAULT_LOG_LEVEL_DEBUG
#define LOG_ATTACH_SERIAL(Serial)
#include <DebugLog.h>
// #include <TaskScheduler.h>
#include <Adafruit_MCP23X17.h>
//#include <due_can.h>
#define MAX_CAN_FRAME_DATA_LEN   8
#include <ACAN_ESP32.h>
#include <core_version.h>
#include "soc/rtc_wdt.h"
#include <pthread.h>
#include <esp_sleep.h>

#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;

// #include <BLEDevice.h>
// #include <BLEUtils.h>
// #include <BLEServer.h>

// // See the following for generating UUIDs:
// // https://www.uuidgenerator.net/

// #define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
// #define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"


////////////////////////////////
///////Light Sensor Object//////
////////////////////////////////
hp_BH1750 lightSensor;
////////////////////////////////

////////////////////////////////
/////GPIO Expander Objects//////
////////////////////////////////
Adafruit_MCP23X17 io_expanders[3];
////////////////////////////////


////////////////////////////////
/////Nextion Display Pages//////
////////////////////////////////
bool halt = false;
EasyNex interfaceScreen(Serial2);
////////////////////////////////

////////////////////////////////
///////Gyroscope Object/////////
////////////////////////////////
Adafruit_MPU6050 gyroSensor;
struct sensorData {
	long gyroRoll;
	long gyroPitch;
	String gyroTemp;
	String lightLux;
	float voltage;
};
////////////////////////////////
struct test_message {
	char message_id[2];
	char message[20];
};
QueueHandle_t test_queue;
////////////////////////////////
///////////Switches/////////////
////////////////////////////////
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
		Switch(String display_name, int display_type, bool state, bool active_low, int pin_location, int address, int input_pin, int output_led, int output_pin, int physical_type, String switch_function) {
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
		bool handle_switch_event(int source) {
			CANMessage frame;
			bool ok;
			switch (source) {
				case EXT_IO_LOCAL_BUTTON:
				state = !state;
				
				delay(50);
				
				while (interfaceScreen.readNumber(String(display_name + ".val")) != (int) state) {
					interfaceScreen.writeNum(String(display_name + ".val"), (int)state);
					LOG_DEBUG("Screen Current Value: ", interfaceScreen.readNumber(String(display_name + ".val")));
				}
				
				break;
			}
			if (output_led != -1) {
				gpio_defintions temp_gpio;
				temp_gpio = gpio_expander_io_map[output_led];
				io_expanders[temp_gpio.gpio_expander].digitalWrite(temp_gpio.pin_number, state);
			}
			
			LOG_DEBUG("Switches invoked");
			switch (pin_location) {
			case GPIO_CAN:
				
				frame.id= address;
				frame.len = MAX_CAN_FRAME_DATA_LEN;
				frame.data16[0] = state ^ active_low;
				frame.data16[1] = output_pin;
				frame.data16[2] = 0x0000;
				frame.data16[3] = 0x0000;
				ok = ACAN_ESP32::can.tryToSend (frame) ;
				if(ok) {
					LOG_DEBUG("CAN MESSAGE SENT: ", ok);
				}
				
				break;
			case GPIO_LOCAL:
				LOG_DEBUG("GPIO_LOCAL");
				digitalWrite(output_pin, state ^ active_low);
				break;
			}
			LOG_DEBUG("Stack High Mark:", uxTaskGetStackHighWaterMark(NULL));
			return true;
		}

};

Switch switches[] = {
	Switch( "P1_SW0", TOGGLE, true, false, GPIO_CAN, 0x7FF, 0, 10, 7, BUTTON, "Roof Lights"),					// Roof Toggle SW
	Switch( "P1_SW1", TOGGLE, false, false, GPIO_CAN, 0x7FF, 1, 11, 6, BUTTON, "Bumper Lights"),					// Bumper Toggle SW
	Switch( "P1_SW2", TOGGLE, false, false, GPIO_CAN, 0x7FF, 2, 12, 5, BUTTON, "Ditch Lights"),					// Ditch Toggle SW
	Switch( "P1_SW3", TOGGLE, false, false, GPIO_CAN, 0x7FF, 3, 13, 4, BUTTON, "Rear Lights"),					// Rear Toggle SW
	Switch( "P1_SW4", TOGGLE, false, false, GPIO_CAN, 0x7FF, 4, 14, 3, BUTTON, "Rock Lights")					// Rock Toggle SW
};





bool interrupt_trigger = false;
interrupt_sources interrupt_source;
int gpio_pin;
int gpio_expander_number;
int8_t screen_on_off = -1;

////////////////////////////////
///////Function Definitions/////
////////////////////////////////
void led_sequence();
void sensors_init();
// void handleSwitchEvent (int incomingSwitch, int triggerSource);
// void raspSerialListenerSCH();
// void getGyroData();
void indexSWError();
float mapF(float x, float in_min, float in_max, float out_min, float out_max);
void IRAM_ATTR isr_gpio_expander0();
void IRAM_ATTR isr_gpio_expander1();
void IRAM_ATTR isr_gpio_expander2();
////////////////////////////////

////////////////////////////////
////////Global Variables////////
////////////////////////////////
struct sensorData currentSensorData;
// const int BUFFER_SIZE = 128;
// char inByte[BUFFER_SIZE];
int ilumination_limits[] = {300, 500};
uint8_t ilumination_hysteresis = 25;
ilumination_states ilumination_current_state = NIGHT;
ilumination_states ilumination_previous_state = NIGHT;
ignition_states ignition_current_state = IGNITION_INIT;
ignition_states ignition_previous_state = IGNITION_INIT;
int NXTpage = 1;
//  CAN ESP32 Desired Bit Rate
// static const uint32_t DESIRED_BIT_RATE = 1000UL * 250UL ; // 250 Kb/s

////////////////////////////////

////////////////////////////////
////////////Tasks///////////////
////////////////////////////////
void *nextion_screen_thread(void *threadid);
void *sensor_acquisition_thread(void *threadid);
void *bluetooth_handler_thread(void *threadid);
pthread_t nxt_thread;
pthread_t sensors_thread;
// pthread_t interrupt_thread;
pthread_t bluetooth_thread;



// Scheduler runnerSCH;
void bluetooth_init() {
	SerialBT.begin("Car Interface");
	LOG_DEBUG("Bluetooth Started! Ready to pair...");
}
// void go_to_sleep() {

// }
void io_init() {
	LOG_DEBUG("Initializing IOs");
	int gpio_map_size = sizeof(gpio_expander_io_map) / sizeof(gpio_defintions);
	int active_gpio_expanders = sizeof(gpio_expander_hw) / sizeof(gpio_expander_param);
	LOG_DEBUG("Active GPIO Expanders: ", active_gpio_expanders);
	for (int i = 0; i < active_gpio_expanders; i++) {
		LOG_DEBUG("Resetting GPIO Expander ", i);
		pinMode(gpio_expander_hw[i].RST_PIN, OUTPUT);
		digitalWrite(gpio_expander_hw[i].RST_PIN, LOW);
		delay(50);
		digitalWrite(gpio_expander_hw[i].RST_PIN, HIGH);
		int count = 0;
		LOG_DEBUG("Initializing GPIO Expander ", i);
		while (true) {
			count++;
			if (count >= 10 || io_expanders[i].begin_I2C(gpio_expander_hw[i].I2C_ADDRESS, &Wire1)) break;
			LOG_ERROR("GPIO Expander: Reinitializing");
			delay(100);

		}
		if (count >= 10) {
			LOG_ERROR("GPIO Expander: Error Initializing");
			LOG_ERROR("Rebooting Device");
			ESP.deepSleep(1000000);
			ESP.restart();
		}
		if (gpio_expander_hw[i].INTERRUPTABLE) io_expanders[i].setupInterrupts(true, false, LOW);
	}
	
	LOG_DEBUG("Found ", gpio_map_size, "IOs in map");
	
	for (int i = 0; i < gpio_map_size; i++) {
		String temp_log_message = "";
		temp_log_message.concat("GPIO Expander ");
		temp_log_message.concat(gpio_expander_io_map[i].gpio_expander);
		temp_log_message.concat(": ");
		temp_log_message.concat(gpio_expander_io_map[i].pin_name);
		temp_log_message.concat(": ");
		temp_log_message.concat(gpio_expander_io_map[i].pin_function);
		temp_log_message.concat(" - Direction: ");
		switch (gpio_expander_io_map[i].pin_direction) {
			case INPUT:
				temp_log_message.concat(" INPUT");
			break;
			case OUTPUT:
				temp_log_message.concat(" OUTPUT");
			break;
			case PULLUP:
				temp_log_message.concat(" PULLUP");
			break;
			case INPUT_PULLUP:
				temp_log_message.concat(" INPUT_PULLUP");
			break;
			case PULLDOWN:
				temp_log_message.concat(" PULLDOWN");
			break;
			case INPUT_PULLDOWN:
				temp_log_message.concat(" INPUT_PULLDOWN");
			break;
			case OPEN_DRAIN:
				temp_log_message.concat(" OPEN_DRAIN");
			break;
			case OUTPUT_OPEN_DRAIN:
				temp_log_message.concat(" OUTPUT_OPEN_DRAIN");
			break;
			case ANALOG:
				temp_log_message.concat(" ANALOG");
			break;
			default:
				temp_log_message.concat(" N/A");
			break;

		}
		io_expanders[gpio_expander_io_map[i].gpio_expander].pinMode(gpio_expander_io_map[i].pin_number, gpio_expander_io_map[i].pin_direction);
		if (gpio_expander_io_map[i].interrupt_pin) {
			temp_log_message.concat(" - Interrupt Pin: ");
			if (gpio_expander_io_map[i].interrupt_high) { 
				temp_log_message.concat("HIGH");
				io_expanders[gpio_expander_io_map[i].gpio_expander].setupInterruptPin(gpio_expander_io_map[i].pin_number, HIGH);
			}
			else { 
				temp_log_message.concat("LOW");
				io_expanders[gpio_expander_io_map[i].gpio_expander].setupInterruptPin(gpio_expander_io_map[i].pin_number, LOW);
			}
			io_expanders[gpio_expander_io_map[i].gpio_expander].clearInterrupts();
		}

		// LOG_DEBUG("GPOIO Expander", gpio_expander_io_map[i].gpio_expander, ":", gpio_expander_io_map[i].pin_name, ":", gpio_expander_io_map[i].pin_function, gpio_expander_io_map[i].pin_direction);
		LOG_DEBUG(temp_log_message);
	}
	
	pinMode(gpio_expander_hw[0].INT_PIN, INPUT_PULLUP);
	attachInterrupt(gpio_expander_hw[0].INT_PIN, isr_gpio_expander0, FALLING);

	pinMode(gpio_expander_hw[1].INT_PIN, INPUT_PULLUP);
	attachInterrupt(gpio_expander_hw[1].INT_PIN, isr_gpio_expander1, FALLING);

	// pinMode(gpio_expander_hw[2].INT_PIN, INPUT_PULLUP);
	// attachInterrupt(gpio_expander_hw[2].INT_PIN, isr_gpio_expander2, FALLING);

	// pinMode(ILLUMINATION_PIN, INPUT_PULLUP);
	pinMode(IGNITION_PIN, INPUT_PULLUP);

}
void setup() {
	test_queue = xQueueCreate( 10, sizeof(struct test_message) );
	// initialize both serial ports:
	
	Serial.begin(115200);
	Wire1.begin(GPIO_I2C_SDA, GPIO_I2C_SCL);
	// Serial1.begin(9600);
	interfaceScreen.begin(9600);
	Serial.println("Car Interface");
	bluetooth_init();
	
	
	
	sensors_init();
	while (interfaceScreen.readNumber("dp") != 0) {
		interfaceScreen.writeNum("sleep", 0);
		interfaceScreen.writeStr("page 0");
	}
	pthread_attr_t attr;
	size_t stacksize;
	pthread_attr_init(&attr);
	pthread_attr_getstacksize(&attr, &stacksize);
	LOG_DEBUG("Max pthread stack size Before:", stacksize);
	stacksize = stacksize + 1024;
	// stacksize = stacksize * 2;
	pthread_attr_setstacksize(&attr, stacksize);

	pthread_attr_getstacksize(&attr, &stacksize);
	LOG_DEBUG("Max pthread stack size After:", stacksize);
	
	delay(50);
	io_init();
	delay(50);
	pthread_create(&nxt_thread, &attr, nextion_screen_thread, (void *) 0);
	delay(50);
	pthread_create(&sensors_thread, &attr, sensor_acquisition_thread, (void *) 1);
	delay(50);
	pthread_create(&bluetooth_thread, &attr, bluetooth_handler_thread, (void *) 2);
	
	
	
	LOG_DEBUG("Configure ESP32 CAN");
	ACAN_ESP32_Settings settings (250 * 1000) ;
	//settings.mRxPin = GPIO_NUM_4 ; // Optional, default Tx pin is GPIO_NUM_4
	//settings.mTxPin = GPIO_NUM_5 ; // Optional, default Rx pin is GPIO_NUM_5
	// TODO: Change CAN to Normal Mode for transmission error handling, would require changing the switch event class method
	// settings.mRequestedCANMode = ACAN_ESP32_Settings::LoopBackMode ;
	settings.mRequestedCANMode = ACAN_ESP32_Settings::NormalMode ;

	const uint32_t errorCode = ACAN_ESP32::can.begin (settings) ;
	
	if (errorCode == 0) {
		LOG_DEBUG("Configuration ESP32 OK!");
	}else {
		LOG_ERROR("Configuration error");
		LOG_ERROR(errorCode, HEX);
	}
	delay(100);
	// interfaceScreen.NextionListen();
	// int current_page = interfaceScreen.currentPageId;
	// LOG_DEBUG("Current Page: ", current_page);
	
	led_sequence();


}
struct test_message recv_message;
int gpio_expander_index = -1;
char test_char[20];
void loop() {
	recv_message.message[20] = {NULL};
	if (uxQueueMessagesWaiting(test_queue) > 0) {
		xQueueReceive(test_queue, &(recv_message), (TickType_t) 10);
		strcpy(test_char, recv_message.message);
		LOG_DEBUG("Message Receieved:", test_char);
		strcpy(test_char, "");
	}
	if (currentSensorData.lightLux.toFloat() <= ilumination_limits[0] - ilumination_hysteresis) {
		ilumination_current_state = NIGHT;
	}
	if (currentSensorData.lightLux.toFloat() >= ilumination_limits[0] + ilumination_hysteresis && currentSensorData.lightLux.toFloat() <= ilumination_limits[1] - ilumination_hysteresis) {
		ilumination_current_state = MEDIUM;
	}
	if (currentSensorData.lightLux.toFloat() >= ilumination_limits[1] + ilumination_hysteresis) {
		ilumination_current_state = DAY;
	}
	if (ilumination_current_state != ilumination_previous_state) {
		switch(ilumination_current_state) {
			case NIGHT:
			LOG_DEBUG("Change to night mode");
			ilumination_previous_state = NIGHT;
			break;
			case MEDIUM:
			LOG_DEBUG("Change to mid mode");
			ilumination_previous_state = MEDIUM;
			break;
			case DAY:
			LOG_DEBUG("Change to day mode");
			ilumination_previous_state = DAY;
			break;

		}
	}

	if (digitalRead(IGNITION_PIN)) ignition_current_state = IGNITION_OFF;
	else ignition_current_state = IGNITION_ON;
	if (ignition_current_state != ignition_previous_state) {
		switch(ignition_current_state) {
			case IGNITION_ON:
			LOG_DEBUG("Ignition On");
			ignition_previous_state = IGNITION_ON;
			screen_on_off = 1;
			break;
			case IGNITION_OFF:
			LOG_DEBUG("Ignition Off");
			ignition_previous_state = IGNITION_OFF;
			for (int i = 0; i < sizeof(switches) / sizeof(switches[0]); i++) {
				interfaceScreen.writeNum(String(switches[i].display_name + ".val"), (int)false);
				// handleSwitchEvent(i, GUI_DISPLAY);
				switches[i].state = false;
				delay(50);
				switches[i].handle_switch_event(GUI_DISPLAY);
				//digitalWrite(switches[i].pin, switches[i].state ^ switches[i].active_low);   //
			}
			screen_on_off = 2;
			gpio_wakeup_enable(GPIO_NUM_19, GPIO_INTR_LOW_LEVEL);
    		esp_sleep_enable_gpio_wakeup();
			delay(500);
     		esp_light_sleep_start();
			ESP.deepSleep(1000000);
			ESP.restart();
			break;
		}
	}
	
	// delay(50);
	// LOG_DEBUG("[APP] Free memory:", esp_get_free_heap_size(), "bytes");



	// LOG_DEBUG("Voltage:", voltage, "V");
	// LOG_DEBUG("Light: ", currentSensorData.lightLux, "Gyro Temp: ", currentSensorData.gyroTemp, "Gyro Pitch: ", currentSensorData.gyroPitch, "Gyro Roll: ", currentSensorData.gyroRoll);

	// runnerSCH.execute();

}

// enum task_list{
// 	SEND_CAN
// };
// task_list current_task;

// void *task_handler(void *threadid) {
// 	while(true) {
// 		switch (current_task) {
// 			case SEND_CAN:

// 			break;
// 		}
// 	}
// }
// TODO: [ORC-18] Issue with Stack, getting a stack overflow
void *nextion_screen_thread(void *threadid) {
	LOG_DEBUG("Thread Running on Core: ", xPortGetCoreID());
	while (true) {
		interfaceScreen.NextionListen();
		if (screen_on_off == 1) {
			interfaceScreen.writeNum("sleep", 0);
			LOG_DEBUG("SCREEN_ON");
			screen_on_off = -1;
		}
		if (screen_on_off == 2) {
			interfaceScreen.writeNum("sleep", 1);
			LOG_DEBUG("SCREEN_OFF");
			screen_on_off = -1;
		}
		NXTpage = interfaceScreen.currentPageId;
		switch (NXTpage) {
		case SPLASH:
			//Serial.println("NXT Page 0");
			break;
		case START:
			//Serial.println("NXT Page 1");
			//getGyroData();
			if (!halt) {
				
				interfaceScreen.writeNum("P1_ROLL.val", currentSensorData.gyroRoll);
				interfaceScreen.writeNum("P1_PITCH.y", currentSensorData.gyroPitch);
				interfaceScreen.writeStr("P1_TEMP.txt", currentSensorData.gyroTemp);
				interfaceScreen.writeStr("P1_LUX.txt", currentSensorData.lightLux);
			}




			break;
		case MAIN:
			//Serial.println("NXT Page 2");
			break;
		}
		delay(50);
		//LOG_DEBUG("NXT Task");
		//yield();
		if (gpio_expander_index != -1) {
			LOG_DEBUG("Current MAP Index:", gpio_expander_index, gpio_expander_io_map[gpio_expander_index].pin_function);
			if (gpio_expander_io_map[gpio_expander_index].switch_index >= 0 && gpio_expander_io_map[gpio_expander_index].switch_index <= 99){
				switches[gpio_expander_io_map[gpio_expander_index].switch_index].handle_switch_event(EXT_IO_LOCAL_BUTTON);
				LOG_DEBUG("Stack High Mark:", uxTaskGetStackHighWaterMark(NULL));
			} else LOG_DEBUG("Input not Implemented");
			gpio_expander_index = -1;
		}
	}
	return 0;
}



void trigger0(){
	//Switches Trigger Function
	//LOG_INFO("trigger0 - Switches Service Function");
	int sw = interfaceScreen.readByte();
	delay(50);
	halt = true;
	String tempLog;
	if (sw != -1) {
		switch(switches[sw].display_type) {
		case TOGGLE:
			switches[sw].state = interfaceScreen.readNumber(String(switches[sw].display_name + ".val"));
			tempLog = "";
			tempLog.concat("Toggle Switch - Index: ");
			tempLog.concat(sw);
			tempLog.concat(" - Value: ");
			tempLog.concat(switches[sw].state);
			tempLog.concat(" - Switch Name: ");
			tempLog.concat(String(switches[sw].display_name + ".val"));
			tempLog.concat(" - Active Low: ");
			tempLog.concat(switches[sw].active_low);
			LOG_DEBUG(tempLog);
			switches[sw].handle_switch_event(GUI_DISPLAY);

			break;
		case BUTTON:
			LOG_DEBUG("Not implemented");
			break;
		}
	}
	else
		indexSWError();
	delay(50);
	halt = false;
}
void indexSWError() {
	LOG_ERROR("indexSWError - Error Occurred, Reinitializing Switches");
	for (int i = 0; i < sizeof(switches) / sizeof(switches[0]); i++) {
		interfaceScreen.writeNum(String(switches[i].display_name + ".val"), (int)switches[i].state);
	}
}


void trigger1(){
	//Page Init
	interfaceScreen.NextionListen();
	NXTpage = interfaceScreen.currentPageId;
	delay(50);
	switch (NXTpage) {
	case SPLASH:
		//Serial.println("NXT Page 0");
		break;
	case START:
		//Init Switches
		LOG_DEBUG("trigger1 - Initializing Switches");
		for (int i = 0; i < sizeof(switches) / sizeof(switches[0]); i++) {
			interfaceScreen.writeNum(String(switches[i].display_name + ".val"), (int)switches[i].state);
			// handleSwitchEvent(i, GUI_DISPLAY);
			delay(50);
			switches[i].handle_switch_event(GUI_DISPLAY);
			//digitalWrite(switches[i].pin, switches[i].state ^ switches[i].active_low);   //
		}



		break;
	case MAIN:
		//Serial.println("NXT Page 2");
		break;
	}
	delay(50);
}

void IRAM_ATTR isr_gpio_expander0() {
	interrupt_source = GPIO_EXPANDER;
	gpio_expander_number = 0;
	interrupt_trigger = true;
}
void IRAM_ATTR isr_gpio_expander1() {
	interrupt_source = GPIO_EXPANDER;
	gpio_expander_number = 1;
	interrupt_trigger = true;
}
void IRAM_ATTR isr_gpio_expander2() {
	interrupt_source = GPIO_EXPANDER;
	gpio_expander_number = 2;
	interrupt_trigger = true;
}
void handle_input_interrupt_event(int gpio_expander_index) {
	LOG_DEBUG("Current MAP Index:", gpio_expander_index, gpio_expander_io_map[gpio_expander_index].pin_function);
	if (gpio_expander_io_map[gpio_expander_index].switch_index >= 0 && gpio_expander_io_map[gpio_expander_index].switch_index <= 99){
		switches[gpio_expander_io_map[gpio_expander_index].switch_index].handle_switch_event(EXT_IO_LOCAL_BUTTON);
	}
}
struct test_message tx_message;
void *sensor_acquisition_thread(void *threadid) {
	LOG_DEBUG("Thread Running on Core: ", xPortGetCoreID());
	float roll=0;
	float pitch=0;
	int current_pin;
	int current_value;
	sensors_event_t a, g, temp;
	while (true) {
		//Gyro Acquisition, limit setting, and data perperation
		if (interrupt_trigger) {
			switch(interrupt_source) {
				case GPIO_EXPANDER:
				current_pin = io_expanders[gpio_expander_number].getLastInterruptPin();
				LOG_DEBUG(current_pin);
				current_pin = io_expanders[gpio_expander_number].getLastInterruptPin();
				LOG_DEBUG(current_pin);
				current_pin = io_expanders[gpio_expander_number].getLastInterruptPin();
				LOG_DEBUG(current_pin);
				LOG_DEBUG("Stack High Mark:", uxTaskGetStackHighWaterMark(NULL));

				current_value = io_expanders[gpio_expander_number].getCapturedInterrupt();
				LOG_DEBUG("GPIO_EXPANDER ISR: ", gpio_expander_number, current_pin, current_value);
				interrupt_trigger = false;
				delay(100);			
				while (true) {
					io_expanders[gpio_expander_number].clearInterrupts();
					if (io_expanders[gpio_expander_number].getLastInterruptPin() == 255) break;
					delay(100);
				}
				tx_message.message[20] = {NULL};
				strcpy(tx_message.message, "Nasir Aladdin - 1");
				strcpy(tx_message.message_id, "N");
				LOG_DEBUG("Sending:", tx_message.message);
				xQueueSend( test_queue,  (void *) &tx_message, ( TickType_t ) 10 );
				// if (current_pin >= 0 && current_pin <= 15) handle_input_interrupt_event(current_pin + gpio_expander_hw[gpio_expander_number].BASE_INDEX);
				// else LOG_ERROR("Index out of bound");
				tx_message.message[20] = {NULL};
				strcpy(tx_message.message, "Nasir Aladdin - 2");
				strcpy(tx_message.message_id, "N");
				LOG_DEBUG("Sending:", tx_message.message);
				xQueueSend( test_queue,  (void *) &tx_message, ( TickType_t ) 10 );
				if (current_pin >= 0 && current_pin <= 15) gpio_expander_index = current_pin + gpio_expander_hw[gpio_expander_number].BASE_INDEX;
				else LOG_ERROR("Index out of bound");




				// if (current_pin >= GPIO_EXPANDER0_BASE_ADDRESS && current_pin < GPIO_EXPANDER0_BASE_ADDRESS + 16 && gpio_expander_io_map[current_pin].switch_index != -1) {
				// 	LOG_DEBUG(switches[gpio_expander_io_map[current_pin].switch_index].display_name);
				// 	halt = true;
				// 	switches[gpio_expander_io_map[current_pin].switch_index].handle_switch_event(EXT_IO_LOCAL_BUTTON);
				// } else {
				// 	LOG_ERROR("Index out of bound");
				// }
				
				// io_expanders[0].clearInterrupts();
				// LOG_DEBUG("D18 After Clear: ", io_expanders[0].getLastInterruptPin(), " ", io_expanders[0].getCapturedInterrupt());
				break;

			}
		delay(200);
		halt = false;
		} else {
			currentSensorData.voltage = 0.00485 * analogRead(36) + 0.76864;
			gyroSensor.getEvent(&a, &g, &temp);
			roll=mapF(a.acceleration.y,-10,10,0,180);
			pitch=mapF(a.acceleration.x,-5.55,5.55,PITCH_ORIGIN_Y+50,PITCH_ORIGIN_Y-50);
			if (pitch > PITCH_ORIGIN_Y+50) pitch = PITCH_ORIGIN_Y+50;
			if (pitch < PITCH_ORIGIN_Y-50) pitch = PITCH_ORIGIN_Y-50;
			currentSensorData.gyroPitch=(long)pitch; 
			currentSensorData.gyroRoll=(long)roll;
			currentSensorData.gyroTemp = "";
			currentSensorData.gyroTemp.concat((int)temp.temperature);
			delay(50);
			//Light sensor Acquisition
			if (lightSensor.hasValue()) {
				currentSensorData.lightLux = "";
				currentSensorData.lightLux.concat((float)lightSensor.getLux());
				lightSensor.adjustSettings(90);
				lightSensor.start();
				//Serial.println(currentSensorData.lightLux,4);

			}
			delay(100);
		}
	}
	return 0;
}









float mapF(float x, float in_min, float in_max, float out_min, float out_max) { return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }
void sensors_init() {

	if (!gyroSensor.begin()) {
		LOG_ERROR("Failed to find MPU6050 chip");
		// while (1) delay(10);
	} else {
		LOG_INFO("MPU6050 Found!");

		gyroSensor.setAccelerometerRange(MPU6050_RANGE_8_G);
		switch (gyroSensor.getAccelerometerRange()) {
		case MPU6050_RANGE_2_G:
			LOG_DEBUG("Accelerometer range set to: +-2G");
			break;
		case MPU6050_RANGE_4_G:
			LOG_DEBUG("Accelerometer range set to: +-4G");
			break;
		case MPU6050_RANGE_8_G:
			LOG_DEBUG("Accelerometer range set to: +-8G");
			break;
		case MPU6050_RANGE_16_G:
			LOG_DEBUG("Accelerometer range set to: +-16G");
			break;
		}
		gyroSensor.setGyroRange(MPU6050_RANGE_500_DEG);
		switch (gyroSensor.getGyroRange()) {
		case MPU6050_RANGE_250_DEG:
			LOG_DEBUG("Gyro range set to: +- 250 deg/s");
			break;
		case MPU6050_RANGE_500_DEG:
				LOG_DEBUG("Gyro range set to: +- 500 deg/s");
			break;
		case MPU6050_RANGE_1000_DEG:
			LOG_DEBUG("Gyro range set to: +- 1000 deg/s");
			break;
		case MPU6050_RANGE_2000_DEG:
				LOG_DEBUG("Gyro range set to: +- 2000 deg/s");
			break;
		}

		gyroSensor.setFilterBandwidth(MPU6050_BAND_21_HZ);
		switch (gyroSensor.getFilterBandwidth()) {
		case MPU6050_BAND_260_HZ:
				LOG_DEBUG("Filter bandwidth set to: 260 Hz");
			break;
		case MPU6050_BAND_184_HZ:
				LOG_DEBUG("Filter bandwidth set to: 184 Hz");
			break;
		case MPU6050_BAND_94_HZ:
				LOG_DEBUG("Filter bandwidth set to: 94 Hz");
			break;
		case MPU6050_BAND_44_HZ:
				LOG_DEBUG("Filter bandwidth set to: 44 Hz");
			break;
		case MPU6050_BAND_21_HZ:
				LOG_DEBUG("Filter bandwidth set to: 21 Hz");
			break;
		case MPU6050_BAND_10_HZ:
				LOG_DEBUG("Filter bandwidth set to: 10 Hz");
			break;
		case MPU6050_BAND_5_HZ:
				LOG_DEBUG("Filter bandwidth set to: 5 Hz");
			break;
		}
		LOG_INFO("MPU6050 Configuration Complete!");
	}
	delay(100);
	if (!lightSensor.begin(BH1750_TO_GROUND)) {
		LOG_ERROR("Failed to find BH1750 chip");
		/*while (1) {
		delay(10);
		}*/
	} else {
		lightSensor.calibrateTiming();
		lightSensor.start();
		LOG_INFO("BH1750 Configuration Complete!");
	}
}




// void *interruptSCH(void *threadid) {
// 	LOG_DEBUG("Thread Running on Core: ", xPortGetCoreID());
// 	int current_pin;
// 	int current_value;
// 	while (true) {
// 		if (interrupt_trigger) {
// 			switch(interrupt_source) {
// 				case GPIO_EXPANDER0:
// 				current_pin = io_expanders[0].getLastInterruptPin();
// 				current_value = io_expanders[0].getCapturedInterrupt();
// 				LOG_DEBUG("GPIO_EXPANDER0 ISR: ", current_pin, " ", current_value);
// 				interrupt_trigger = false;
// 				delay(100);			
// 				while (true) {
// 					io_expanders[0].clearInterrupts();
// 					if (io_expanders[0].getLastInterruptPin() == 255) break;
// 					delay(100);
// 				}
// 				if (current_pin >= GPIO_EXPANDER0_BASE_ADDRESS && current_pin < GPIO_EXPANDER0_BASE_ADDRESS + 16 && gpio_expander_io_map[current_pin].switch_index != -1) {
// 					LOG_DEBUG(switches[gpio_expander_io_map[current_pin].switch_index].display_name);
// 					switches[gpio_expander_io_map[current_pin].switch_index].handle_switch_event(EXT_IO_LOCAL_BUTTON);
// 				} else {
// 					LOG_ERROR("Index out of bound");
// 				}
				
// 				// io_expanders[0].clearInterrupts();
// 				// LOG_DEBUG("D18 After Clear: ", io_expanders[0].getLastInterruptPin(), " ", io_expanders[0].getCapturedInterrupt());
// 				break;

// 			}
			
// 		}
// 		delay(200);
// 	}
// 	return 0;
	
// }
void *bluetooth_handler_thread(void *threadid) {
	LOG_DEBUG("Thread Running on Core: ", xPortGetCoreID());
	String incoming_bluetooth;
	while (true) {


		if (SerialBT.available()) {

			
			incoming_bluetooth = SerialBT.readStringUntil(0x0A);
			LOG_DEBUG(incoming_bluetooth.substring(0, 2));
			LOG_DEBUG(incoming_bluetooth.substring(2, 4));
			LOG_DEBUG("Stack High Mark:", uxTaskGetStackHighWaterMark(NULL));
			if (incoming_bluetooth.substring(0, 2) == "SW") {
				// int index = (int) incoming_bluetooth.substring(2, 3).toInt();
				halt = true;
				delay(50);
				switches[incoming_bluetooth.substring(2, 4).toInt()].handle_switch_event(EXT_IO_LOCAL_BUTTON);
				halt = false;
				SerialBT.println("Bluetooth has been Awesomed!");
			}
			LOG_DEBUG(incoming_bluetooth);
		}


		delay(50);
	}
	return 0;
}
void led_sequence() {
	gpio_defintions temp_gpio0;
	gpio_defintions temp_gpio1;
	int led_state = HIGH;
	for (int j = 0; j < 2; j++){
		for (int i = 0; i < 5; i++){
			temp_gpio0 = gpio_expander_io_map[led_indecies[i]];
			temp_gpio1 = gpio_expander_io_map[led_indecies[9 - i]];
			delay(100);
			// LOG_DEBUG(temp_gpio.pin_function);
			io_expanders[temp_gpio0.gpio_expander].digitalWrite(temp_gpio0.pin_number, led_state);
			io_expanders[temp_gpio1.gpio_expander].digitalWrite(temp_gpio1.pin_number, led_state);
			
		}
		delay(500);
		led_state = LOW;
	}
		// for (int i = 0; i < 5; i++){
		// 	temp_gpio0 = gpio_expander_io_map[led_indecies[i]];
		// 	temp_gpio1 = gpio_expander_io_map[led_indecies[9 - i]];
		// 	delay(200);
		// 	// LOG_DEBUG(temp_gpio.pin_function);
		// 	io_expanders[temp_gpio0.gpio_expander].digitalWrite(temp_gpio0.pin_number, LOW);
		// 	io_expanders[temp_gpio1.gpio_expander].digitalWrite(temp_gpio1.pin_number, LOW);
			
		// }
	// for (int i = 9; i >= 0; i--){
	// 	temp_gpio0 = gpio_expander_io_map[led_indecies[i]];
	// 	io_expanders[temp_gpio0.gpio_expander].digitalWrite(temp_gpio0.pin_number, LOW);
	// 	delay(100);
	// }
}