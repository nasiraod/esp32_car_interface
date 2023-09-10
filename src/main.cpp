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
#include <TaskScheduler.h>
#include <Adafruit_MCP23X17.h>
//#include <due_can.h>
#define MAX_CAN_FRAME_DATA_LEN   8
#include <ACAN_ESP32.h>
#include <core_version.h>
#include "soc/rtc_wdt.h"



////////////////////////////////
///////Light Sensor Object//////
////////////////////////////////
hp_BH1750 lightSensor;
////////////////////////////////

////////////////////////////////
/////GPIO Expander Objects//////
////////////////////////////////
Adafruit_MCP23X17 input_expanders[3];
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
};
////////////////////////////////

////////////////////////////////
///////////Switches/////////////
////////////////////////////////
class Switch {
	public:
		String nextion_name;
		int nextion_type;
		bool state;
		bool active_low;
		int pin_location;
		int address;
		int input_expander;
		int input_pin;
		int input_led;
		int output_pin;
		int physical_type;
		Switch(String nextion_name, int nextion_type, bool state, bool active_low, int pin_location, int address, int input_expander, int input_pin, int input_led, int output_pin, int physical_type) {
			this -> nextion_name = nextion_name;
			this -> nextion_type = nextion_type;
			this -> state = state;
			this -> active_low = active_low;
			this -> pin_location = pin_location;
			this -> address = address;
			this -> input_expander = input_expander;
			this -> input_pin = input_pin;
			this -> input_led = input_led;
			this -> output_pin = output_pin;
			this -> physical_type = physical_type;
		}
		bool handle_switch_event(int source) {
			CANMessage frame;
			bool ok;
			// frame.id = address;
			//ACAN_ESP32::can.tryToSend (frame) ;
			
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
					// char temp[] = "";
					// sprintf(temp, "CAN MESSAGE SENT: %x", frame.id);
					LOG_DEBUG("CAN MESSAGE SENT: ", ok);
					// LOG_DEBUG(temp);

				}
				
				break;
			case GPIO_LOCAL:
				LOG_DEBUG("GPIO_LOCAL");
				// digitalWrite(switches[incomingSwitch].output_pin, switches[incomingSwitch].state ^ switches[incomingSwitch].active_low);
				digitalWrite(output_pin, state ^ active_low);
				break;
			}
			return true;
		}

};

Switch switches[] = {
	Switch( "P1_SW0", TOGGLE, true, true, GPIO_CAN, 0x7FF, 0, 0, 65, 0, BUTTON),					// Roof Toggle SW
	Switch( "P1_SW1", TOGGLE, false, true, GPIO_CAN, 0x7FF, 0, 1, 65, 1, BUTTON ),					// Bumper Toggle SW
	Switch( "P1_SW2", TOGGLE, false, true, GPIO_CAN, 0x7FF, 0, 2, 65, 2, BUTTON ),					// Ditch Toggle SW
	Switch( "P1_SW3", TOGGLE, false, true, GPIO_CAN, 0x7FF, 0, 3, 65, 3, BUTTON ),					// Rear Toggle SW
	Switch( "P1_SW4", TOGGLE, false, true, GPIO_CAN, 0x7FF, 0, 4, 65, 4, BUTTON )					// Rock Toggle SW
};





bool interrupt_trigger = false;
interrupt_sources interrupt_source;

////////////////////////////////
///////Function Definitions/////
////////////////////////////////
void raspPacketParser(int numBytes);
void nextionScreenSendSCH();
void sensorAcqSCH();
void interruptSCH();
void gyroSetup();
void handleSwitchEvent (int incomingSwitch, int triggerSource);
// void raspSerialListenerSCH();
// void getGyroData();
void indexSWError();
float mapF(float x, float in_min, float in_max, float out_min, float out_max);
void IRAM_ATTR isrD18();
////////////////////////////////

////////////////////////////////
////////Global Variables////////
////////////////////////////////
struct sensorData currentSensorData;
const int BUFFER_SIZE = 128;
// char inByte[BUFFER_SIZE];
float roll=0;
float pitch=0;
int NXTpage=1;
//  CAN ESP32 Desired Bit Rate
// static const uint32_t DESIRED_BIT_RATE = 1000UL * 250UL ; // 250 Kb/s

////////////////////////////////

////////////////////////////////
////////////Tasks///////////////
////////////////////////////////
Task NXTTask(0, TASK_FOREVER, &nextionScreenSendSCH);
Task sensorsTask(0, TASK_FOREVER, &sensorAcqSCH);
Task interruptTask(0, TASK_FOREVER, &interruptSCH);

Scheduler runnerSCH;
////////////////////////////////
void io_init() {
	LOG_DEBUG("Initializing IOs");
	pinMode(GPIO_EXPANDER0_RST_PIN, OUTPUT);
	digitalWrite(GPIO_EXPANDER0_RST_PIN, LOW);
	delay(50);
	digitalWrite(GPIO_EXPANDER0_RST_PIN, HIGH);
	int count = 0;
	while (true) {
		count++;
		if (count >= 10 || input_expanders[0].begin_I2C(0x20)) break;
		LOG_DEBUG("GPIO Expander: Error Initializing");
		delay(100);

	}
	input_expanders[0].setupInterrupts(true, false, LOW);
	input_expanders[0].pinMode(1, INPUT_PULLDOWN);
	input_expanders[0].setupInterruptPin(1, HIGH);
	pinMode(GPIO_EXPANDER0_INT_PIN, INPUT_PULLUP);
	attachInterrupt(GPIO_EXPANDER0_INT_PIN, isrD18, FALLING);
	pinMode(ILLUMINATION_PIN, INPUT_PULLUP);
	pinMode(IGNITION_PIN, INPUT_PULLUP);

}
void setup() {
	// initialize both serial ports:
	
	Serial.begin(115200);
	// Serial1.begin(9600);
	interfaceScreen.begin(9600);
	Serial.println("Car Interface Test");
	io_init();
	gyroSetup();
	interfaceScreen.writeStr("page 0");
	lightSensor.begin(BH1750_TO_GROUND);
	lightSensor.calibrateTiming();
	lightSensor.start();
	delay(50);

	runnerSCH.init();
	LOG_DEBUG("Scheduler Initialized");
	runnerSCH.addTask(NXTTask);
	LOG_DEBUG("Nextion Task added");
	runnerSCH.addTask(sensorsTask);
	runnerSCH.addTask(interruptTask);
	NXTTask.enable();
	sensorsTask.enable();
	interruptTask.enable();
	
	
	LOG_DEBUG("Configure ESP32 CAN");
	ACAN_ESP32_Settings settings (250 * 1000) ;
	//settings.mRxPin = GPIO_NUM_4 ; // Optional, default Tx pin is GPIO_NUM_4
	//settings.mTxPin = GPIO_NUM_5 ; // Optional, default Rx pin is GPIO_NUM_5
	// TODO: Change CAN to Normal Mode for transmission error handling, would require changing the switch event class method
	settings.mRequestedCANMode = ACAN_ESP32_Settings::LoopBackMode ;
	const uint32_t errorCode = ACAN_ESP32::can.begin (settings) ;
	
	if (errorCode == 0) {
		LOG_DEBUG("Configuration ESP32 OK!");
	}else{
		LOG_ERROR("Configuration error");
		LOG_ERROR(errorCode, HEX);
	}

}

void loop() {
	//Serial.write("Main Loop!\n");
	//getGyroData();
	//delay(1000);
	// LOG_DEBUG(inputExpander0.getLastInterruptPin(), " ", inputExpander0.getCapturedInterrupt());
	runnerSCH.execute();

}




void nextionScreenSendSCH() {
	interfaceScreen.NextionListen();
	NXTpage=interfaceScreen.currentPageId;
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
}



void trigger0(){
	//Switches Trigger Function
	//LOG_INFO("trigger0 - Switches Service Function");
	int sw = interfaceScreen.readByte();
	delay(50);
	halt = true;
	String tempLog;
	if (sw != -1) {
		switch(switches[sw].nextion_type) {
		case TOGGLE:
			switches[sw].state = interfaceScreen.readNumber(String(switches[sw].nextion_name + ".val"));
			tempLog = "";
			tempLog.concat("Toggle Switch - Index: ");
			tempLog.concat(sw);
			tempLog.concat(" - Value: ");
			tempLog.concat(switches[sw].state);
			tempLog.concat(" - Switch Name: ");
			tempLog.concat(String(switches[sw].nextion_name + ".val"));
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
		interfaceScreen.writeNum(String(switches[i].nextion_name + ".val"), (int)switches[i].state);
	}
}


void trigger1(){
	//Page Init
	interfaceScreen.NextionListen();
	NXTpage=interfaceScreen.currentPageId;
	delay(50);
	switch (NXTpage) {
	case SPLASH:
		//Serial.println("NXT Page 0");
		break;
	case START:
		//Init Switches
		LOG_DEBUG("trigger1 - Initializing Switches");
		for (int i = 0; i < sizeof(switches) / sizeof(switches[0]); i++) {
			interfaceScreen.writeNum(String(switches[i].nextion_name + ".val"), (int)switches[i].state);
			// handleSwitchEvent(i, GUI_DISPLAY);
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


void sensorAcqSCH() {
	//Gyro Acquisition, limit setting, and data perperation
	sensors_event_t a, g, temp;
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
	delay(50);
}









float mapF(float x, float in_min, float in_max, float out_min, float out_max) { return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }
void gyroSetup() {

	  if (!gyroSensor.begin()) {
	    LOG_ERROR("Failed to find MPU6050 chip");
	    while (1) {
	      delay(10);
	    }
	  }
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
	  delay(100);
}


void IRAM_ATTR isrD18() {
	interrupt_source = D18;
	interrupt_trigger = true;
}

void interruptSCH() {
	if (interrupt_trigger) {
		switch(interrupt_source) {
			case D18:
			int current_pin = input_expanders[0].getLastInterruptPin();
			int current_value = input_expanders[0].getCapturedInterrupt();
			LOG_DEBUG("D18 ISR: ", current_pin, " ", current_value);
			interrupt_trigger = false;
			delay(100);
			int count = 0;
			while (true) {
				input_expanders[0].clearInterrupts();
				if (input_expanders[0].getLastInterruptPin() == 255) break;
				delay(100);
			}
			// input_expanders[0].clearInterrupts();
			// LOG_DEBUG("D18 After Clear: ", input_expanders[0].getLastInterruptPin(), " ", input_expanders[0].getCapturedInterrupt());
			break;

		}
		
	}
	delay(50);
	
}