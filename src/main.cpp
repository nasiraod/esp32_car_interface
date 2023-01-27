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





////////////////////////////////
///////Light Sensor Object//////
////////////////////////////////
hp_BH1750 lightSensor;
////////////////////////////////

////////////////////////////////
//////GPIO Expander Object//////
////////////////////////////////
Adafruit_MCP23X17 inputExpander0;
////////////////////////////////


////////////////////////////////
/////Nextion Display Pages//////
////////////////////////////////
bool halt = false;
EasyNex interfaceScreen(Serial2);
enum NXTPages {
	SPLASH,
	START,
	MAIN,
	SETTINGS,
	INTERIOR
};
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
struct switchStruct{
	String NXTName;
	int NXTType;
	bool state;
	bool activeLow;
	int devLoc;
	int address;
	int pin;
  int physicalType;

};
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
struct switchStruct switches[] = {
		{ "P1_SW0", TOGGLE, true, true, GPIO_CAN, 0x7FF, 0, BUTTON },						// Roof Toggle SW
		{ "P1_SW1", TOGGLE, false, true, GPIO_CAN, 0x7FF, 1, BUTTON },						// Bumper Toggle SW
		{ "P1_SW2", TOGGLE, false, true, GPIO_CAN, 0x7FF, 2, BUTTON },						// Ditch Toggle SW
		{ "P1_SW3", TOGGLE, false, true, GPIO_CAN, 0x7FF, 3, BUTTON },						// Rear Toggle SW
		{ "P1_SW4", TOGGLE, false, true, GPIO_CAN, 0x7FF, 4, BUTTON }						// Rock Toggle SW
};
////////////////////////////////


////////////////////////////////
///////Function Definitions/////
////////////////////////////////
void raspPacketParser(int numBytes);
void nextionScreenSendSCH();
void sensorAcqSCH();
void gyroSetup();
void handleSwitchEvent (int incomingSwitch);
void raspSerialListenerSCH();
//void getGyroData();
void indexSWError();
float mapF(float x, float in_min, float in_max, float out_min, float out_max);
////////////////////////////////

////////////////////////////////
////////Global Variables////////
////////////////////////////////
struct sensorData currentSensorData;
const int BUFFER_SIZE = 128;
char inByte[BUFFER_SIZE];
float roll=0;
float pitch=0;
int NXTpage=1;
////////////////////////////////

////////////////////////////////
////////////Tasks///////////////
////////////////////////////////
Task NXTTask(0, TASK_FOREVER, &nextionScreenSendSCH);
Task sensorsTask(0, TASK_FOREVER, &sensorAcqSCH);

Scheduler runnerSCH;
////////////////////////////////

void setup() {
	// initialize both serial ports:
	Serial.begin(115200);
	Serial1.begin(9600);
	interfaceScreen.begin(9600);
	Serial.println("Car Interface Test");
	

	/* for (int i = 0; i < sizeof(switches) / sizeof(switches[0]); i++) {
		pinMode(switches[i].pin, OUTPUT);
	}*/
	pinMode(ILLUMINATION_PIN, INPUT_PULLUP);
	pinMode(IGNITION_PIN, INPUT_PULLUP);

	gyroSetup();
	interfaceScreen.writeStr("page 0");
	lightSensor.begin(BH1750_TO_GROUND);
	lightSensor.calibrateTiming();
	lightSensor.start();
	delay(50);
	
	
	
	
	//Scheduler.startLoop(raspSerialListenerSCH);
	//Scheduler.startLoop(nextionScreenSendSCH);
	//Scheduler.startLoop(sensorAcqSCH);
	runnerSCH.init();
	LOG_DEBUG("Scheduler Initialized");
	runnerSCH.addTask(NXTTask);
	LOG_DEBUG("Nextion Task added");
	runnerSCH.addTask(sensorsTask);
	NXTTask.enable();
	sensorsTask.enable();
	

	

	/*if (Can0.begin(CAN_BPS_250K))  {
	  }
	  else {
	    Serial.println("CAN initialization (sync) ERROR");
	  }
	Can0.watchFor(0x7FF);
*/
}

void loop() {
	//Serial.write("Main Loop!\n");
	//getGyroData();
	//delay(1000);
	runnerSCH.execute();

}



void raspSerialListenerSCH() {
	// read from port 1, send to port 0:
	if (Serial1.available()) {
		//String inByte = Serial1.readString();
		int numBytes = Serial1.readBytesUntil(TERMINATOR, inByte, BUFFER_SIZE);
		//Serial.println(inByte);
		//Serial.write(inByte);
		//char * inByteCh = inByte;
		raspPacketParser(numBytes);










	}
	// read from port 0, send to port 1:
	if (Serial.available()) {
		int inByte = Serial.read();
		Serial1.write(inByte);
	}
	yield();
}
//Data Packet: <HEADER_TYPE><COMMAND><DATA_TYPE><ELEMENT><NUMBER_DATA_BYTES><DATA><TERMINATOR>

void raspPacketParser(int numBytes) {
	switch (inByte[0]) {
	case CONFIG:
		Serial.write("Config command received!\n");
		if (inByte[1] == GET) {
			Serial.write("Config get command received!\n");
		}
		else if (inByte[1] == SET) {
			Serial.write("Config set command received!\n");
		}
		else
			Serial.write("Invalid command!\n");

		break;
	case STATE:
		Serial.write("State command received!\n");
		break;
	default:
		// block of code default: do something when var is not equal to any of above label
		break;
	}
	//Done Parsing, empty buffer
	memset(inByte, 0, BUFFER_SIZE);
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
		switch(switches[sw].NXTType) {
		case TOGGLE:
			//tempLog.concat("Toggle Switch - Index: ");
			//tempLog.concat(sw);

			//LOG_DEBUG("Toggle Switch");
			//LOG_DEBUG("trigger0 - Index: ", sw);
			//Serial.print(interfaceScreen.readByte());
			//Serial.println(interfaceScreen.readByte());
			switches[sw].state = interfaceScreen.readNumber(String(switches[sw].NXTName + ".val"));
			//LOG_DEBUG("trigger0 - Value: ", switches[sw].state); //Serial.println(switches[sw].state);
			//LOG_DEBUG("trigger0 - Switch Name: ", String(switches[sw].name + ".val")); //Serial.println(String(switches[sw].name + ".val"));
			//LOG_DEBUG("trigger0 - Active Low: ", switches[sw].activeLow);
			tempLog = "";
			tempLog.concat("Toggle Switch - Index: ");
			tempLog.concat(sw);
			tempLog.concat(" - Value: ");
			tempLog.concat(switches[sw].state);
			tempLog.concat(" - Switch Name: ");
			tempLog.concat(String(switches[sw].NXTName + ".val"));
			tempLog.concat(" - Active Low: ");
			tempLog.concat(switches[sw].activeLow);
			LOG_DEBUG(tempLog);

			handleSwitchEvent(sw);




			break;
		case BUTTON:
			//LOG_DEBUG("Not implemented");
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
		interfaceScreen.writeNum(String(switches[i].NXTName + ".val"), (int)switches[i].state);
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
			interfaceScreen.writeNum(String(switches[i].NXTName + ".val"), (int)switches[i].state);
			handleSwitchEvent(i);
			//digitalWrite(switches[i].pin, switches[i].state ^ switches[i].activeLow);   //
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

	//Light sensor Acquisition
	if (lightSensor.hasValue()) {
		currentSensorData.lightLux = "";
		currentSensorData.lightLux.concat((float)lightSensor.getLux());
		lightSensor.adjustSettings(90);
		lightSensor.start();
		//Serial.println(currentSensorData.lightLux,4);

	}
	delay(50);
	yield();
}


void handleSwitchEvent (int incomingSwitch) {
	LOG_DEBUG("Switches invoked");
	switch (switches[incomingSwitch].devLoc) {
	case GPIO_CAN:
		/*CAN_FRAME commandOut;
		commandOut.id = 0x7FF;
		commandOut.length = MAX_CAN_FRAME_DATA_LEN;
		commandOut.data.s0 = switches[incomingSwitch].state ^ switches[incomingSwitch].activeLow;
		commandOut.data.s1 = switches[incomingSwitch].pin;
		commandOut.data.s2 = 0x0000;
		commandOut.data.s3 = 0x0000;
		commandOut.extended = 0;
		//Can0.sendFrame(commandOut);*/
		break;
	case GPIO_LOCAL:
		digitalWrite(switches[incomingSwitch].pin, switches[incomingSwitch].state ^ switches[incomingSwitch].activeLow);
		break;
	}
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



