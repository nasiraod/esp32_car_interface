#include <Arduino.h>

// DebugLog Config must be before other includes that use it
#define DEBUGLOG_DEFAULT_LOG_LEVEL_DEBUG
#define LOG_ATTACH_SERIAL(Serial)
#include <DebugLog.h>

#include "dispDefsNXT.h"
#include "dataPacket.h"
#include "arduinoDefs.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <hp_BH1750.h>
#include <Adafruit_MCP23X17.h>

// Suppress deprecated warnings for ACAN and BluetoothSerial
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <ACAN_ESP32.h>
#include "BluetoothSerial.h"
#pragma GCC diagnostic pop

#include <core_version.h>
#include <rtc_wdt.h>
#include <pthread.h>
#include <esp_sleep.h>
#include <EEPROM.h>

#include "Switch.h"
#include "CanUtils.h"

#define MAX_CAN_FRAME_DATA_LEN   8

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Suppress warning for instantiation of deprecated class
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
BluetoothSerial SerialBT;
#pragma GCC diagnostic pop

#define EEPROM_SIZE 128

////////////////////////////////
///////Light Sensor Object//////
////////////////////////////////
hp_BH1750 lightSensor;

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

struct test_message {
	char message_id[2];
	char message[20];
};
QueueHandle_t test_queue;

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
float mapF(float x, float in_min, float in_max, float out_min, float out_max);

bool halt = false;
////////////////////////////////
////////Global Variables////////
////////////////////////////////
struct sensorData currentSensorData;
int ilumination_limits[] = {300, 500};
uint8_t ilumination_hysteresis = 25;
ilumination_states ilumination_current_state = NIGHT;
ilumination_states ilumination_previous_state = NIGHT;
ignition_states ignition_current_state = IGNITION_INIT;
ignition_states ignition_previous_state = IGNITION_INIT;
int NXTpage = 1;

////////////////////////////////
////////////Tasks///////////////
////////////////////////////////
void *i2c_devices_thread(void *threadid);
void *bluetooth_handler_thread(void *threadid);
pthread_t sensors_thread;
pthread_t bluetooth_thread;

void bluetooth_init() {
	SerialBT.begin("Car Interface");
	LOG_DEBUG("Bluetooth Started! Ready to pair...");
}

//////////////////////////////////////
////////////GPIO ISRs/////////////////
//////////////////////////////////////
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


void io_init() {
	LOG_DEBUG("Initializing IOs");
	int gpio_map_size = num_gpio_map_size;
	int active_gpio_expanders = num_active_gpio_expanders;
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

		LOG_DEBUG(temp_log_message);
	}
	
	pinMode(gpio_expander_hw[0].INT_PIN, INPUT_PULLUP);
	attachInterrupt(gpio_expander_hw[0].INT_PIN, isr_gpio_expander0, FALLING);

	pinMode(gpio_expander_hw[1].INT_PIN, INPUT_PULLUP);
	attachInterrupt(gpio_expander_hw[1].INT_PIN, isr_gpio_expander1, FALLING);

	pinMode(IGNITION_PIN, INPUT_PULLUP);

}

void setup() {
	EEPROM.begin(EEPROM_SIZE);
	test_queue = xQueueCreate( 10, sizeof(struct test_message) );
	// initialize both serial ports:
	
	Serial.begin(115200);
	Wire1.begin(GPIO_I2C_SDA, GPIO_I2C_SCL);
	// Serial1.begin(9600);
	Serial.println("Car Interface");
	bluetooth_init();
	for (int i = 0; i < num_switches; i++) {
		switches[i].state = EEPROM.read(i);
	}
	
	
	sensors_init();
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
	pthread_create(&sensors_thread, &attr, i2c_devices_thread, (void *) 1);
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
        // HALT if config error
        while(1) {
            delay(100);
        }
	}
	delay(100);
	
	led_sequence();
	delay(100);
	for (int i = 0; i < num_switches; i++) {
		delay(50);
		switches[i].handle_switch_event(GUI_DISPLAY);
	}


}
struct test_message recv_message;
int gpio_expander_index = -1;
char test_char[20];
int count = 0;
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
			for (int i = 0; i < num_switches; i++) {
				EEPROM.write(i, switches[i].state);
				EEPROM.commit();
			}
			for (int i = 0; i < num_switches; i++) {
				switches[i].state = false;
				delay(50);
				switches[i].handle_switch_event(GUI_DISPLAY);
			}
			screen_on_off = 2;
			can_relay(0x7FF, 0, 0xFF);
			gpio_wakeup_enable(GPIO_NUM_19, GPIO_INTR_LOW_LEVEL);
    		esp_sleep_enable_gpio_wakeup();
			delay(500);
     		esp_light_sleep_start();
			ESP.deepSleep(1000000);
			ESP.restart();
			break;
		}
		

	}

}




void handle_input_interrupt_event(int gpio_expander_index) {
	LOG_DEBUG("Current MAP Index:", gpio_expander_index, gpio_expander_io_map[gpio_expander_index].pin_function);
	if (gpio_expander_io_map[gpio_expander_index].switch_index >= 0 && gpio_expander_io_map[gpio_expander_index].switch_index <= 99){
		switches[gpio_expander_io_map[gpio_expander_index].switch_index].handle_switch_event(EXT_IO_LOCAL_BUTTON);
	}
}
struct test_message tx_message;
void *i2c_devices_thread(void *threadid) {
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
				tx_message.message[20] = {NULL};
				strcpy(tx_message.message, "Nasir Aladdin - 2");
				strcpy(tx_message.message_id, "N");
				LOG_DEBUG("Sending:", tx_message.message);
				xQueueSend( test_queue,  (void *) &tx_message, ( TickType_t ) 10 );
				if (current_pin >= 0 && current_pin <= 15){
					gpio_expander_index = current_pin + gpio_expander_hw[gpio_expander_number].BASE_INDEX;
					handle_input_interrupt_event(gpio_expander_index);
				} else LOG_ERROR("Index out of bound");
				
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
	}
}

void led_sequence() {
	bool led_flash = true;
	for(int j = 9; j>=0; --j){
		for (int i = 0; i < num_led_indecies; i++) {
			gpio_defintions temp_gpio;
			temp_gpio = gpio_expander_io_map[led_indecies[i]];
			io_expanders[temp_gpio.gpio_expander].digitalWrite(temp_gpio.pin_number, led_flash);
		}
		led_flash = !led_flash;
		delay(200);
	}
}

void *bluetooth_handler_thread(void *threadid) {
	while (true) {
		if (Serial.available()) {
			SerialBT.write(Serial.read());
		}
		if (SerialBT.available()) {
			Serial.write(SerialBT.read());
		}
		delay(20);
	}
	return NULL;
}