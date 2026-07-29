#include "CanUtils.h"

int can_relay(int can_address, bool state, int pin){
	bool ok;
	CANMessage frame;
	CANMessage recv_frame;
	bool ack_recieved = false;
	char msgString[128];
	char final_msgString[256];
    final_msgString[0] = '\0'; // Initialize string 
	frame.id= can_address;
	int challenge = random(0, 254);
	frame.len = MAX_CAN_FRAME_DATA_LEN;
	frame.data16[0] = state;
	frame.data16[1] = pin;
	frame.data16[2] = 0x0000;
	frame.data[6] = 0x00;
	frame.data[7] = challenge;
	// frame.data16[3] = 0x0000;
	// ACAN_ESP32::can.resetDriverReceiveBufferPeakCount();
	ok = ACAN_ESP32::can.tryToSend (frame) ;
	bool print_recv = true;
	if(ok) {
		LOG_DEBUG("CAN MESSAGE SENT: ", ok);
	}
	for(int j = 9; j>=0; --j){
		while (ACAN_ESP32::can.receive(recv_frame)) {
			if (print_recv){
				for(int i = 7; i>=0; --i){
					sprintf(msgString, "%d-0x%.2X ", i, recv_frame.data[i]);
					strcat(final_msgString, msgString);
				}
				LOG_DEBUG(final_msgString);
			}
			ack_recieved = true;
			
		}
		if (ack_recieved) break;
		delay(200);
	}
	if (ack_recieved) { 
		LOG_DEBUG("Response Recieved!");
		if (challenge == recv_frame.data[7]) {
			LOG_DEBUG("Response Challenge Success!");
			return 0;
		}
		else { 
			LOG_DEBUG("Response Challenge Fail!");
			for(int j = 100; j>=0; --j){
				while (ACAN_ESP32::can.receive(recv_frame)) {
				}
			}
			return -2;
		}
		
	}
	else { 
		LOG_ERROR("No Response Recieved!"); 
		return -1;
	}
	
	return 0;

}
