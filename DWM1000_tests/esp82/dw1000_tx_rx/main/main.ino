#include "dw1000.h"

#define RX_BUFFER_SIZE 6

DWM_Module dwm1 = { D8, D4 }; // CS=D8, RESET=D4

void setup() {
    Serial.begin(115200);
    pinMode(dwm1.cs_pin, OUTPUT);
    pinMode(dwm1.reset_pin, OUTPUT);
    digitalWrite(dwm1.cs_pin, HIGH);

    // to allow time to load the serial monitor
    int countdown=10;
    for (int i = countdown; i >= 0 ; i--){
        Serial.println(i);
        delay(1000);
    }
    SPI.begin();
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);

    dwm_reset(&dwm1);
    
    delay(1000);

    dwm_configure(&dwm1);

}

int test_counter = 0;
uint8_t device_id[4] = {0};
uint8_t device_id_low[2] = {0};
int txfrs_error_count = 0;
uint8_t sys_cfg[4] = {0};

uint8_t clear_tx[5] = {0};

void loop(){

    test_counter++;
	dwm_read_reg(&dwm1, 0x00, device_id, 4);
	dwm_read_reg_sub(&dwm1, 0x40, 0x02, device_id_low, 2);
	// dwm_basic_transmit(&dwm1);
	// delay(10);
    printRegHex(device_id,RX_BUFFER_SIZE,"Received Data:");


    uint8_t sys_event_status_reg[5] = {0};
    uint8_t rx_frame_info_reg[4] = {0};
    
    // uint8_t rx_buffer[RX_BUFFER_SIZE] = {0}; // tflen = 8  top2 crc bits can be ignored here



    //Transmit
	// dwm_read_reg(&dwm1, 0x0f, sys_event_status_reg, 5);

    // if(!(sys_event_status_reg[0]&0x80)){
    // 	txfrs_error_count++;
    // }

    // clear_tx[0] = 0xF0;
    // dwm_write_reg(&dwm1, 0x0F, clear_tx, 5);
        
    //Receive Data
    // if(!dwm_receive(&dwm1,rx_buffer,RX_BUFFER_SIZE)){
    //     Serial.println("Message not received");
    // }
    // printRegHex(rx_buffer,RX_BUFFER_SIZE,"Received Data:");




    // Serial.print("Count: ");
    // Serial.println(test_counter);
    // printRegHex(device_id,4,"Device ID");
    // printRegHex(device_id_low,2,"Device ID Low");
    // Serial.print("Tx Error Count: ");
    // Serial.println(txfrs_error_count);
    // printRegHex(sys_event_status_reg,5,"System Even Status Reg");

    // delay(10);






}
