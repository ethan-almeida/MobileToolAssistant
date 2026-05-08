#include "dw1000.h"

#define RX_BUFFER_SIZE 2  // Only payload (no CRC)

DWM_Module dwm1 = { D8, D4 }; // CS=D8, RESET=D4


void setup() {
    Serial.begin(115200);
    pinMode(dwm1.cs_pin, OUTPUT);
    pinMode(dwm1.reset_pin, OUTPUT);
    digitalWrite(dwm1.cs_pin, HIGH);

    // to allow time to load the serial monitor
    int countdown=5;
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

uint8_t rx_buffer[127] = {0};

uint8_t send_payload[4] = {0x0A, 0x0B, 0x0F, 0x0F};

void loop(){

    // Remote_Timestamps ts = {0,0,0,0}; // reinit every session

    test_counter++;
	dwm_read_reg(&dwm1, 0x00, device_id, 4);
    printRegHex(device_id,4,"device_id");

    // run_remote(&dwm1);

    // send_payload[0]++;
    // send_frame(&dwm1,send_payload,4);


    // if(dwm_receive(&dwm1,rx_buffer,127)){
    //     printRegHex(rx_buffer,1,"Rx Buffer");
    // }else{
    //     Serial.println("Waiting...");
    // }



}
