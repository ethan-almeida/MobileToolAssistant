#include "dw1000.h"
#include <ESP8266WiFi.h>
#include <SPI.h>


#define RX_BUFFER_SIZE 2  // Only payload (no CRC)

DWM_Module dwm1 = { D8, D4 }; // CS=D8, RESET=D4


void setup() {

    WiFi.persistent(false);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin();
    delay(1);

    Serial.begin(115200);
    pinMode(dwm1.cs_pin, OUTPUT);
    pinMode(dwm1.reset_pin, OUTPUT);
    digitalWrite(dwm1.cs_pin, HIGH);
    

    // to allow time to load the serial monitor
    int countdown=5;
    for (int i = countdown; i >= 0 ; i--){
        Serial.println(i);
        delay(10);
    }
    SPI.begin();
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);

    dwm_reset(&dwm1);
    
    delay(1000);

    dwm_configure(&dwm1);

    delay(1000);



    // enable receiver
    // uint8_t sys_ctrl[4] = {0};
    // dwm_read_reg(&dwm1, 0x0D, sys_ctrl, 4);
    // sys_ctrl[1] |= (1 << 0);   // RXENAB
    // dwm_write_reg(&dwm1, 0x0D, sys_ctrl, 4);


}
    
int test_counter = 0;
uint8_t device_id[4] = {0};
uint8_t device_id_low[2] = {0};
int txfrs_error_count = 0;
uint8_t sys_cfg[4] = {0};

uint8_t clear_tx[5] = {0};

uint8_t rx_buffer[6] = {0};

uint8_t send_payload[4] = {0x0A, 0x0B, 0x0F, 0x0F};
uint8_t sys_status[5] = {0};

uint64_t distance_cm = 0;

bool uwb_return;

int tr_count = 0;
uint64_t acc = 0;

void loop(){


    // test_counter++;
	// dwm_read_reg(&dwm1, 0x00, device_id, 4);
    // printRegHex(device_id,4,"device_id");

    uwb_return = robot_ranging_step(&dwm1,&distance_cm);

    if(uwb_return){
        Serial.println(distance_cm);
        acc += distance_cm;
        tr_count++;

    }

    if(tr_count >= 100){
        uint64_t avg = acc/100;
        Serial.print("Average");
        Serial.println(avg);

        acc = 0;
        tr_count = 0;

    }

}

    // RX interrupt
//     ↓
    // Read RX timestamp
//     ↓
// Compute T_tx = T_rx + delay
//     ↓
// Zero lower 9 bits
//     ↓
// Prepare response payload (insert timestamps)
//     ↓
// Write TX buffer
//     ↓
// Write frame length (TX_FCTRL)
//     ↓
// Write DX_TIME (0x0A)
//     ↓
// Set TXDLYS + TXSTRT