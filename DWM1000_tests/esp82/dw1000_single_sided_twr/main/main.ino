#include "dw1000.h"
#include <ESP8266WiFi.h>

#define RX_BUFFER_SIZE 2  // Only payload (no CRC)

DWM_Module dwm1 = { D8, D4 }; // CS=D8, RESET=D4

volatile bool dw_event = false;

void IRAM_ATTR dwm_isr() {
    dw_event = true;   
}

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
    
    pinMode(D1, INPUT); // IRQ
    attachInterrupt(digitalPinToInterrupt(D1), dwm_isr, RISING);


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

    delay(2000);



    // enable receiver
    uint8_t sys_ctrl[4] = {0};
    dwm_read_reg(&dwm1, 0x0D, sys_ctrl, 4);
    sys_ctrl[1] |= (1 << 0);   // RXENAB
    dwm_write_reg(&dwm1, 0x0D, sys_ctrl, 4);

    // Add MRXDFR and MTXFRS Interrupt -  receiver data frame ready and message transmitted
    uint8_t sys_event_mask_reg[4] = {0};
    dwm_read_reg(&dwm1,0x0E,sys_event_mask_reg, 4);
    sys_event_mask_reg[1] = sys_event_mask_reg[1]|0x20; // MRXDFR Bit
    sys_event_mask_reg[0] = sys_event_mask_reg[0]|0x80; // MTXFRS Bit
    dwm_write_reg(&dwm1,0x0E,sys_event_mask_reg,4);


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

void loop(){


    // test_counter++;
	// dwm_read_reg(&dwm1, 0x00, device_id, 4);
    // printRegHex(device_id,4,"device_id");
    // Main Remote Function
    run(&dwm1,&dw_event);
    // Serial.println(test_counter);



    // int send_frame(DWM_Module* module, uint8_t* payload, uint8_t len)
    // send_payload[0]++;
    // if(send_frame(&dwm1,send_payload,4)){
    //     Serial.println("Sent Frame");
    // }else{
    //     Serial.println("Frame not Sent");
    // }


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