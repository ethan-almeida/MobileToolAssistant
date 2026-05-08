#include "dw1000.h"
#include <ESP8266WiFi.h>


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
    
void loop(){

    run(&dwm1,&dw_event);   

}

