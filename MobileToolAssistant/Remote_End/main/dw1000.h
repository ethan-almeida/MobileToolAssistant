#ifndef DWM1000_ESP_H
#define DWM1000_ESP_H

#include <Arduino.h>
#include <SPI.h>

typedef struct {
    uint8_t cs_pin;
    uint8_t reset_pin;
} DWM_Module;

typedef enum {
    REMOTE_WAIT_M1,
    REMOTE_SEND_M2,
    REMOTE_WAIT_TX_DONE,
    REMOTE_WAIT_M3,
    REMOTE_SEND_M4,
    REMOTE_WAIT_TX_DONE_FINAL

} remote_state_t;


void dwm_reset(DWM_Module *module);

void dwm_read_reg(DWM_Module *module, uint8_t reg_id, uint8_t *data, uint16_t len);
void dwm_write_reg(DWM_Module *module, uint8_t reg_id, uint8_t *data, uint16_t len);

void dwm_read_reg_sub(DWM_Module *module, uint8_t reg_id, uint16_t subaddr, uint8_t *data, uint16_t len);
void dwm_write_reg_sub(DWM_Module *module, uint8_t reg_id, uint16_t subaddr, uint8_t *data, uint16_t len);

void dwm_configure(DWM_Module *module);
void dwm_basic_transmit(DWM_Module *module);

void printRegHex(uint8_t* data, size_t len, const char* info);

bool dwm_receive(DWM_Module* module, uint8_t* buffer, uint16_t len);

// Main state machine for DS TWR
void run(DWM_Module* module, volatile bool* isr_flag);


int send_frame(DWM_Module* module, uint8_t* payload, uint8_t len);

bool process_response(uint8_t *rx_buffer, uint16_t len, uint8_t msg_type);
void remote_reset_session();


#endif