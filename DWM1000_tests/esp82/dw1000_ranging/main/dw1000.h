#ifndef DWM1000_ESP_H
#define DWM1000_ESP_H

#include <Arduino.h>
#include <SPI.h>

typedef struct {
    uint8_t cs_pin;
    uint8_t reset_pin;
} DWM_Module;



// typedef struct {
//     uint64_t t2; //poll receive
//     uint64_t t3; //response
//     uint64_t t5; // second receive
//     uint64_t t6; // final response
// } Remote_Timestamps;


// typedef struct {
//     uint8_t type;   // 0x01
// } poll_msg_t;


// typedef struct {
//     uint8_t type;   // 0x02
//     uint8_t t2[5];
//     uint8_t t3[5];
// } response_msg_t;

// typedef struct {
//     uint8_t type;   // 0x03
//     uint8_t t1[5];
//     uint8_t t4[5];
//     uint8_t t5[5];
// } final_msg_t;

// typedef struct {
//     uint64_t t2;
//     uint64_t t3;
//     uint64_t t6;
//     uint64_t t1;
//     uint64_t t4;
//     uint64_t t5;
// } remote_session_t;

typedef enum {
    REMOTE_WAIT_M1 = 0,
    REMOTE_SEND_M2,
    REMOTE_WAIT_M3,
    REMOTE_SEND_M4
} remote_state_t;

extern  int retry_counter;

void dwm_reset(DWM_Module *module);

void dwm_read_reg(DWM_Module *module, uint8_t reg_id, uint8_t *data, uint16_t len);
void dwm_write_reg(DWM_Module *module, uint8_t reg_id, uint8_t *data, uint16_t len);

void dwm_read_reg_sub(DWM_Module *module, uint8_t reg_id, uint16_t subaddr, uint8_t *data, uint16_t len);
void dwm_write_reg_sub(DWM_Module *module, uint8_t reg_id, uint16_t subaddr, uint8_t *data, uint16_t len);

void dwm_configure(DWM_Module *module);
void dwm_basic_transmit(DWM_Module *module);

void printRegHex(uint8_t* data, size_t len, const char* info);

bool dwm_receive(DWM_Module* module, uint8_t* buffer, uint16_t len);
// uint64_t remote_receive_poll(DWM_Module* module);
// uint64_t remote_send_response(DWM_Module* module);
// uint64_t remote_receive_final(DWM_Module* module);

// uint64_t start_ranging(DWM_Module* module);

void run_remote(DWM_Module* module);
int send_frame(DWM_Module* module, uint8_t* payload, uint8_t len);



#endif
