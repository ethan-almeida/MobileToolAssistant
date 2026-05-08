///*
// * dwm1000.h
// *
// *  Created on: Feb 16, 2026
// *      Author: Aneek Mubarak
// */
//
#ifndef INC_DWM1000_H_
#define INC_DWM1000_H_

#include "main.h"
#include <stdint.h>
#include <string.h>  // --> for memcpy
#include <stdbool.h>

//DWM Module Structure
typedef struct {
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    GPIO_TypeDef *reset_port;
    uint16_t reset_pin;
} DWM_Module;

typedef enum {
    ROBOT_SEND_POLL,
    ROBOT_WAIT_RESP,
    ROBOT_SEND_FINAL,
    ROBOT_WAIT_ACK
} robot_state_t;


void dwm_reset(DWM_Module *module);

/**
 * @brief Read a register from the DWM module.
 * @param module Pointer to DWM_Module struct
 * @param reg_id Register ID to read
 * @param data Buffer to store the read data
 * @param len Number of bytes to read
 */
void dwm_read_reg(DWM_Module *module, uint8_t reg_id, uint8_t *data, uint16_t len);

/**
 * @brief Write data to a register of the DWM module.
 * @param module Pointer to DWM_Module struct
 * @param reg_id Register ID to write
 * @param data Data buffer to write
 * @param len Number of bytes to write
 */
void dwm_write_reg(DWM_Module *module, uint8_t reg_id, uint8_t *data, uint8_t len);


/**
 *  read sub registers and wrote to buffer
 */

void dwm_write_reg_sub(DWM_Module *module, uint8_t reg_id, uint16_t subaddr, uint8_t *data, uint16_t len);
void dwm_read_reg_sub(DWM_Module *module, uint8_t reg_id, uint16_t subaddr, uint8_t *data, uint16_t len);



/*
 * Configure the DWM according to the datasheetss recommended mods to the default configuration
 * NOTE: DATASHEET_UNCLEAR (refer pg 153) for RF_TXCTRL
 *
 */
void dwm_configure(DWM_Module* module);


void dwm_basic_transmit(DWM_Module* module);

///uint64_t robot_receive_response(DWM_Module* module);
//uint64_t robot_send_final(DWM_Module* module);

//bool dwm_wait_for_response(DWM_Module* module,uint8_t* buffer,uint16_t max_len,uint32_t timeout_ms,uint64_t* t4_out);

//uint64_t dwm_get_distance(DWM_Module* module);
bool dwm_receive(DWM_Module* module, uint8_t* buffer, uint16_t len);

int send_frame(DWM_Module* module, uint8_t* payload, uint8_t len);
void robot_uwb_task(DWM_Module* module);



bool process_response(uint8_t *rx_buffer, uint16_t len, uint64_t *treply_out);

void start_ranging(DWM_Module* module, uint64_t* t_reply_p,uint64_t* t_prop);


uint64_t ts_diff(uint64_t a, uint64_t b);

#endif /* INC_DWM1000_H_ */
