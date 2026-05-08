/*
 * dwm1000_dirver.h
 *
 *  Created on: Feb 12, 2026
 *      Author: Aneek Mubarak
 */

#ifndef INC_DWM1000_DRIVER_H_
#define INC_DWM1000_DRIVER_H_


#include "stm32f4xx_hal.h"

// Public API
void dwm_reset(void);

uint32_t dwm_read_reg_32(uint8_t reg_id);
void dwm_write_reg_32(uint8_t reg_id, uint32_t value);

uint32_t dwm_read_sub_reg_32(uint8_t reg, uint8_t sub);
void dwm_write_sub_reg_32(uint8_t reg, uint8_t sub, uint32_t value);

void dwm_tx_test(void);

void modify_default_config(void);

// Register defines
#define DEV_ID     0x00
#define SYS_CONFIG 0x04
#define SYS_STATUS 0x0F



#endif /* INC_DWM1000_DRIVER_H_ */
