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
#include <string.h>
#include <stdbool.h>
#include <math.h>

//DWM Module Structure
typedef struct {
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    GPIO_TypeDef *reset_port;
    uint16_t reset_pin;
    uint64_t antenna_delay;
    int offset_cm;
} DWM_Module;


typedef enum  {
    RSTATE_IDLE,
    RSTATE_SEND_POLL,
	RSTATE_WAIT_RESPONSE_1,

	RSTATE_COMPUTE,
    RSTATE_ERROR_RECOVERY,

	RSTATE_SEND_POLL_2,
	RSTATE_WAIT_RESPONSE_2
} robot_state_t;




typedef struct {
    float distance_cm;// Distance from robot center to remote
    float angle_deg;// 0=right,90=forward,180=left
    float x_cm;// Lateral offset (+ = right, - = left)
    float y_cm;// Forward distance
    uint8_t valid;// 1 = valid measurement, 0 = invalid
} RemotePosition;




#define UWB_BASELINE_CM 29.0f //28
#define RAD_TO_DEG 57.2957795f

RemotePosition calculate_remote_position(float dist_left_cm, float dist_right_cm);


void dwm_reset(DWM_Module *module);

void dwm_read_reg(DWM_Module *module, uint8_t reg_id, uint8_t *data, uint16_t len);
void dwm_write_reg(DWM_Module *module, uint8_t reg_id, uint8_t *data, uint8_t len);
void dwm_write_reg_sub(DWM_Module *module, uint8_t reg_id, uint16_t subaddr, uint8_t *data, uint16_t len);
void dwm_read_reg_sub(DWM_Module *module, uint8_t reg_id, uint16_t subaddr, uint8_t *data, uint16_t len);

void dwm_configure(DWM_Module* module);
uint64_t ts_diff(uint64_t a, uint64_t b);

bool dwm_receive(DWM_Module* module, uint8_t* buffer, uint16_t len);
int send_frame(DWM_Module* module, uint8_t* payload, uint8_t len);

bool process_response(uint8_t *rx_buffer, uint16_t len, uint64_t *treply_out, bool* standby_flag);
bool process_response_2(uint8_t *rx_buffer, uint16_t len, uint64_t *treply_out);

// main state machine for robot end
bool robot_ranging_step(DWM_Module* module, int* distance_cm_out, bool* standby_flag);


int snap_to_45(int angle);



#endif /* INC_DWM1000_H_ */
