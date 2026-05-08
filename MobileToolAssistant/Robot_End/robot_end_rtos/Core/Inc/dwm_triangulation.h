/*
 * dwm_triangulation.h
 *
 *  Created on: Mar 15, 2026
 *      Author: Aneek Mubarak
 */

#ifndef INC_DWM_TRIANGULATION_H_
#define INC_DWM_TRIANGULATION_H_

#include "main.h"
#include "dwm1000.h"
#include <math.h>


//for spi device id tests for checking connection
extern uint8_t dwm1_device_id[4];
extern uint8_t dwm2_device_id[4];

/**
 * Configures GPIO pins, resets both DWM modules, and performs initial 
 * register configuration via SPI.
 * @param hspi Pointer to the SPI handle used for communication.
 */void DWM_Triangulation_Init(SPI_HandleTypeDef *hspi);

/**
 * Alternates ranging between left and right anchors, averages results, 
 * and calculates a new position calculation once enough samples are collected.
 */
void DWM_Triangulation_TaskStep(void);

/**
 * Retrieves the most recently calculated remote position.
 * @param pos Pointer to a RemotePosition struct to be populated.
 * @return true if a new valid position was available, false otherwise.
 */
bool DWM_Triangulation_GetPosition(RemotePosition *pos);

//Current boolean status of the standby flag.
bool get_standby_state();



#endif /* INC_DWM_TRIANGULATION_H_ */
