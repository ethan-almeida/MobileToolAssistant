///*
// * dwm1000.c
// *
// *  Created on: Feb 16, 2026
// *      Author: Aneek Mubarak
// */
//
//
#include "dwm1000.h"
#include "main.h"
#include "stm32f4xx_hal.h"

#define MSG_TYPE_POLL 0xA1
#define MSG_TYPE_RESPONSE 0xB2
#define MSG_TYPE_FINAL 0xA2



extern SPI_HandleTypeDef hspi1;

static robot_state_t robot_state = ROBOT_SEND_POLL;
static uint8_t retries = 0;
#define MAX_RETRIES 3

#define ANTENNA_DELAY 32000ULL



void dwm_reset(DWM_Module *module) {
    HAL_GPIO_WritePin(module->reset_port, module->reset_pin, GPIO_PIN_RESET);
//    HAL_Delay(1);  // 1 ms minimum
    osDelay(1);

    // Release  to Hi-Z because open-drain
    HAL_GPIO_WritePin(module->reset_port, module->reset_pin, GPIO_PIN_SET);
//    HAL_Delay(10); // Allow DW1000 to boot
    osDelay(1);
}


//void dwm_read_reg(DWM_Module *module, uint8_t reg_id, uint8_t *data, uint16_t len) {
//    uint8_t tx[1 + len];
//    uint8_t rx[1 + len];
//
//    tx[0] = reg_id & 0x3F;
//
//    memset(&tx[1], 0, len);
//
//    HAL_GPIO_WritePin(module->cs_port, module->cs_pin, GPIO_PIN_RESET);
//    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 1 + len, HAL_MAX_DELAY);
//    HAL_GPIO_WritePin(module->cs_port, module->cs_pin, GPIO_PIN_SET);
//
//    memcpy(data, &rx[1], len);
//}

void dwm_read_reg(DWM_Module *module, uint8_t reg_id, uint8_t *data, uint16_t len)
{
    uint8_t header = reg_id & 0x3F;

    HAL_GPIO_WritePin(module->cs_port, module->cs_pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi1, &header, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, data, len, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(module->cs_port, module->cs_pin, GPIO_PIN_SET);
}






void dwm_write_reg(DWM_Module *module, uint8_t reg_id, uint8_t *data, uint8_t len) {
    uint8_t tx[1 + len];
    tx[0] = 0x80 | (reg_id & 0x3F);

    memcpy(&tx[1], data, len);

    HAL_GPIO_WritePin(module->cs_port, module->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tx, 1 + len, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(module->cs_port, module->cs_pin, GPIO_PIN_SET);
}



void dwm_write_reg_sub(DWM_Module *module, uint8_t reg_id, uint16_t subaddr, uint8_t *data, uint16_t len) {

	uint8_t header[3];
    uint8_t header_len = 1;

    //first header byte - write + reg ID
    header[0] = 0x80 | (reg_id & 0x3F);

    if (subaddr != 0xFFFF) {  // sentinel = no subaddr
        header[0] |= 0x40;    // subaddress flag

        header[1] = subaddr & 0x7F;
        header_len = 2;

        if (subaddr > 0x7F) {
            header[1] |= 0x80;
            header[2] = (subaddr >> 7) & 0xFF;
            header_len = 3;
        }
    }

    HAL_GPIO_WritePin(module->cs_port, module->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, header, header_len, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, data, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(module->cs_port, module->cs_pin, GPIO_PIN_SET);
}


void dwm_read_reg_sub(DWM_Module *module,uint8_t reg_id, uint16_t subaddr, uint8_t *data, uint16_t len) {

	uint8_t header[3];
    uint8_t header_len = 1;

    header[0] = reg_id & 0x3F;

    if (subaddr != 0xFFFF) {
        header[0] |= 0x40;

        header[1] = subaddr & 0x7F;
        header_len = 2;

        if (subaddr > 0x7F) {
            header[1] |= 0x80;
            header[2] = subaddr >> 7;
            header_len = 3;
        }
    }

    //combine header + dummy
    uint8_t tx[header_len + len];
    uint8_t rx[header_len + len];

    memcpy(tx, header, header_len);
    memset(tx + header_len, 0, len);

    HAL_GPIO_WritePin(module->cs_port, module->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, header_len + len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(module->cs_port, module->cs_pin, GPIO_PIN_SET);

    memcpy(data, rx + header_len, len);
}

void dwm_configure(DWM_Module* module){

	//1. AGC Tune1 - 2 octets
	uint8_t current_agc1_config[2] = {0};
	dwm_read_reg_sub(module,0x23,0x04,current_agc1_config,2);

	current_agc1_config[0] = 0x70;
	current_agc1_config[1] = 0x88;

	dwm_write_reg_sub(module, 0x23,0x04,current_agc1_config,2);
	dwm_read_reg_sub(module, 0x23,0x04,current_agc1_config,2);


	//2.AGC Tune2 (4 oct)
	uint8_t new_agc2_config[4] = {0x07,0xa9,0x02,0x25};
	dwm_write_reg_sub(module, 0x23,0x0c,new_agc2_config,4);

	//	//3. DRX_Tune_2 (4 oct)
//	uint8_t new_drx2_config[4] = {0x2d,0x00,0x1a,0x31};
    uint8_t new_drx2_config[4] = {0x9a,0x00,0x1a,0x35};

	dwm_write_reg_sub(module, 0x27,0x08,new_drx2_config,4);

	//4. NTM
	uint8_t lde_cfg_1[1] = {0};
	dwm_read_reg_sub(module, 0x2E,0x0806,lde_cfg_1,1);
	lde_cfg_1[0] = (lde_cfg_1[0] & ~0x1F) | (0x0D & 0x1F);
	dwm_write_reg_sub(module, 0x2E,0x0806,lde_cfg_1,1);

//	5.LDE Config 2
	uint8_t lde_cfg_2[2] = {0x07,0x16};
	dwm_write_reg_sub(module, 0x2E,0x1806,lde_cfg_2,2);

//	//6. Tx Power
	uint8_t tx_power_ctrl[4] ={0x48,0x28,0x08,0x0e};
	dwm_write_reg_sub(module, 0x1E,0x00,tx_power_ctrl,4);


//	7. RF_TXCTRL (3 oct)
//	 DATASHEET_UNCLEAR (refer pg 153)
//	 The data sheet says set 24 bits,  last oct is reserved and set to 0x00 but when reading it has 0xde, might have to set to 0x00 of issues
	uint8_t rf_txctrl[3] = {0xe3,0x3f,0x1e};
	dwm_write_reg_sub(module, 0x28,0x0c,rf_txctrl,3);

//	8.TC_PGDELAY (1 oct)
	uint8_t tc_pgdelay[1] = {0xc0};
	dwm_write_reg_sub(module, 0x2A,0x0b,tc_pgdelay,1);

//	9.PLL_TUNE (1 oct)
	uint8_t fs_pll_tune[1] = {0xbe};
	dwm_write_reg_sub(module, 0x2b,0x0b,fs_pll_tune,1);


//	10. LDE_LOAD
	//NOTE: refer page 176 for ldeload instruction if waking up from sleep/deep sleep

	//L1 - write to PMSC Control0 lower 16 bits
	uint8_t pmsc_ctrl_0_lower_2_oct[2] = {0x01,0x03};
	dwm_write_reg_sub(module, 0x36, 0x00, pmsc_ctrl_0_lower_2_oct, 2);

	//L2 - set OTP control LDELOAD bit (write entire reg)
	uint8_t otp_control[2] = {0x00,0x80};
	dwm_write_reg_sub(module, 0x2d, 0x06, otp_control, 2);

	//Wait 150us
//	HAL_Delay(1);
    osDelay(1);


	// L-3
	uint8_t pmsc_ctrl_0_lower_2_oct_L3[2] = {0x00,0x02};
	dwm_write_reg_sub(module, 0x36, 0x00, pmsc_ctrl_0_lower_2_oct_L3, 2);


	// 10.LDO TUNE
		//NOTE: can be skipped ig

	// ADDDITIONAL - (not part of the recommended configs)

	//To fix the CLKPLL_LL bit not locking issue in reg 0x0f, the PLLDT bit of 0x24 must be set

	  uint8_t ec_ctrl_reg[4] = {0};
	  dwm_read_reg_sub(module, 0x24,0x00,ec_ctrl_reg,4);
	  ec_ctrl_reg[0] = ec_ctrl_reg[0]|0x04;
	  dwm_write_reg_sub(module, 0x24,0x00,ec_ctrl_reg,4);

}

void dwm_basic_transmit(DWM_Module* module){
	/*
	 * 1. Write data in TX_BUFFER
	 * 2. Set frame len + other details in TX_FCTRL
	 * 3. Initialte using TXSTRT bit in SysCtrlReg (0x0D)
	 *
	 * NOTE: Cannot read from TxBuffer
	 * WARNING: reafing TX_FCTRL also reads TX_BUFFER, so dont read it during actove tx
	 *
	 * TX_FCTRL
	 * 		-TFLEN: frame len, payload length + 2 CRC bytes
	 * 		-TFLE: extended mode - not needed
	 * 		-TXBR: bit rate, default = 6.8Mbps
	 * 		-TR: Transmit bit rate ( ig its not needed) its set to 1 by def DATASHEET_UNCLEAR
	 * 		-TXPRF: Pulse repetion freq (default of 16Mhz is ok, must be grater than 4Mhz)
	 * 		-TXPSR: preamble length
	 * 		-NOTE - Modify premable length (TXPSR = 0b10, PE = 0b00) for preamble length of 1024
	 * 		-NOTE - Receiver must match this
	 *
	 */



	// WRITE DATA TO TX BUFFER
	uint8_t tx_data[6] ={0xAA,0xBB,0xCC,0xDD,0xEE,0xFF};
//	uint8_t tx_data[4] ={0xCA,0xAC,0x09,0xEF};
	//0xFFEEDDCCBBAA

	dwm_write_reg(module, 0x09, tx_data, 6);

	//SET FRAME LENGTH AND PREAMBLE LENGTH
	uint8_t tx_frame_control[5] = {0};
	dwm_read_reg(module, 0x08, tx_frame_control, 5);
	// fr_len = 4 + 2 +2  =
	// Set frame length
	tx_frame_control[0] &= 0x80;
	tx_frame_control[0] |= 8;
	// Set TXPSR=10, PE=00
	tx_frame_control[2] &= ~(0x3C);
	tx_frame_control[2] |= (0b10 << 2);

    // Set bitrate to 850kbps (Comment the blwow lines for default of 6.8Mbps)
    tx_frame_control[1] &= 0x9f; //clear
    tx_frame_control[1] |= 0x20; //set to 850kpbs


	dwm_write_reg(module, 0x08, tx_frame_control, 5);


	//TRASMIT
	uint8_t sys_ctrl[4] = {0};
	dwm_read_reg(module, 0x0D, sys_ctrl, 4);
	sys_ctrl[0] |= (1 << 1);
	dwm_write_reg(module, 0x0D, sys_ctrl, 4);

}

/*
    buffer: buffer for message , use expected size (only payload no CRC)
    len: len of buffer array (octs)
*/
bool dwm_receive(DWM_Module* module, uint8_t* buffer, uint16_t len){

    uint8_t sys_event_status_reg[5] = {0};
    uint8_t rx_frame_info_reg[4] = {0};
    // uint8_t rx_buffer[6] = {0}; // tflen = 6 but the top2 crc bits can be ignored here
    uint8_t sys_ctrl_reg[4] = {0};

    int tflen = 0;


    // ENABLE Rx
//    dwm_read_reg(module, 0x0D, sys_ctrl_reg, 4);
//    sys_ctrl_reg[1] |= (1 << 0); // RXENAB
//
//    dwm_write_reg(module, 0x0D, sys_ctrl_reg, 4);

    // now dw1000 is waiting for preamble
    // Add timeouts here ig
    int tries = 300;
    for (int i=0;i<=tries;i++){
        dwm_read_reg(module, 0x0F, sys_event_status_reg, 5);

        if(sys_event_status_reg[1]&0x20){ // RXDFR is high
            dwm_read_reg(module, 0x10,rx_frame_info_reg,4);
            tflen = rx_frame_info_reg[0]&0x7f; //tflen mask
            dwm_read_reg(module, 0x11,buffer,tflen-2); // no need CRC bits --> write data to buffer
            // PROBLEM: can corrupt mem if buff too small
            // Better to come up with a standard tflen across all DWMs and set to max.

            dwm_write_reg(module, 0x0F, sys_event_status_reg, 5); // clear all status bits
            return true;
        }

//        HAL_Delay(1);
        osDelay(1);

    }

    return false;

}

//------------------
// Two Way Ranging
//-------------------
// use 0x17 for TX_time and 0x15 for Rx_time
uint64_t read_timestamp(DWM_Module* module,uint8_t reg)
{
    uint8_t ts[5];
    dwm_read_reg_sub(module, reg, 0x00, ts, 5);

    uint64_t value = 0;
    for (int i = 4; i >= 0; i--) {
        value = (value << 8) | ts[i];
    }

    return value;
}

uint64_t ts_diff(uint64_t a, uint64_t b)
{
    //handle wraparound
    const uint64_t MASK = ((uint64_t)1 << 40) - 1;
    return (a - b) & MASK;
}



int send_frame(DWM_Module* module, uint8_t* payload, uint8_t len)
{
    // write to TX buffer
    dwm_write_reg(module, 0x09, payload, len);

    // configure TX_FCTRL
    uint8_t tx_frame_control[5] = {0};
    dwm_read_reg(module, 0x08, tx_frame_control, 5);

    tx_frame_control[0] &= 0x80;
    tx_frame_control[0] |= (len + 2); // include CRC

    dwm_write_reg(module, 0x08, tx_frame_control, 5);

    // trigger TX
    uint8_t sys_ctrl[4] = {0};
    dwm_read_reg(module, 0x0D, sys_ctrl, 4);
    sys_ctrl[0] |= (1 << 1); // TXSTRT
    dwm_write_reg(module, 0x0D, sys_ctrl, 4);


    uint8_t sys_event_status_reg[5] = {0};

    osDelay(1);

    dwm_read_reg(module, 0x0f, sys_event_status_reg, 5);

    if(!(sys_event_status_reg[0]&0x80)){
    	return 0;
    }

    uint8_t clear[5] = {0};
    clear[0] = 0x80;  // TXFRS
    clear[1] = 0xF0;  // clear RXFCE, RXRFTO, RXPTO etc
    dwm_write_reg(module, 0x0F, clear, 5);

    return 1; // success
}


bool process_response(uint8_t *rx_buffer, uint16_t len, uint64_t *treply_out)
{
    if (len < 6)
        return false;

    //Check message type
    if (rx_buffer[0] != MSG_TYPE_RESPONSE)
        return false;

    //Reconstruct 40-bit little-endian timestamp
    uint64_t treply = 0;

    for (int i = 0; i < 5; i++)
    {
        treply |= ((uint64_t)rx_buffer[1 + i]) << (8 * i);
    }

    *treply_out = treply;

    return true;
}




void start_ranging(DWM_Module* module, uint64_t* distance, uint64_t* t_prop){

    static int receive = 0;
    static int sent = 0;
    uint64_t t_reply = 0; // delay time of the remote
    static uint64_t tx_timestamp = 0;
    static uint64_t rx_timestamp = 0;
    uint64_t  distance_cm = 0;


//    uint8_t sys_ctrl_reg[4] = {0};

    uint8_t rx_buff[6];
    uint8_t tx_buff[4] = {0xAA, 0xBB, 0xCC, 0xEE};

    if(!sent){
        sent = send_frame(module, tx_buff, 4);

        if(sent){

        	// save timestamp
            // ENABLE Rx
        	tx_timestamp = read_timestamp(module, 0x17);
//            dwm_read_reg(module, 0x0D, sys_ctrl_reg, 4);
            uint8_t sys_ctrl_reg[4] = {0};
            sys_ctrl_reg[1] |= (1 << 0); // RXENAB
            dwm_write_reg(module, 0x0D, sys_ctrl_reg, 4);
        }

    }
    else if(sent && !receive){
        if(dwm_receive(module, rx_buff, 6)){
            receive = 1;
            rx_timestamp = read_timestamp(module, 0x15);
            process_response(rx_buff,6, &t_reply);

        }
    }

    if(sent && receive){
    	// done
    	uint64_t t_rtt = ts_diff(rx_timestamp,tx_timestamp);

//    	uint64_t t_prop = 0.5*(t_rtt - t_reply);
//    	uint64_t t_prop = (t_rtt - t_reply) >> 1;
    	if (t_rtt > t_reply) {
    	    *t_prop = (t_rtt - t_reply) >> 1;

    	    if (*t_prop > ANTENNA_DELAY)
    	    {
    	        *t_prop -= ANTENNA_DELAY;
    	    }
//    	    distance_cm = (t_prop * 469176) / 1000000;
    	    distance_cm = (*t_prop * 469176ULL) / 1000000ULL;
    	    distance_cm = (*t_prop * 469176ULL) / 1000000ULL;
//    	    *distance = distance_cm;

    	    uint64_t offset = 360;
    	    *distance = distance_cm-offset;;

    	}


        // reset for next round
        sent = 0;
        receive = 0;
    }
}

