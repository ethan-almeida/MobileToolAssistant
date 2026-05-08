///*
// * dwm1000.c
// *
// *  Created on: Feb 16, 2026
// *      Author: Aneek Mubarak
// */
//
//
#include "core_esp8266_features.h"
#include <stdint.h>
#include "HardwareSerial.h"
#include "dw1000.h"
#include <cstdint>
#include "dwm_regs.h"

#define SPI_SPEED_MAX 20000000


#define MSG_TYPE_POLL 0xA1
#define MSG_TYPE_POLL_2 0xA2
#define MSG_TYPE_RESPONSE 0xB2
#define MSG_TYPE_FINAL 0xA3

#define DWT_TIME_UNITS (1.0 / (499.2e6 * 128.0))
#define US_TO_DWT(us) ((uint64_t)((us) / (DWT_TIME_UNITS * 1e6)))


//static robot_state_t robot_state = ROBOT_SEND_POLL;
static uint8_t retries = 0;
#define MAX_RETRIES 3

#define ANTENNA_DELAY 33008.6534  // NO SCRATCH
//#define ANTENNA_DELAY 0  // NO SCRATCH


// Reading for 7.94m with default config is 16,373cm  --> 33 000

//#define ANTENNA_DELAY 32835.0   // calibrated at 0.4m


static robot_state_t robot_state = RSTATE_SEND_POLL;
static uint32_t dw_spi_speed = 3000000; // 3 MHz safe init


void dwm_reset(DWM_Module *module) {
    // pinMode(module->reset_pin, OUTPUT_OPEN_DRAIN); // ensure open-drain

    // // Pull low to reset
    // digitalWrite(module->reset_pin, LOW);
    // delay(1); // TRST_OK minimum

    // // Release to high-impedance
    // digitalWrite(module->reset_pin, HIGH); // now acts as Hi-Z because pin is open-drain
    // delay(10); // allow DW1000 to boot

    pinMode(module->reset_pin, OUTPUT);
    digitalWrite(module->reset_pin, LOW);
    delay(2);

    // Release to high-Z
    pinMode(module->reset_pin, INPUT);
    delay(10);

}



void dwm_read_reg(DWM_Module *module, uint8_t reg_id, uint8_t *data, uint16_t len)
{
    uint8_t header = reg_id & 0x3F;

    SPI.beginTransaction(SPISettings(dw_spi_speed, MSBFIRST, SPI_MODE0));
    digitalWrite(module->cs_pin, LOW);

    SPI.transfer(header);

    for (uint16_t i = 0; i < len; i++) {
        data[i] = SPI.transfer(0x00);
    }

    digitalWrite(module->cs_pin, HIGH);
    SPI.endTransaction();
}





////
//void dwm_write_reg(DWM_Module *module, uint8_t reg_id, uint8_t *data, uint8_t len) {
//    uint8_t tx[1 + len];
//    tx[0] = 0x80 | (reg_id & 0x3F);
//
//    memcpy(&tx[1], data, len);
//
//    HAL_GPIO_WritePin(module->cs_port, module->cs_pin, GPIO_PIN_RESET);
//    HAL_SPI_Transmit(&hspi1, tx, 1 + len, HAL_MAX_DELAY);
//
//    HAL_GPIO_WritePin(module->cs_port, module->cs_pin, GPIO_PIN_SET);
//}


void dwm_write_reg(DWM_Module *module, uint8_t reg_id, uint8_t *data, uint16_t len) {

    uint8_t header = 0x80 | (reg_id & 0x3F);

    SPI.beginTransaction(SPISettings(dw_spi_speed, MSBFIRST, SPI_MODE0));
    digitalWrite(module->cs_pin, LOW);

    SPI.transfer(header);

    for (int i = 0; i < len; i++) {
        SPI.transfer(data[i]);
    }

    digitalWrite(module->cs_pin, HIGH);
    SPI.endTransaction();
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

    //start SPI 
    SPI.beginTransaction(SPISettings(dw_spi_speed, MSBFIRST, SPI_MODE0));
    //pull cs low
    digitalWrite(module->cs_pin, LOW);
    //send header
    for (int i = 0; i < header_len; i++) {
        SPI.transfer(header[i]);
    }
    //send data
    for (int i = 0; i < len; i++) {
        SPI.transfer(data[i]);
    }
    //release
    digitalWrite(module->cs_pin, HIGH);
    SPI.endTransaction();
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

    SPI.beginTransaction(SPISettings(dw_spi_speed, MSBFIRST, SPI_MODE0));
    digitalWrite(module->cs_pin, LOW);

    //send header only (recv dummy)
    for (int i = 0; i < header_len; i++) {
        SPI.transfer(header[i]);  // just send header
    }
    //read data
    for (int i = 0; i < len; i++) {
        data[i] = SPI.transfer(0x00); //send dummy 0x00 
    }
    //release
    digitalWrite(module->cs_pin, HIGH);
    SPI.endTransaction();
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
//    0x31 3B 00 6B
    // for PAC 8 -> 64Mhz Prf
    // uint8_t new_drx2_config[4] = {0x6b,0x00,0x3b,0x31};
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
	delay(1);

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

      delay(10);
      dw_spi_speed = SPI_SPEED_MAX; // switch to fast mode

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
    int tries = 60000;
    for (int i=0;i<=tries;i++){
        dwm_read_reg(module, 0x0F, sys_event_status_reg, 5);

        if(sys_event_status_reg[1]&0x20){ // RXDFR is high
            dwm_read_reg(module, 0x10,rx_frame_info_reg,4);
            tflen = rx_frame_info_reg[0]&0x7f; //tflen mask
            dwm_read_reg(module, 0x11,buffer,tflen-2); // no need CRC bits --> write data to buffer
            // PROBLEM: can corrupt mem if buff too small
            // Better to come up with a standard tflen across all DWMs and set to max.

//            dwm_write_reg(module, 0x0F, sys_event_status_reg, 5); // clear all status bits

            uint8_t clear[5] = {0};
            clear[1] = 0x20 | 0x40 | 0x04; // RXDFR | RXFCG | LDEDONE
            dwm_write_reg(module, 0x0F, clear, 5);

            return true;
        }

    }
    // force RX reset
    uint8_t sys_ctrl[4] = {0};
    sys_ctrl[0] |= (1 << 6); // TRXOFF
    dwm_write_reg(module, 0x0D, sys_ctrl, 4);

    // clear RX error flags only
    uint8_t clear[5] = {0};
    clear[1] = 0xF0; // RX errors
    dwm_write_reg(module, 0x0F, clear, 5);

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

//    // NEW
//    // Set PRF to 64Mhz (comment out for 16Mhz)
//    tx_frame_control[1] &= 0b11001111;
//    tx_frame_control[1] |= 0b00100000;


    dwm_write_reg(module, 0x08, tx_frame_control, 5);

    // trigger TX
    uint8_t sys_ctrl[4] = {0};
    dwm_read_reg(module, 0x0D, sys_ctrl, 4);
    sys_ctrl[0] |= (1 << 1); // TXSTRT
    dwm_write_reg(module, 0x0D, sys_ctrl, 4);



    uint8_t sys_status[5];
    int timeout = 10000;

    while(timeout--)
    {
        dwm_read_reg(module, 0x0F, sys_status, 5);
        if(sys_status[0] & 0x80)  // TXFRS
            break;
    }

    if(!(sys_status[0] & 0x80))
        return 0;   // timeout

    uint8_t clear[5] = {0};
    clear[0] = 0x80;
    dwm_write_reg(module, 0x0F, clear, 5);



    return 1;
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


bool process_response_2(uint8_t *rx_buffer, uint16_t len, uint64_t *treply_out)
{
    if (len < 6)
        return false;

    //Check message type
    if (rx_buffer[0] != MSG_TYPE_FINAL)
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



bool robot_ranging_step(DWM_Module* module, uint64_t* distance_cm_out)
{
    static uint64_t t_tx_poll = 0;
    static uint64_t t_rx_resp = 0;
    static uint64_t t_reply_1 = 0;

    static uint64_t t_tx_poll_2 = 0;
    static uint64_t t_rx_resp_2 = 0;
    static uint64_t t_reply_2 = 0;


    uint8_t rx_buff[6];

    switch(robot_state)
    {
        //*******************************************
        case RSTATE_SEND_POLL:
        {
            // Try sending poll frame
            uint8_t poll_frame[6] = {MSG_TYPE_POLL, 0,0,0,0,0};

            if (!send_frame(module, poll_frame, 6)) {
                // send failed; retry a few times
                robot_state = RSTATE_ERROR_RECOVERY;
                return false;
            }

            // Save TX timestamp immediately
            t_tx_poll = read_timestamp(module, 0x17);

            // enable receiver
            uint8_t sys_ctrl_reg[4] = {0};
            sys_ctrl_reg[1] |= (1 << 0); // RXENAB
            dwm_write_reg(module, 0x0D, sys_ctrl_reg, 4);

            // Go to receiving state
            robot_state = RSTATE_WAIT_RESPONSE_1;


            break;
        }

        case RSTATE_WAIT_RESPONSE_1:
        {
            if (dwm_receive(module, rx_buff, 6)) {
            	if (!process_response(rx_buff, 6, &t_reply_1)) {
                    robot_state = RSTATE_ERROR_RECOVERY;
                    return false;
                }
                t_rx_resp = read_timestamp(module, 0x15);

                // ===== SEND POLL_2 IMMEDIATELY — DO NOT RETURN =====

                // Force IDLE first (critical after RX)
                uint8_t trxoff[4] = {0};
                trxoff[0] = (1 << 6);  // TRXOFF
                dwm_write_reg(module, 0x0D, trxoff, 4);

                // Clear all status
                uint8_t clearall[5] = {0};
                dwm_read_reg(module,0x0F,clearall,5);
                dwm_write_reg(module, 0x0F, clearall, 5);

                // Send Poll_2
                uint8_t poll_frame[6] = {MSG_TYPE_POLL_2, 0,0,0,0,0};
                if (!send_frame(module, poll_frame, 6)) {
                    robot_state = RSTATE_ERROR_RECOVERY;
                    return false;
                }

                t_tx_poll_2 = read_timestamp(module, 0x17);

                // Enable RX for Final
                uint8_t sys_ctrl_reg[4] = {0};
                sys_ctrl_reg[1] = (1 << 0); // RXENAB
                dwm_write_reg(module, 0x0D, sys_ctrl_reg, 4);

                robot_state = RSTATE_WAIT_RESPONSE_2;
            } else {
                robot_state = RSTATE_ERROR_RECOVERY;
            }
            break;
        }


        //**** NEW : FOR DS TWR stuff ********

        case RSTATE_SEND_POLL_2:
        {


            // Try sending poll frame
            uint8_t poll_frame[6] = {MSG_TYPE_POLL_2, 0,0,0,0,0};


            if (!send_frame(module, poll_frame, 6)) {
                // send failed; retry a few times
                robot_state = RSTATE_ERROR_RECOVERY;
                return false;
            }

            // Save TX timestamp immediately
            t_tx_poll_2 = read_timestamp(module, 0x17);


            // enable receiver to get the final response

            uint8_t clear[5] = {0};
            clear[0] = 0x80;
            dwm_write_reg(module, 0x0F, clear, 5);

            uint8_t sys_ctrl_reg[4] = {0};
            sys_ctrl_reg[1] |= (1 << 0); // RXENAB
            dwm_write_reg(module, 0x0D, sys_ctrl_reg, 4);

            // Go to receiving state
            robot_state = RSTATE_WAIT_RESPONSE_2;


            break;
        }

        //*****************************
        case RSTATE_WAIT_RESPONSE_2:
        {
        	// get the first responcse and record t_reply1
            // Try receive with timeout
            if (dwm_receive(module, rx_buff, 6)) {

                // Check response contents
                if (!process_response_2(rx_buff, 6, &t_reply_2)) {
                    // Bad response format
                    robot_state = RSTATE_ERROR_RECOVERY;
                    return false;
                }

                // Save RX time
                t_rx_resp_2 = read_timestamp(module, 0x15);

//                // Compute range
//                robot_state = RSTATE_COMPUTE;

                //compute
                robot_state = RSTATE_COMPUTE;
            }else{
            	robot_state = RSTATE_ERROR_RECOVERY;
            }
            break;
        }

        
        case RSTATE_COMPUTE:
        {
            uint64_t t_rtt_1  = ts_diff(t_rx_resp,   t_tx_poll);
            uint64_t t_rtt_2  = ts_diff(t_rx_resp_2, t_tx_poll_2);

            // DS-TWR: t_prop = (rtt1*rtt2 - reply1*reply2) / (rtt1+rtt2+reply1+reply2)
            double rtt1   = (double)t_rtt_1;
            double rtt2   = (double)t_rtt_2;
            double reply1 = (double)t_reply_1;
            double reply2 = (double)t_reply_2;

            double t_prop = (rtt1 * rtt2 - reply1 * reply2)
                          / (rtt1 + rtt2 + reply1 + reply2);


            t_prop -= ANTENNA_DELAY;
            if (t_prop < 0) {
                robot_state = RSTATE_ERROR_RECOVERY;
                return false;
            }

            double distance_m = t_prop * DWT_TIME_UNITS * 299792458.0;
            *distance_cm_out = (uint64_t)(distance_m * 100.0);

            robot_state = RSTATE_SEND_POLL;
            return true;
        }

        // --------------------------------------------------------------------
        case RSTATE_ERROR_RECOVERY:
        {
            // Abort any pending operation
            uint8_t sys_ctrl[4] = {0};
            sys_ctrl[0] |= (1 << 6);  // TRXOFF
            dwm_write_reg(module, 0x0D, sys_ctrl, 4);

            // Clear all status (including HPDWARN)
            uint8_t clear[5] = {0};
            dwm_read_reg(module, 0x0F, clear, 5);
            dwm_write_reg(module, 0x0F, clear, 5);
            // Reset state, maybe increment retry counters
            robot_state = RSTATE_SEND_POLL;
            return false;
        }
    }

    return false;  // no new valid distance yet
}

