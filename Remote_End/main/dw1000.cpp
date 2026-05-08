#include "core_esp8266_features.h"
#include <stdint.h>
#include "HardwareSerial.h"
#include "dw1000.h"
#include <cstdint>

#define SPI_SPEED_MAX 20000000  //20Mhz

#define MSG_TYPE_POLL 0xA1
#define MSG_TYPE_POLL_2 0xA2

#define MSG_TYPE_RESPONSE 0xB2
#define MSG_TYPE_FINAL 0xA3

#define DWT_TIME_UNITS (1.0 / (499.2e6 * 128.0))
#define US_TO_DWT(us) ((uint64_t)((us) / (DWT_TIME_UNITS * 1e6)))
#define REPLY_DELAY_DTU 64000000ULL

#define FRAME_LEN_MAX 127 // for received frame
static uint8_t rx_buffer[FRAME_LEN_MAX];
static uint32_t status_reg = 0;
static uint16_t frame_len = 0;

static remote_state_t remote_state = REMOTE_WAIT_M1;
static uint64_t t_rx_m1 = 0;
static uint64_t t_tx_m2 = 0;

static uint64_t t_rx_m3 = 0;
static uint64_t t_tx_m4 = 0;


static uint64_t last_timestamp = 0;


static int retry_counter = 0;
#define MAX_RETRIES 20000



static uint32_t dw_spi_speed = 3000000; // 3 MHz safe init

//// Reset DWM1000 to high impedence state/pullup
void dwm_reset(DWM_Module *module) {

    pinMode(module->reset_pin, OUTPUT);
    digitalWrite(module->reset_pin, LOW);
    delay(2);

    // Release to high-Z
    pinMode(module->reset_pin, INPUT);
    delay(10);

}

// Read basic DWM1000 register of len number of bytes
/**
 * @param reg_id The 6-bit register ID.
 * @param data Buffer to store the read bytes.
 * @param len Number of bytes to read.
 */
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


// Read from DWM1000 sub register of len number of bytes
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

// Write to basic DWM1000 register of len number of bytes
/**
 * @param reg_id The 6-bit register ID.
 * @param data Buffer containing data to write.
 * @param len Number of bytes to write.
 */
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

// Write  to DWM1000 sub register of len number of bytes
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


// print register values in serial monitor in hex
void printRegHex(uint8_t* data, size_t len, const char* info = "") {
    
    if (info[0] != '\0') {
        Serial.print(info);
        Serial.print(": ");
    }
    for (int i = (int)len - 1; i >= 0; i--) {
        if (data[i] < 0x10) Serial.print("0"); //pad single hex digits
        Serial.print(data[i], HEX);
    }
    Serial.println();
}



//Configures the DWM1000 with recommended values for AGC, DRX, and LDE.
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
    //   delay(10);


}


void dwm_basic_transmit(DWM_Module* module){
	/*
    /* Notes
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
	uint8_t tx_data[4] ={0xAA,0xBB,0xCC,0xDD};
	dwm_write_reg(module, 0x09, tx_data, 4);

	//SET FRAME LENGTH AND PREAMBLE LENGTH
	uint8_t tx_frame_control[5] = {0};
	dwm_read_reg(module, 0x08, tx_frame_control, 5);
	// fr_len = 4 + 2 = 6
	// Set frame length
	tx_frame_control[0] &= 0x80; //clear
	tx_frame_control[0] |= 6; //set
	// Set TXPSR=10, PE=00
	tx_frame_control[2] &= ~(0x3C);
	tx_frame_control[2] |= (0b10 << 2);

    // Set bitrate to 850kbps (Comment the blwow linesx for default of 6.8Mbps)
    // tx_frame_control[1] &= 0x9f; //clear
    // tx_frame_control[1] |= 0x20; //set to 850kpbs

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
    timeout of 50ms, will return false in that case
*/
bool dwm_receive(DWM_Module* module, uint8_t* buffer, uint16_t len){

    uint8_t sys_event_status_reg[5] = {0};
    uint8_t rx_frame_info_reg[4] = {0};
    // uint8_t rx_buffer[6] = {0}; // tflen = 6 but the top2 crc bits can be ignored here
    uint8_t sys_ctrl_reg[4] = {0};

    int tflen = 0;


    // ENABLE Rx
    dwm_read_reg(module, 0x0D, sys_ctrl_reg, 4);
    sys_ctrl_reg[1] |= (1 << 0); // RXENAB

    dwm_write_reg(module, 0x0D, sys_ctrl_reg, 4);

    // now dw1000 is waiting for preamble
    // Add timeouts here ig
    int tries = 500;
    for (int i=0;i<=tries;i++){
        dwm_read_reg(module, 0x0F, sys_event_status_reg, 5);

        if(sys_event_status_reg[1]&0x20){ // RXDFR is high
            dwm_read_reg(module, 0x10,rx_frame_info_reg,4);
            tflen = rx_frame_info_reg[0]&0x7f; //tflen mask
            // Serial.print("Does this work??");
            dwm_read_reg(module, 0x11,buffer,tflen-2); // no need CRC bits --> write data to buffer
            // PROBLEM: can corrupt mem if buff too small
            // Better to come up with a standard tflen across all DWMs and set to max. --> DONE
            // Serial.print("Does this work tooooo ??");

            // clear flags
            uint8_t clear[5] = {0};
            clear[0] = 0x80;  // TXFRS
            dwm_write_reg(module, 0x0F, clear, 5);


            return true;
        }

        delay(1);
        // Serial.println(i);
    }

    return false;

}



// Modifying data rate

/*Def  = 6.8Mbs
but for longer range it needs to be reduced (110 or 850 kbps)

TX_FCTRL TXBR bits must be set to 0b00(110 kbps) 0r 0b01(850kbps)
SYS_CFG  bit RXM110K must be 1 if using 110 kbps else leave 0

*/

int send_frame(DWM_Module* module, uint8_t* payload, uint8_t len)
{
    // write to TX buffer
    dwm_write_reg(module, 0x09, payload, len);

    // configure TX_FCTRL
    uint8_t tx_frame_control[5] = {0};
    dwm_read_reg(module, 0x08, tx_frame_control, 5);

    tx_frame_control[0] &= 0x80;
    tx_frame_control[0] |= (len + 2); // include CRC

    // NEW
    // Set PRF to 64Mhz (comment out for 16Mhz)
    tx_frame_control[1] &= 0b11001111;
    tx_frame_control[1] |= 0b00100000;

    dwm_write_reg(module, 0x08, tx_frame_control, 5);

    // trigger TX
    uint8_t sys_ctrl[4] = {0};
    dwm_read_reg(module, 0x0D, sys_ctrl, 4);
    sys_ctrl[0] |= (1 << 1); // TXSTRT
    dwm_write_reg(module, 0x0D, sys_ctrl, 4);

    uint8_t sys_event_status_reg[5] = {0};
    // delay(1);    
    delayMicroseconds(500); // this is not an issue bcs this happens after sending a frame, t_remote will be unaffected
    dwm_read_reg(module, 0x0f, sys_event_status_reg, 5);
    if(!(sys_event_status_reg[0]&0x80)){
    	// Serial.println("No Send");
        return 0;
    }

    return 1;
}


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


// Calculate time difference between 2 timestamops
uint64_t ts_diff(uint64_t a, uint64_t b)
{
    //handle wraparound
    const uint64_t MASK = ((uint64_t)1 << 40) - 1;
    return (a - b) & MASK;
}



// Reset remote state machine and timestamps
void remote_reset_session(DWM_Module* module)
{
    retry_counter = 0;
    t_rx_m1 = 0;
    t_tx_m2 = 0;
    t_rx_m3 = 0;
    t_tx_m4 = 0;

    uint8_t sys_ctrl[4] = {0};
    sys_ctrl[0] |= (1 << 6);  // TRXOFF
    dwm_write_reg(module, 0x0D, sys_ctrl, 4);

    // Clear all status flags
    uint8_t clear[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    dwm_write_reg(module, 0x0F, clear, 5);

    // Re-enable RX
    uint8_t sys_ctrl2[4] = {0};
    sys_ctrl2[1] |= (1 << 0);  // RXENAB
    dwm_write_reg(module, 0x0D, sys_ctrl2, 4);
    remote_state = REMOTE_WAIT_M1;
}


bool process_response(uint8_t *rx_buffer, uint16_t len, uint8_t msg_type)
{
    if (len < 6)
        return false;

    //Check message type
    if (rx_buffer[0] != msg_type)
        return false;

    return true;
}



// State Machine for remote implementing double sided two way ranging.
// param: isr_flag = to indicate button press for standby
void run(DWM_Module* module, volatile bool* isr_flag)
{
    uint8_t sys_status[5] = {0};
    uint8_t frame_info[4] = {0};

    switch(remote_state)
    {
        case REMOTE_WAIT_M1:
        {
            if(*isr_flag)
            {   
                *isr_flag = false;

                dwm_read_reg(module, 0x0F, sys_status, 5);
                dwm_write_reg(module, 0x0F, sys_status, 5); // clear status

                bool rxdfr   = sys_status[1] & 0x20;
                bool rxfcg   = sys_status[1] & 0x40;
                bool ldedone = sys_status[1] & 0x04;

                if(rxdfr && rxfcg && ldedone)
                {   

                    // check if the response is of the correct type
                    uint8_t rx_buff_m3[6];

                    dwm_read_reg(module, 0x11, rx_buff_m3, 6);

                    if(!process_response(rx_buff_m3, 6,MSG_TYPE_POLL)){
                    remote_reset_session(module);

                    Serial.println("STATE: REMOTE WAIT M1 -> RX Error: Returning to REMOTE_WAIT_M1");
                    break;
                    }
                    // save timestamp
                    t_rx_m1 = read_timestamp(module, 0x15);
                    retry_counter = 0; // reset retries
                    remote_state = REMOTE_SEND_M2;
                }
                else
                {
                    // RX corrupted → reset session
                    remote_reset_session(module);
                    Serial.println("STATE: REMOTE WAIT M1 - > RX Error: Returning to REMOTE_WAIT_M1");
                    break;

                }
            }
            else
            {
                // optional retry for timeout handling
                if(++retry_counter > MAX_RETRIES)
                {
                    remote_reset_session(module);
                    // Serial.println("RX Timeout: Returning to REMOTE_WAIT_M1");
                    break;
                }
            }
            break;
        }

        case REMOTE_SEND_M2:
        {
            // compute delayed TX time
            uint64_t reply_delay = US_TO_DWT(1000);  // 500 µs
            t_tx_m2 = (t_rx_m1 + reply_delay) & (((uint64_t)1 << 40) - 1);
            t_tx_m2 &= ~0x1FFULL; // zero lower 9 bits

            // write DX_TIME (0x0A)
            uint8_t dx_time[5];
            for(int i = 0; i < 5; i++)
                dx_time[i] = (t_tx_m2 >> (8*i)) & 0xFF;
            dwm_write_reg_sub(module, 0x0A, 0x00, dx_time, 5);

            // prepare payload
            uint64_t treply = ts_diff(t_tx_m2, t_rx_m1);
            uint8_t response_payload[6];
            response_payload[0] = MSG_TYPE_RESPONSE;
            for(int i = 0; i < 5; i++){
                response_payload[1+i] = (treply >> (8*i)) & 0xFF;

            }

            // Write TX buffer
            dwm_write_reg(module, 0x09, response_payload, 6);            

            //Set frame length
            uint8_t tx_frame_control[5] = {0};
            dwm_read_reg(module, 0x08, tx_frame_control, 5);
            tx_frame_control[0] &= 0x80;
            tx_frame_control[0] |= (6 + 2);

            //Trigger delayed TX
            uint8_t sys_ctrl[4] = {0};
            sys_ctrl[0] |= (1 << 2); // TXDLYS
            sys_ctrl[0] |= (1 << 1); // TXSTRT
            dwm_write_reg(module, 0x0D, sys_ctrl, 4);


            remote_state = REMOTE_WAIT_TX_DONE;
            retry_counter = 0;
            break;
        }

        case REMOTE_WAIT_TX_DONE:
        {
            if(*isr_flag)
            {
                *isr_flag = false;
                dwm_read_reg(module, 0x0F, sys_status, 5);

                bool txfrs = sys_status[0] & 0x80; // TXFRS bit
                if(txfrs)
                {
                    uint8_t clear[5] = {0}; 
                    dwm_read_reg(module,0x0F,clear,5);
                    // clear[0] = 0x80;
                    dwm_write_reg(module, 0x0F, clear, 5); // clear All

                    // re-enable RX
                    uint8_t sys_ctrl[4] = {0};
                    dwm_read_reg(module, 0x0D, sys_ctrl, 4);
                    sys_ctrl[1] |= (1 << 0); // RXENAB
                    dwm_write_reg(module, 0x0D, sys_ctrl, 4);


                    remote_state = REMOTE_WAIT_M3;
                    retry_counter = 0;
                    Serial.println("STATE: REMOTE WAIT TX DONE -> TX Success: Moving on to REMOTE_WAIT_M3");
                }
                else
                {
                    remote_reset_session(module);
                    Serial.println(" STATE: REMOTE WAIT TX DONE -> TX Failure: Returning to REMOTE_WAIT_M1");
                }
            }
            else
            {
                if(++retry_counter > MAX_RETRIES)
                {
                    remote_reset_session(module);
                    Serial.println("STATE: REMOTE WAIT TX DONE -> TX Timeout: Returning to REMOTE_WAIT_M1");
                }
            }
            break;
        }

/// NEW ADDITION FOR DS-TWR from single sided
        case REMOTE_WAIT_M3:
        {
            if(*isr_flag)
            {
                *isr_flag =  false;
                dwm_read_reg(module, 0x0F, sys_status, 5);
                dwm_write_reg(module, 0x0F, sys_status, 5); // clear status

                bool rxdfr   = sys_status[1] & 0x20;
                bool rxfcg   = sys_status[1] & 0x40;
                bool ldedone = sys_status[1] & 0x04;

                if(rxdfr && rxfcg && ldedone){
                    // check if the response is of the correct type
                    uint8_t rx_buff_m3[6];

                    dwm_read_reg(module, 0x11, rx_buff_m3, 6);

                    if(!process_response(rx_buff_m3, 6,MSG_TYPE_POLL_2)){
                    remote_reset_session(module);
                    Serial.println("STATE: REMOTE WAIT M3 -> RX Error: Returning to REMOTE_WAIT_M1 - WRONG MSG TYPE");
                    break;
                    }
                    

                    //save timestamp
                    t_rx_m3 = read_timestamp(module, 0x15);
                    retry_counter = 0; //reset retries
                    remote_state = REMOTE_SEND_M4;
                }
                else
                {
                    //RX corrupted-reset session
                    remote_reset_session(module);
                    Serial.println("STATE: REMOTE WAIT M3 -> RX Error: Returning to REMOTE_WAIT_M1");

                    Serial.print("RXDFR: ");
                    Serial.print(rxdfr ? "1" : "0");

                    Serial.print("  RXFCG: ");
                    Serial.print(rxfcg ? "1" : "0");

                    Serial.print("  LDEDONE: ");
                    Serial.println(ldedone ? "1" : "0");
                }                
                
            }else{

                if(++retry_counter > MAX_RETRIES)
                {
                    remote_reset_session(module);
                    Serial.println(" STATE: REMOTE WAIT M3 -> RX Timeout: Returning to REMOTE_WAIT_M1");
                    break;
                }
            }

            break;
        }

        case REMOTE_SEND_M4:
        {
            
            // compute delayed TX time
            uint64_t reply_delay = US_TO_DWT(1000);  // 500 µs
            t_tx_m4 = (t_rx_m3 + reply_delay) & (((uint64_t)1 << 40) - 1);
            t_tx_m4 &= ~0x1FFULL; // zero lower 9 bits

            // write DX_TIME (0x0A)
            uint8_t dx_time[5];
            for(int i = 0; i < 5; i++)
                dx_time[i] = (t_tx_m4 >> (8*i)) & 0xFF;
            dwm_write_reg_sub(module, 0x0A, 0x00, dx_time, 5);

            // prepare payload
            uint64_t treply = ts_diff(t_tx_m4, t_rx_m3);
            uint8_t response_payload[6];
            response_payload[0] = MSG_TYPE_FINAL;
            for(int i = 0; i < 5; i++){
                response_payload[1+i] = (treply >> (8*i)) & 0xFF;

            }

            // Write TX buffer
            dwm_write_reg(module, 0x09, response_payload, 6);            

            //Set frame length
            uint8_t tx_frame_control[5] = {0};
            dwm_read_reg(module, 0x08, tx_frame_control, 5);
            tx_frame_control[0] &= 0x80;
            tx_frame_control[0] |= (6 + 2);

            //Trigger delayed TX
            uint8_t sys_ctrl[4] = {0};
            sys_ctrl[0] |= (1 << 2); //TXDLYS
            sys_ctrl[0] |= (1 << 1); //TXSTRT
            dwm_write_reg(module, 0x0D, sys_ctrl, 4);


            remote_state = REMOTE_WAIT_TX_DONE_FINAL;
            retry_counter = 0;
            break;
        }

        case REMOTE_WAIT_TX_DONE_FINAL:
        {
            if(*isr_flag)
            {
                *isr_flag = false;
                dwm_read_reg(module, 0x0F, sys_status, 5);

                bool txfrs = sys_status[0] & 0x80; //TXFRS bit
                if(txfrs)
                {
                    uint8_t clear[5] = {0}; 
                    dwm_read_reg(module,0x0F,clear,5);
                    // clear[0] = 0x80;
                    dwm_write_reg(module, 0x0F, clear, 5); //clear All

                    // re-enable RX
                    uint8_t sys_ctrl[4] = {0};
                    dwm_read_reg(module, 0x0D, sys_ctrl, 4);
                    sys_ctrl[1] |= (1 << 0); // RXENAB
                    dwm_write_reg(module, 0x0D, sys_ctrl, 4);

                    remote_state = REMOTE_WAIT_M1;
                    retry_counter = 0;
                    Serial.println("STATE: REMOTE_WAIT_TX_DONE_FINAL -> TX Success: Moving on to REMOTE_WAIT_M1");
                }
                else
                {
                    remote_reset_session(module);
                    Serial.println("STATE: REMOTE_WAIT_TX_DONE_FINAL -> TX Failure: Returning to REMOTE_WAIT_M1");
                    break;
                }
            }
            else
            {
                if(++retry_counter > MAX_RETRIES)
                {
                    remote_reset_session(module);
                    Serial.println("STATE: REMOTE_WAIT_TX_DONE_FINAL -> TX Timeout: Returning to REMOTE_WAIT_M1");
                    break;
                }
            }
            break;
        }



    }
}