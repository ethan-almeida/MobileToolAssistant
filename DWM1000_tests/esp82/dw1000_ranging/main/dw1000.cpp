#include "core_esp8266_features.h"
#include <stdint.h>
#include "HardwareSerial.h"
#include "dw1000.h"
#include <cstdint>
#include "dwm_regs.h"

#define SPI_SPEED_MAX 20000000

#define MSG_TYPE_POLL 0xA1
#define MSG_TYPE_RESPONSE 0xB2
#define MSG_TYPE_FINAL 0xA2

#define FRAME_LEN_MAX 127 // for received frame
static uint8_t rx_buffer[FRAME_LEN_MAX];
static uint32_t status_reg = 0;
static uint16_t frame_len = 0;

static remote_state_t remote_state = REMOTE_WAIT_M1;
static uint64_t t_rx_m1 = 0;
static uint64_t t_tx_m2 = 0;
static uint64_t t_rx_m3 = 0;

int retry_counter = 0;
#define MAX_RETRIES 3




//old 20000000 
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

/*
reg_id = register id 
len = no of octets in register (bytes)
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



void printRegHex(uint8_t* data, size_t len, const char* info = "") {
    
    if (info[0] != '\0') {
        Serial.print(info);
        Serial.print(": ");
    }

    // in the buffer --> lsbs of the register are in lower indexes 
    for (int i = (int)len - 1; i >= 0; i--) {
        if (data[i] < 0x10) Serial.print("0"); // pad single hex digits
        Serial.print(data[i], HEX);
    }
    Serial.println();
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
	// uint8_t new_drx2_config[4] = {0x2d,0x00,0x1a,0x31};

    // mofified PAC size=32 for 1024 preamble size
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
            Serial.print("Does this work??");
            dwm_read_reg(module, 0x11,buffer,tflen-2); // no need CRC bits --> write data to buffer
            // PROBLEM: can corrupt mem if buff too small
            // Better to come up with a standard tflen across all DWMs and set to max. --> DONE
            Serial.print("Does this work tooooo ??");

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

    dwm_write_reg(module, 0x08, tx_frame_control, 5);

    // trigger TX
    uint8_t sys_ctrl[4] = {0};
    dwm_read_reg(module, 0x0D, sys_ctrl, 4);
    sys_ctrl[0] |= (1 << 1); // TXSTRT
    dwm_write_reg(module, 0x0D, sys_ctrl, 4);


    uint8_t sys_event_status_reg[5] = {0};
    dwm_read_reg(module, 0x0f, sys_event_status_reg, 5);

    if(!(sys_event_status_reg[0]&0x80)){
    	return 0;
    }

    return 1;
}


//______________________________________
// DOUBLE SIDED TWO WAY RANGING 
// ______________________________________


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


void run_remote(DWM_Module* module)
{
    uint8_t buffer[FRAME_LEN_MAX];
    switch(remote_state)
    {
        case REMOTE_WAIT_M1: // this ones forever wait
        {   
            Serial.println("In State REMOTE_WAIT_M1");
            if (dwm_receive(module, buffer, FRAME_LEN_MAX))
            {
                if (buffer[0] == MSG_TYPE_POLL) //0xA1
                {
                    Serial.println("REMOTE_WAIT_M1: Received Message");
                    t_rx_m1 = read_timestamp(module, 0x15); // RX time
                    remote_state = REMOTE_SEND_M2;
                } else {
                    Serial.println("REMOTE_WAIT_M1: Received Message of wrong type");
                    printRegHex(buffer,10);

                }
            }
            break;
        }

        case REMOTE_SEND_M2:
        {
            Serial.println("In State REMOTE_SEND_M2");
            uint8_t resp[1] = { MSG_TYPE_RESPONSE }; //0xB2

            int status = send_frame(module, resp, 1);
            if(status){
                t_tx_m2 = read_timestamp(module, 0x17); // TX time
                // retry_counter = 0;
                remote_state = REMOTE_WAIT_M3;
                Serial.println("REMOTE_SEND_M2: Message sent");
                delayMicroseconds(500);
                break;
            }
            

            break;
        }

        case REMOTE_WAIT_M3:
        {
            Serial.println("In State REMOTE_WAIT_M3");
            if ((dwm_receive(module, buffer, FRAME_LEN_MAX)) && (buffer[0] == MSG_TYPE_FINAL))
            {
                // if (buffer[0] == MSG_TYPE_FINAL) //0xA3
                // {
                    Serial.println("REMOTE_WAIT_M3: Message received...");
                    t_rx_m3 = read_timestamp(module, 0x15);
                    remote_state = REMOTE_SEND_M4;
                // }
            }
            else
            {
                Serial.println("REMOTE_WAIT_M3: Message not received. Retrying...");
                // timeout occurred
                if (retry_counter < MAX_RETRIES)
                {
                    Serial.println(retry_counter);
                    retry_counter++;
                    remote_state = REMOTE_SEND_M2;
                }
                else
                {
                    remote_state = REMOTE_WAIT_M1;
                    retry_counter = 0;

                }
            }

            break;
        }

        case REMOTE_SEND_M4:
        {
            Serial.println("In State REMOTE_SEND_M4");
            uint8_t final_msg[1] = { 0xB4 }; 

            send_frame(module, final_msg, 1);

            // session complete
            remote_state = REMOTE_WAIT_M1;

            Serial.println("Session Complete");
            break;
        }
    }
}



void remote_reset_session()
{
    retry_counter = 0;
    t_rx_m1 = 0;
    t_tx_m2 = 0;
    t_rx_m3 = 0;

    remote_state = REMOTE_WAIT_M1;
}
// Logic

/*
    STATE =  WAITING FOR M1
        If received and MSG_ID = M1_ID
            STATE = SEND_MESSAGE_M2
        ELSE
            state unchanged
            But the robot will continue sending the wrong message type
    


    STATE = SEND M2
        Change state to receive M3


    SATE WAITING FOR M3
        if timeout state = SEND M2(resend)
        if (message received and ID matches)--> State = SEND_M4
    
    STATE SEND M4
        send message (can use delayed tx here to send correct timestamp)
        //send continously for some time ig, bcs state 1 waits for some time

    
    Is this a feasible schedule



*/

/*
/*
REMOTE / ROBOT FSM (Responder in DS-TWR)

STATE = WAIT_M1
--------------------------------
- Enable RX and wait for packet
- If packet received:
    - If MSG_ID == M1_ID:
        → STATE = SEND_M2
    - Else:
        Ignore packet
        Stay in WAIT_M1
- No transmission in this state


STATE = SEND_M2
--------------------------------
- Capture RX timestamp of M1
- Prepare M2 response
- Transmit M2 (immediate or delayed TX)
- Start M3 timeout timer
- Retry counter = 0
→ STATE = WAIT_M3


STATE = WAIT_M3
--------------------------------
- Enable RX and wait for M3

- If valid M3 received:
    → STATE = SEND_M4

- If timeout occurs:
    - If retry_counter < MAX_RETRIES:
        retry_counter++
        → STATE = SEND_M2   // resend M2
    - Else:
        → STATE = WAIT_M1   // abort session


STATE = SEND_M4
--------------------------------
- Compute timestamps
- Send final message M4
  (use delayed TX for precise timing)

- End session
→ STATE = WAIT_M1


Key Design Rules

WAIT states never transmit

SEND states transmit once

All retries are bounded

Every session ends by returning to WAIT_M1

No continuous packet spamming
*/




