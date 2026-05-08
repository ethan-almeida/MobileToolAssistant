/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DW1000.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//void dwm_read_sub_reg_len(uint8_t reg_file_id,
//                      uint8_t sub_reg,
//                      uint8_t *data,
//                      uint8_t len)
//{
//    uint8_t tx[2 + len];
//    uint8_t rx[2 + len];
//
//    tx[0] = (1 << 6) | (reg_file_id & 0x3F);
//    tx[1] = sub_reg & 0x7F;
//
//    for (int i = 2; i < 2 + len; i++)
//        tx[i] = 0;
//
//    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_RESET);
//    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2 + len, HAL_MAX_DELAY);
//    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_SET);
//
//    memcpy(data, &rx[2], len);
//}


uint32_t dwm_read_device_id(void)
{
    uint8_t tx[5] = {0};
    uint8_t rx[5] = {0};

    // Read, no sub-address
    tx[0] = 0x00;  // Read command for ID reg

    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 5, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_SET);

    return (rx[4] << 24) | (rx[3] << 16) | (rx[2] << 8) | rx[1];


}


#define SYS_STATUS 0x0F
#define SYS_CONFIG 0x04


uint32_t dwm_read_sys_status(void)
{
    uint8_t tx[5] = {0};
    uint8_t rx[5] = {0};

    tx[0] = SYS_STATUS;

    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 5, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_SET);

    return (rx[4] << 24) | (rx[3] << 16) | (rx[2] << 8) | rx[1];
}



/*
 *
	 read 4 octet registers
	 max reg id = 0x3F, to allow two MSBs for SPI header bits
 *
 */
uint32_t dwm_read_reg_32(uint8_t reg_id){

	// SPI
	uint8_t tx[5] = {0};
	uint8_t rx[5] = {0};
	uint32_t reg_read = 0; // temp

	// Bit 7 = 0 (read)
	// Bit 6 = 0 (no sub index present)
	tx[0] = ((0 << SPI_OPERATION_OFFSET) | (0 << SPI_SUB_INDEX_PRESENT_OFFSET) | (reg_id & 0x3F));
    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, sizeof(tx), HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_SET);

    // first byte has to be discarded (dummy)
    reg_read  = (rx[4] << 24) | (rx[3] << 16) | (rx[2] << 8) | rx[1];

    return reg_read;

}

// Write 32-bit register
void dwm_write_reg_32(uint8_t reg_id, uint32_t value)
{
    uint8_t tx[5];

    // Write command: bit 7 = 1, bit 6 = 0 (no sub-index)
    tx[0] = (1 << 7) | (0 << 6) | (reg_id & 0x3F);

    // Data in little-endian (LSB first)
    tx[1] = (value >> 0) & 0xFF;
    tx[2] = (value >> 8) & 0xFF;
    tx[3] = (value >> 16) & 0xFF;
    tx[4] = (value >> 24) & 0xFF;

    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tx, 5, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_SET);
}



// Write to sub-register (for registers like 0x23:04)
void dwm_write_sub_reg_32(uint8_t reg_file_id, uint8_t sub_reg, uint32_t value)
{
    uint8_t tx[6];

    // Write with sub-index: bit 7 = 1, bit 6 = 1
    tx[0] = (1 << 7) | (1 << 6) | (reg_file_id & 0x3F);

    // Sub-index (7-bit)
    tx[1] = sub_reg & 0x7F;

    // Data
    tx[2] = (value >> 0) & 0xFF;
    tx[3] = (value >> 8) & 0xFF;
    tx[4] = (value >> 16) & 0xFF;
    tx[5] = (value >> 24) & 0xFF;

    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tx, 6, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_SET);
}



uint32_t dwm_read_sub_reg_32(uint8_t reg_file_id, uint8_t sub_reg)
{
    uint8_t tx[6] = {0};
    uint8_t rx[6] = {0};

    // Read operation with sub-index: bit 7 = 0 (read), bit 6 = 1 (sub-index present)
    tx[0] = (0 << 7) | (1 << 6) | (reg_file_id & 0x3F);
    tx[1] = sub_reg & 0x7F;
    // Note: tx[2] to tx[5] remain 0 - these are dummy bytes for reading

    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 6, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_SET);

    // Data starts at rx[2] because:
    // rx[0] = echoed header byte (the one we sent)
    // rx[1] = echoed sub-index byte (the one we sent)
    // rx[2] = LSB of the 32-bit value
    // rx[3] = byte 1
    // rx[4] = byte 2
    // rx[5] = MSB of the 32-bit value
    return (rx[5] << 24) | (rx[4] << 16) | (rx[3] << 8) | rx[2];
}


//////////////////////////////////////////////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

void dwm_read_reg(uint8_t reg_id, uint8_t *data, uint8_t len)
{
    uint8_t tx[1 + len];
    uint8_t rx[1 + len];

    tx[0] = reg_id & 0x3F;  // read, no sub-index
    memset(&tx[1], 0, len);

    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 1 + len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_SET);

    memcpy(data, &rx[1], len);
}


void dwm_write_reg(uint8_t reg_id, uint8_t *data, uint8_t len)
{
    uint8_t tx[1 + len];

    tx[0] = 0x80 | (reg_id & 0x3F);  // write bit = 1
    memcpy(&tx[1], data, len);

    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tx, 1 + len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_SET);
}





void dwm_write_reg_sub(uint8_t reg_id,
                       uint16_t subaddr,
                       uint8_t *data,
                       uint16_t len)
{
    uint8_t header[3];
    uint8_t header_len = 1;

    // First header byte: write + reg ID
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

    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi1, header, header_len, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, data, len, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_SET);
}


void dwm_read_reg_sub(uint8_t reg_id,
                      uint16_t subaddr,
                      uint8_t *data,
                      uint16_t len)
{
    uint8_t header[3];
    uint8_t header_len = 1;

    // Base read header
    header[0] = reg_id & 0x3F;

    // Add sub-address if requested
    if (subaddr != 0xFFFF) {   // sentinel: no subaddress
        header[0] |= 0x40;     // sub-address flag

        header[1] = subaddr & 0x7F;
        header_len = 2;

        if (subaddr > 0x7F) {
            header[1] |= 0x80;           // extension flag
            header[2] = subaddr >> 7;
            header_len = 3;
        }
    }

    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_RESET);

    // Send header
    HAL_SPI_Transmit(&hspi1, header, header_len, HAL_MAX_DELAY);

    // Receive data
    HAL_SPI_Receive(&hspi1, data, len, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_SET);
}




uint32_t dwm_read_sys_config(void)
{
    uint8_t tx[5] = {0};
    uint8_t rx[5] = {0};

    tx[0] = SYS_CONFIG;

    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 5, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_SET);

    return (rx[4] << 24) | (rx[3] << 16) | (rx[2] << 8) | rx[1];
}




void dwm_reset(void)
{


    HAL_GPIO_WritePin(DWM_RESET_N_GPIO_Port,
                      DWM_RESET_N_Pin,
                      GPIO_PIN_RESET);

    HAL_Delay(1);   // 1ms is more than enough

    // Release reset (Hi-Z because open-drain)
    HAL_GPIO_WritePin(DWM_RESET_N_GPIO_Port,
                      DWM_RESET_N_Pin,
                      GPIO_PIN_SET);

    HAL_Delay(10);   // allow DW1000 time to boot
}


uint32_t dwm_read_sys_state(void)
{
    uint8_t tx[2] = {0};
    uint8_t rx[4] = {0};

    tx[0] = 0x19;  // SYS_STATE register ID
    tx[1] = 0x00;  // sub-address

    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, rx, 4, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_SET);

    return (rx[3] << 24) | (rx[2] << 16) | (rx[1] << 8) | rx[0];
}

void dwm_tx_test(void)
{
    uint8_t frame[] = {0x41, 0x88, 0x00, 0xCA, 0xDE, 0xBE, 0xEF};

    // 1. Write frame to TX buffer (reg 0x09)
    uint8_t tx_buf[1 + sizeof(frame)];
    tx_buf[0] = 0x09 | (1 << 7);  // write to TX_BUFFER
    memcpy(&tx_buf[1], frame, sizeof(frame));

    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tx_buf, sizeof(tx_buf), HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_SET);

    // 2. Set frame length (TX_FCTRL = 0x08)
    dwm_write_reg_32(0x08, sizeof(frame));

    // 3. Start transmission (SYS_CTRL = 0x0D)
    dwm_write_reg_32(0x0D, 0x00000002);  // TXSTRT
}


/*
 * SPI lets u read more than the reg if octets do not match
 */


void modify_default_config(void){

	//1. AGC Tune1 - 2 octets
	uint8_t current_agc1_config[2] = {0};
	dwm_read_reg_sub(0x23,0x04,current_agc1_config,2);

	current_agc1_config[0] = 0x70;
	current_agc1_config[1] = 0x88;

	dwm_write_reg_sub(0x23,0x04,current_agc1_config,2);
	dwm_read_reg_sub(0x23,0x04,current_agc1_config,2);


	//2.AGC Tune2 (4 oct)
	uint8_t new_agc2_config[4] = {0x07,0xa9,0x02,0x25};
	dwm_write_reg_sub(0x23,0x0c,new_agc2_config,4);

	//	//3. DRX_Tune_2 (4 oct)
	uint8_t new_drx2_config[4] = {0x2d,0x00,0x1a,0x31};
	dwm_write_reg_sub(0x27,0x08,new_drx2_config,4);

	//4. NTM
	uint8_t lde_cfg_1[1] = {0};
	dwm_read_reg_sub(0x2E,0x0806,lde_cfg_1,1);
	lde_cfg_1[0] = (lde_cfg_1[0] & ~0x1F) | (0x0D & 0x1F);
	dwm_write_reg_sub(0x2E,0x0806,lde_cfg_1,1);

//	5.LDE Config 2
	uint8_t lde_cfg_2[2] = {0x07,0x16};
	dwm_write_reg_sub(0x2E,0x1806,lde_cfg_2,2);

//	//6. Tx Power
	uint8_t tx_power_ctrl[4] ={0x48,0x28,0x08,0x0e};
	dwm_write_reg_sub(0x1E,0x00,tx_power_ctrl,4);


//	7. RF_TXCTRL (3 oct)
//	 DATASHEET_UNCLEAR (refer pg 153)
//	 The data sheet says set 24 bits,  last oct is reserved and set to 0x00 but when reading it has 0xde, might have to set to 0x00 of issues
	uint8_t rf_txctrl[3] = {0xe3,0x3f,0x1e};
	dwm_write_reg_sub(0x28,0x0c,rf_txctrl,3);

//	8.TC_PGDELAY (1 oct)
	uint8_t tc_pgdelay[1] = {0xc0};
	dwm_write_reg_sub(0x2A,0x0b,tc_pgdelay,1);

//	9.PLL_TUNE (1 oct)
	uint8_t fs_pll_tune[1] = {0xbe};
	dwm_write_reg_sub(0x2b,0x0b,fs_pll_tune,1);

//	10. LDE_LOAD
	//NOTE: refer page 176 for ldeload instruction if waking up from sleep/deep sleep

	//L1 - write to PMSC Control0 lower 16 bits
	uint8_t pmsc_ctrl_0_lower_2_oct[2] = {0x01,0x03};
	dwm_write_reg_sub(0x36, 0x00, pmsc_ctrl_0_lower_2_oct, 2);

	//L2 - set OTP control LDELOAD bit (write entire reg)
	uint8_t otp_control[2] = {0x00,0x80};
	dwm_write_reg_sub(0x2d, 0x06, otp_control, 2);

	//Wait 150us
	HAL_Delay(1);

	// L-3
	uint8_t pmsc_ctrl_0_lower_2_oct_L3[2] = {0x00,0x02};
	dwm_write_reg_sub(0x36, 0x00, pmsc_ctrl_0_lower_2_oct_L3, 2);


// 10.LDO TUNE
	//NOTE: can be skipped ig
}



void transmit(){
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
	dwm_write_reg(0x09, tx_data, 4);

	//SET FRAME LENGTH AND PREAMBLE LENGTH
	uint8_t tx_frame_control[5] = {0};
	dwm_read_reg(0x08, tx_frame_control, 5);
	// fr_len = 4 + 2 =
	// Set frame length
	tx_frame_control[0] &= 0x80;
	tx_frame_control[0] |= 6;
	// Set TXPSR=10, PE=00
	tx_frame_control[2] &= ~(0x3C);
	tx_frame_control[2] |= (0b10 << 2);
	dwm_write_reg(0x08, tx_frame_control, 5);


	//TRASMIT
	uint8_t sys_ctrl[4] = {0};
	dwm_read_reg(0x0D, sys_ctrl, 4);
	sys_ctrl[0] |= (1 << 1);
	dwm_write_reg(0x0D, sys_ctrl, 4);



}






/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

//  HAL_Delay(1000); // to allow pll to lock

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(10);
  dwm_reset();
  uint32_t id = dwm_read_reg_32(DEV_ID);

  int count = 0;
//  int run_count = 0;
  int clk_pll_error_count = 0;
  int txfrs_error_count = 0;
//  uint32_t sys_status = dwm_read_reg_32(SYS_STATUS);
//  uint32_t sys_config = 0;
  uint8_t sys_status_reg_arr[5];

  modify_default_config();

  //set PLLLDT  bit high
  uint8_t ec_ctrl_reg[4] = {0};
  dwm_read_reg_sub(0x24,0x00,ec_ctrl_reg,4);
  ec_ctrl_reg[0] = ec_ctrl_reg[0]|0x04;
  dwm_write_reg_sub(0x24,0x00,ec_ctrl_reg,4);




  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

    uint32_t sys_status = dwm_read_reg_32(SYS_STATUS);

//    if(sys_status & 0x2000000){
//			clk_pll_error_count++;
//    }
    count++;

    uint32_t sys_status_idle = dwm_read_sys_state();

//    dwm_tx_test();
    transmit();
    HAL_Delay(1);

    dwm_read_reg(0x0F, sys_status_reg_arr, 5);
    sys_status = dwm_read_reg_32(SYS_STATUS);


    if(!(sys_status_reg_arr[0]&0x80)){
    	txfrs_error_count++;
    }



  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DWM_RESET_N_Pin|CS_N_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DWM_RESET_N_Pin */
  GPIO_InitStruct.Pin = DWM_RESET_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DWM_RESET_N_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_N_Pin */
  GPIO_InitStruct.Pin = CS_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_N_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */



  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
