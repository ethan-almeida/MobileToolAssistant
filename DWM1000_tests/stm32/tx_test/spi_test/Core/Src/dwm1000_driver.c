///*
// * dwm1000_header.c
// *
// *  Created on: Feb 12, 2026
// *      Author: Aneek Mubarak
// */
//
//
//
///*
// * Register Read Functions
// */
//
//
//
//
//
//uint32_t dwm_read_reg_32(uint8_t reg_id){
//
//	// SPI
//	uint8_t tx[5] = {0};
//	uint8_t rx[5] = {0};
//	uint32_t reg_read = 0; // temp
//
//	// Bit 7 = 0 (read)
//	// Bit 6 = 0 (no sub index present)
//	tx[0] = ((0 << SPI_OPERATION_OFFSET) | (0 << SPI_SUB_INDEX_PRESENT_OFFSET) | (reg_id & 0x3F));
//    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_RESET);
//    HAL_SPI_TransmitReceive(&hspi1, tx, rx, sizeof(tx), HAL_MAX_DELAY);
//    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_SET);
//
//    // first byte has to be discarded (dummy)
//    reg_read  = (rx[4] << 24) | (rx[3] << 16) | (rx[2] << 8) | rx[1];
//    return reg_read;
//}
//
//
//
//
//uint32_t dwm_read_sub_reg_32(uint8_t reg_file_id, uint8_t sub_reg)
//{
//    uint8_t tx[6] = {0};
//    uint8_t rx[6] = {0};
//
//    // Read operation with sub-index: bit 7 = 0 (read), bit 6 = 1 (sub-index present)
//    tx[0] = (0 << 7) | (1 << 6) | (reg_file_id & 0x3F);
//    tx[1] = sub_reg & 0x7F;
//    // Note: tx[2] to tx[5] remain 0 - these are dummy bytes for reading
//
//    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_RESET);
//    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 6, HAL_MAX_DELAY);
//    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_SET);
//
//    // Data starts at rx[2] because:
//    // rx[0] = echoed header byte (the one we sent)
//    // rx[1] = echoed sub-index byte (the one we sent)
//    // rx[2] = LSB of the 32-bit value
//    // rx[3] = byte 1
//    // rx[4] = byte 2
//    // rx[5] = MSB of the 32-bit value
//    return (rx[5] << 24) | (rx[4] << 16) | (rx[3] << 8) | rx[2];
//}
//
//
//
///*
// * REGISTER WRITES
// */
//
//void dwm_write_reg_32(uint8_t reg_id, uint32_t value)
//{
//    uint8_t tx[5];
//
//    // Write command: bit 7 = 1, bit 6 = 0 (no sub-index)
//    tx[0] = (1 << 7) | (0 << 6) | (reg_id & 0x3F);
//
//    // Data in little-endian (LSB first)
//    tx[1] = (value >> 0) & 0xFF;
//    tx[2] = (value >> 8) & 0xFF;
//    tx[3] = (value >> 16) & 0xFF;
//    tx[4] = (value >> 24) & 0xFF;
//
//    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_RESET);
//    HAL_SPI_Transmit(&hspi1, tx, 5, HAL_MAX_DELAY);
//    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_SET);
//}
//
//
//// Write to sub-register (for registers like 0x23:04)
//void dwm_write_sub_reg_32(uint8_t reg_file_id, uint8_t sub_reg, uint32_t value)
//{
//    uint8_t tx[6];
//
//    // Write with sub-index: bit 7 = 1, bit 6 = 1
//    tx[0] = (1 << 7) | (1 << 6) | (reg_file_id & 0x3F);
//
//    // Sub-index (7-bit)
//    tx[1] = sub_reg & 0x7F;
//
//    // Data
//    tx[2] = (value >> 0) & 0xFF;
//    tx[3] = (value >> 8) & 0xFF;
//    tx[4] = (value >> 16) & 0xFF;
//    tx[5] = (value >> 24) & 0xFF;
//
//    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_RESET);
//    HAL_SPI_Transmit(&hspi1, tx, 6, HAL_MAX_DELAY);
//    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_SET);
//}
//
//
//
//
///*
// * CONFIGURATION Fxns
// */
//
//
//
//void dwm_reset(void)
//{
//    HAL_GPIO_WritePin(DWM_RESET_N_GPIO_Port,
//                      DWM_RESET_N_Pin,
//                      GPIO_PIN_RESET);
//
//    HAL_Delay(1);   // 1ms is more than enough
//
//    // Release reset (Hi-Z because open-drain)
//    HAL_GPIO_WritePin(DWM_RESET_N_GPIO_Port,
//                      DWM_RESET_N_Pin,
//                      GPIO_PIN_SET);
//
//    HAL_Delay(5);   // allow DW1000 time to boot
//}
//
//void modify_default_config(void){
//
//	// TODO: Some of these registers are not 4 octets, could be an issue ?????
//	//1. AGC Tune1
//	uint32_t current_agc1_config = dwm_read_sub_reg_32(0x23,0x04);
//	dwm_write_sub_reg_32(0x23,0x04,0x8870);
//
//
//	//2.AGC Tune2
//	uint32_t current_agc2_config = dwm_read_sub_reg_32(0x23,0x0C);
//	dwm_write_sub_reg_32(0x23,0x0C,0x2502A907);
//
//
//	//3. DRX_Tune_2
//	uint32_t current_drx_2_config = dwm_read_sub_reg_32(0x27,0x08);
//	dwm_write_sub_reg_32(0x27,0x08,0x311A002D);
//
//
//	//4. NTM
//	uint32_t current_ntm_lde_1_config = dwm_read_sub_reg_32(0x2E,0x0806);
//	dwm_write_sub_reg_32(0x2E,0x0806,0xD);
//
//
//	//5. LDE Config 2
//	uint32_t current_lde_config_2 = dwm_read_sub_reg_32(0x2E,0x1806);
//	dwm_write_sub_reg_32(0x2E,0x1806,0x1607);
//
//	//6. Tx Power
//	uint32_t current_tx_power = dwm_read_sub_reg_32(0x1E,0x00);
//	dwm_write_sub_reg_32(0x1E,0x00,0x0E082848);
//
//	//7. RX_TXCTRL
//	uint32_t current_rx_txctrl = dwm_read_sub_reg_32(0x28,0x0C);
//	dwm_write_sub_reg_32(0x28,0x0C,0x001E3FE3);
//
//}
//
//
///*
// * OPERATIONS
// */
//
//void dwm_tx_test(void)
//{
//    uint8_t frame[] = {0x41, 0x88, 0x00, 0xCA, 0xDE, 0xBE, 0xEF};
//
//    // 1. Write frame to TX buffer (reg 0x09)
//    uint8_t tx_buf[1 + sizeof(frame)];
//    tx_buf[0] = 0x09 | (1 << 7);  // write to TX_BUFFER
//    memcpy(&tx_buf[1], frame, sizeof(frame));
//
//    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_RESET);
//    HAL_SPI_Transmit(&hspi1, tx_buf, sizeof(tx_buf), HAL_MAX_DELAY);
//    HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_SET);
//
//    // 2. Set frame length (TX_FCTRL = 0x08)
//    dwm_write_reg_32(0x08, sizeof(frame));
//
//    // 3. Start transmission (SYS_CTRL = 0x0D)
//    dwm_write_reg_32(0x0D, 0x00000002);  // TXSTRT
//}
//
//
//
//
//
