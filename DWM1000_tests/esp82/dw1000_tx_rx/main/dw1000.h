#ifndef DWM1000_ESP_H
#define DWM1000_ESP_H

#include <Arduino.h>
#include <SPI.h>

typedef struct {
    uint8_t cs_pin;
    uint8_t reset_pin;
} DWM_Module;

void dwm_reset(DWM_Module *module);

void dwm_read_reg(DWM_Module *module, uint8_t reg_id, uint8_t *data, uint8_t len);
void dwm_write_reg(DWM_Module *module, uint8_t reg_id, uint8_t *data, uint8_t len);

void dwm_read_reg_sub(DWM_Module *module, uint8_t reg_id, uint16_t subaddr, uint8_t *data, uint16_t len);
void dwm_write_reg_sub(DWM_Module *module, uint8_t reg_id, uint16_t subaddr, uint8_t *data, uint16_t len);

void dwm_configure(DWM_Module *module);
void dwm_basic_transmit(DWM_Module *module);

void printRegHex(uint8_t* data, size_t len, const char* info);

bool dwm_receive(DWM_Module* module, uint8_t* buffer, uint16_t len);

#endif
