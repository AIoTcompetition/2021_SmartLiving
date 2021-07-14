#ifndef __MLX90621_H_
#define __MLX90621_H_

#include "circular_buffer.h"
#include "myi2c.h"

//device address
#define MLX90621_EEPROM_ADDRESS 0x50
#define MLX90621_IR_ARRAY_ADDRESS 0x60

//cmd
#define CONFIG_IOP_SWITCH 0x01
#define MLX90621_GET_EEPROM 0x03
#define MLX90621_GET_PTAT 0x05
#define MLX90621_GET_CPIX 0x07
#define MLX90621_GET_CONFIG 0x09
#define MLX90621_SET_CONFIG 0x0B
#define MLX90621_GET_TRIMMING_VALUE 0x0D
#define MLX90621_SET_TRIMMING_VALUE 0x0F
#define MLX90621_GET_IR 0x11

//function
void MLX90621_get_eeprom(uint8_t *buffer);
void MLX90621_get_PTAT(uint16_t *ptat);
void MLX90621_get_CPIX(int16_t *cpix);
void MLX90621_get_config(uint16_t *config);
void MLX90621_set_config(uint8_t *config);
void MLX90621_get_trimming_value(uint8_t *config);
void MLX90621_set_trimming_value(uint8_t *config);
void MLX90621_get_IR(uint8_t *IR_buffer);

#endif