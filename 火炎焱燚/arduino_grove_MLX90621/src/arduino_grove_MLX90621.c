#include "MLX90621.h"

static i2c device;

uint8_t i;

void MLX90621_get_eeprom(uint8_t *buffer)
{
    i2c_reg_read(device, MLX90621_EEPROM_ADDRESS, 0, buffer, 256);
}

void MLX90621_get_PTAT(uint16_t *ptat)
{
    static uint8_t condition[] = {0x02, 0x40, 0x00, 0x01};
    i2c_condition_read(device, MLX90621_IR_ARRAY_ADDRESS, condition, 4, (uint8_t *)ptat, 2);
}

void MLX90621_get_CPIX(int16_t *cpix)
{
    static uint8_t condition[] = {0x02, 0x41, 0x00, 0x01};
    i2c_condition_read(device, MLX90621_IR_ARRAY_ADDRESS, condition, 4, (uint8_t *)cpix, 2);
}

void MLX90621_get_config(uint16_t *config)
{
    static uint8_t condition[] = {0x02, 0x92, 0x00, 0x01};
    i2c_condition_read(device, MLX90621_IR_ARRAY_ADDRESS, condition, 4, (uint8_t *)config, 2);
}

void MLX90621_set_config(uint8_t *config)
{
    uint8_t data[] = {config[0] - 0x55, config[0], config[1] - 0x55, config[1] | 0x04};
    i2c_reg_write(device, MLX90621_IR_ARRAY_ADDRESS, 0x03, data, 4);
}

void MLX90621_get_trimming_value(uint8_t *trimming_value)
{
    static uint8_t condition[] = {0x02, 0x93, 0x00, 0x01};
    i2c_condition_read(device, MLX90621_IR_ARRAY_ADDRESS, condition, 4, trimming_value, 2);
}

void MLX90621_set_trimming_value(uint8_t *trimming_value)
{
    uint8_t data[] = {trimming_value[0] - 0xAA, trimming_value[0], trimming_value[1] - 0xAA, trimming_value[1]};
    i2c_reg_write(device, MLX90621_IR_ARRAY_ADDRESS, 0x04, data, 4);
}

void MLX90621_get_IR(uint8_t *IR_buffer){
    static uint8_t condition[] = {0x02, 0x00, 0x01, 0x40};
    i2c_condition_read(device, MLX90621_IR_ARRAY_ADDRESS, condition, 4, IR_buffer, 0x80);
}

int main()
{
    int cmd;
    device = i2c_open_device(0);

    while (1)
    {
        while ((MAILBOX_CMD_ADDR & 0x01) == 0)
            ;
        cmd = MAILBOX_CMD_ADDR;

        switch (cmd)
        {
        case CONFIG_IOP_SWITCH:
            //no operation
            break;

        case MLX90621_GET_EEPROM:
        {
            uint8_t eeprom_buffer[256];
            i = 0;
            MLX90621_get_eeprom(eeprom_buffer);
            do
            {
                MAILBOX_DATA(i) = (signed int)eeprom_buffer[i];
            } while (++i);
            break;
        }
        case MLX90621_GET_PTAT:
        {
            uint16_t ptat;
            MLX90621_get_PTAT(&ptat);
            MAILBOX_DATA(0) = (signed int)ptat;
            break;
        }
        case MLX90621_GET_CPIX:
        {
            int16_t cpix;
            MLX90621_get_CPIX(&cpix);
            MAILBOX_DATA(0) = (signed int)cpix;
            break;
        }
        case MLX90621_GET_CONFIG:
        {
            uint16_t config;
            MLX90621_get_config(&config);
            MAILBOX_DATA(0) = (signed int)config;
            break;
        }
        case MLX90621_SET_CONFIG:
        {
            uint16_t config = (uint16_t)MAILBOX_DATA(0);
            MLX90621_set_config((uint8_t *)&config);
        }
        case MLX90621_GET_TRIMMING_VALUE:
        {
            uint16_t trimming_value;
            MLX90621_get_trimming_value((uint8_t *)&trimming_value);
            MAILBOX_DATA(0) = (signed int)trimming_value;
            break;
        }
        case MLX90621_SET_TRIMMING_VALUE:
        {
            uint16_t trimming_value = (uint16_t)MAILBOX_DATA(0);
            MLX90621_set_trimming_value((uint8_t *)&trimming_value);
            break;
        }
        case MLX90621_GET_IR:
        {
            int16_t IR_buffer[0x40];
            MLX90621_get_IR((uint8_t*)IR_buffer);
            i = 0;
            do {
                MAILBOX_DATA(i) = (signed int)IR_buffer[i];
            } while(++i < 0x40);
        }
        default:
            break;
        }
        MAILBOX_CMD_ADDR = 0x0;
    }
}