#ifndef _MYI2C_H_
#define _MYI2C_H_
#ifdef __cplusplus 
extern "C" {
#endif
#include <xparameters.h>
#include <stdlib.h>

// #ifdef XPAR_XIIC_NUM_INSTANCES

/* 
 * IIC API
 */
typedef int i2c;

i2c i2c_open_device(unsigned int device);
#ifdef XPAR_IO_SWITCH_NUM_INSTANCES
#ifdef XPAR_IO_SWITCH_0_I2C0_BASEADDR
i2c i2c_open(unsigned int sda, unsigned int scl);
#endif
#endif
int i2c_read(i2c dev_id, unsigned int slave_address,
              unsigned char* buffer, unsigned int length);
int i2c_write(i2c dev_id, unsigned int slave_address,
               unsigned char* buffer, unsigned int length);
int i2c_condition_read(i2c dev_id, unsigned int slave_address,
             unsigned char* condition_data, unsigned int condition_length,
             unsigned char* buffer, unsigned int length);
int i2c_reg_read(i2c dev_id, unsigned int slave_address,
             unsigned char reg_address,
             unsigned char* buffer, unsigned int length);
int i2c_reg_write(i2c dev_id, unsigned int slave_address,
             unsigned char reg_address,
             unsigned char* buffer, unsigned int length);
void i2c_close(i2c dev_id);
unsigned int i2c_get_num_devices(void);

// #endif
#ifdef __cplusplus 
}
#endif
#endif  // _MYI2C_H_
