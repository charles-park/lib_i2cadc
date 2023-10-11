//------------------------------------------------------------------------------
/**
 * @file lib_i2cadc.h
 * @author charles-park (charles.park@hardkernel.com)
 * @brief ADC board(LTC2309) control library for ODROID-JIG.
 * @version 0.2
 * @date 2023-10-11
 *
 * @package apt install minicom
 *
 * @copyright Copyright (c) 2022
 *
 */
//------------------------------------------------------------------------------
#ifndef __LIB_I2CADC_H__
#define __LIB_I2CADC_H__

//------------------------------------------------------------------------------
// function prototype
//------------------------------------------------------------------------------
extern int adc_board_read   (int fd, const char *name, int *read_value, int *cnt);
extern int adc_board_init   (const char *i2c_dev_node);

//------------------------------------------------------------------------------
#endif  // __LIB_I2CADC_H__

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------