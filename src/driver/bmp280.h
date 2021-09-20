/**
 * @file bmp280.h
 * @author LIN 
 * @brief Driver for BMP280 barometer module.
 * 
 * 
 * 
 * @version 0.1
 * @date 2021-08-20
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _BMP280_H_
#define _BMP280_H_

int bmp_init();

int bmp_read_pressure();

int bmp_read_temperature();

int bmp_reset();

#endif // _BMP280_H_
