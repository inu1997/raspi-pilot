/**
 * @file gpio.h
 * @author LIN 
 * @brief Raspberry Pi GPIO utilities.
 * 
 * 
 * 
 * @version 0.1
 * @date 2021-09-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _GPIO_H_
#define _GPIO_H_

int gpio_set_export(int index);

int gpio_set_unexport(int index);

int gpio_set_direction(int index, const char *dir);

int gpio_get_direction(int index, char *dest);

int gpio_write(int index, int val);

int gpio_read(int index, int *val);

#endif // _GPIO_H_
