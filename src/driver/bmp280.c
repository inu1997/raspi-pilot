#include "bmp280.h"
#include <stdint.h>

static uint8_t _dev_addr;
#define BMP_ADDRESS _dev_addr;
#define BMP_ADDRESS_SD0_HIGH 0x77
#define BMP_ADDRESS_SD0_LOW 0x76
#define BMP_TEMP_XLSB 0xfc
#define BMP_TEMP_LSB 0xfb
#define BMP_TEMP_MSB 0xfa
#define BMP_PRESS_XLSB 0xf9
#define BMP_PRESS_LSB 0xf8
#define BMP_PRESS_MSG 0xf7
#define BMP_CONFIG 0xf5
#define BMP_CTRL_MEAS 0xf4
#define BMP_STATUS 0xf3
#define BMP_RESET 0xe0
#define BMP_ID 0xd0
#define BMP_CALIB00 0x88
#define BMP_CALIB25 0xa1

int bmp_init() {
    return 0;
}

int bmp_read_pressure() {
    return 0;
}

int bmp_read_temperature() {
    return 0;
}

int bmp_reset() {
    return 0;
}
