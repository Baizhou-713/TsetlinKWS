#ifndef __ADAU1761_H
#define __ADAU1761_H

#include "Init_ADAU1761.h"

#define FS 16000                    // Sample freq fs: 16khz
#define MCLK 8192000                // MCLK = 512*fs
#define ADAU1761_DEV_ADDR 0x3B      // AUDI1761 I2C address(7-bit) 0111011X
#define IIC_SCLK_RATE 100000        // I2C speed
#define MAX_BRUST_LEN PROGRAM_SIZE_ADAU1761 + 5

#endif