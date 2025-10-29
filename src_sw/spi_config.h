#ifndef __SPI_CONFIG_H
#define __SPI_CONFIG_H

#include <stdio.h>
#include "tf_card.h"
#include "xspips.h"
#include "xparameters.h"


// file name
#define CONF_REG_FILE_NAME          "spi_config_reg.txt"
#define CONF_BLOCK_BANK_FILE_NAME   "block_idx_bank.dat"
#define CONF_ROW_BANK0_FILE_NAME    "row_cnt_bank0.dat"
#define CONF_ROW_BANK1_FILE_NAME    "row_cnt_bank1.dat"
#define CONF_ROW_BANK2_FILE_NAME    "row_cnt_bank2.dat"
#define CONF_ROW_BANK3_FILE_NAME    "row_cnt_bank3.dat"
#define CONF_ROW_BANK4_FILE_NAME    "row_cnt_bank4.dat"

#define CONF_CCL_BANK0_FILE_NAME    "col_cla_idx_bank0.dat"
#define CONF_CCL_BANK1_FILE_NAME    "col_cla_idx_bank1.dat"
#define CONF_CCL_BANK2_FILE_NAME    "col_cla_idx_bank2.dat"
#define CONF_CCL_BANK3_FILE_NAME    "col_cla_idx_bank3.dat"
#define CONF_CCL_BANK4_FILE_NAME    "col_cla_idx_bank4.dat"

#define CONF_WEIGHT_BANK_FILE_NAME  "weight_bank.dat"

// define file length
#define LEN_CONF_REG        28
#define LEN_BLOCK_BANK      1152
#define LEN_ROW_BANK0       1592
#define LEN_ROW_BANK1       1618
#define LEN_ROW_BANK2       1627
#define LEN_ROW_BANK3       1614
#define LEN_ROW_BANK4       1668

#define LEN_CCL_BANK0       3127
#define LEN_CCL_BANK1       3114
#define LEN_CCL_BANK2       3099
#define LEN_CCL_BANK3       3108
#define LEN_CCL_BANK4       3186

#define LEN_WEIGHT_BANK     1440


// declaration buffer
u8 Conf_reg_Buffer  [LEN_CONF_REG * 4];
u8 Conf_block_Buffer[LEN_BLOCK_BANK * 4];
u8 Conf_row_Buffer0 [LEN_ROW_BANK0  * 4];
u8 Conf_row_Buffer1 [LEN_ROW_BANK1  * 4];
u8 Conf_row_Buffer2 [LEN_ROW_BANK2  * 4];
u8 Conf_row_Buffer3 [LEN_ROW_BANK3  * 4];
u8 Conf_row_Buffer4 [LEN_ROW_BANK4  * 4];

u8 Conf_ccl_Buffer0 [LEN_CCL_BANK0  * 4];
u8 Conf_ccl_Buffer1 [LEN_CCL_BANK1  * 4];
u8 Conf_ccl_Buffer2 [LEN_CCL_BANK2  * 4];
u8 Conf_ccl_Buffer3 [LEN_CCL_BANK3  * 4];
u8 Conf_ccl_Buffer4 [LEN_CCL_BANK4  * 4];

u8 Conf_weight_Buffer[(LEN_WEIGHT_BANK) * 4];

u8 Conf_feature_bank_Buffer  [129 * 4];

uint32_t binary_str_to_uint32(char *str);
void read_model_data();
int initial_TMA(XSpiPs *SpiInstancePtr);
void SPIWrite(XSpiPs *SpiPtr, u32 Offset, u32 ByteCount, u8 *Buffer);


#endif