#include "spi_config.h"

char* BLOCK_IDX_BANK_CONFIG_ADDR    = "10010000010001111111000000000000";
char* ROW_CNT_BANK0_CONFIG_ADDR     = "10100000011000110111000000000000";
char* ROW_CNT_BANK1_CONFIG_ADDR     = "10100010011001010001000000000000";
char* ROW_CNT_BANK2_CONFIG_ADDR     = "10100100011001011010000000000000";
char* ROW_CNT_BANK3_CONFIG_ADDR     = "10100110011001001101000000000000";
char* ROW_CNT_BANK4_CONFIG_ADDR     = "10101000011010000011000000000000";

char* CCL_IDX_BANK0_CONFIG_ADDR     = "10110000110000110110000000000000";
char* CCL_IDX_BANK1_CONFIG_ADDR     = "10110010110000101001000000000000";
char* CCL_IDX_BANK2_CONFIG_ADDR     = "10110100110000011010000000000000";
char* CCL_IDX_BANK3_CONFIG_ADDR     = "10110110110000100011000000000000";
char* CCL_IDX_BANK4_CONFIG_ADDR     = "10111000110001110001000000000000";

char* WEIGHT_BANK0_CONFIG_ADDR      = "11000000010110011111000000000000";

char* CONF_SPI_EN_INF_ADDR          = "10000000000000000000000000000001";
char* CONF_SPI_EN_INF_DATA          = "00000000000000000000000000000001";


uint32_t binary_str_to_uint32(char *str) {
    uint32_t result = 0;
    for (int i = 0; i < 32; i++) {
        if (str[i] == '1') {
            result |= (1 << (31 - i));
        }
    }
    return result;
}


void read_model_data(){    
    sd_read_binary("feature_bank_data.txt",     (u8*)Conf_feature_bank_Buffer, 32, 129      );
    sd_read_binary(CONF_REG_FILE_NAME,          (u8*)Conf_reg_Buffer,   32, LEN_CONF_REG    );
    sd_read_binary(CONF_BLOCK_BANK_FILE_NAME,   (u8*)Conf_block_Buffer, 20, LEN_BLOCK_BANK  );
    sd_read_binary(CONF_ROW_BANK0_FILE_NAME,    (u8*)Conf_row_Buffer0,  6, LEN_ROW_BANK0    );
    sd_read_binary(CONF_ROW_BANK1_FILE_NAME,    (u8*)Conf_row_Buffer1,  6, LEN_ROW_BANK1    );
    sd_read_binary(CONF_ROW_BANK2_FILE_NAME,    (u8*)Conf_row_Buffer2,  6, LEN_ROW_BANK2    );
    sd_read_binary(CONF_ROW_BANK3_FILE_NAME,    (u8*)Conf_row_Buffer3,  6, LEN_ROW_BANK3    );
    sd_read_binary(CONF_ROW_BANK4_FILE_NAME,    (u8*)Conf_row_Buffer4,  6, LEN_ROW_BANK4    );
    
    sd_read_binary(CONF_CCL_BANK0_FILE_NAME,    (u8*)Conf_ccl_Buffer0,  5, LEN_CCL_BANK0    );
    sd_read_binary(CONF_CCL_BANK1_FILE_NAME,    (u8*)Conf_ccl_Buffer1,  5, LEN_CCL_BANK1    );
    sd_read_binary(CONF_CCL_BANK2_FILE_NAME,    (u8*)Conf_ccl_Buffer2,  5, LEN_CCL_BANK2    );
    sd_read_binary(CONF_CCL_BANK3_FILE_NAME,    (u8*)Conf_ccl_Buffer3,  5, LEN_CCL_BANK3    );
    sd_read_binary(CONF_CCL_BANK4_FILE_NAME,    (u8*)Conf_ccl_Buffer4,  5, LEN_CCL_BANK4    );
    
    sd_read_hex(CONF_WEIGHT_BANK_FILE_NAME,     (u8*)Conf_weight_Buffer, LEN_WEIGHT_BANK);
    
}


int initial_TMA(XSpiPs *SpiInstancePtr){
    uint32_t addr_uint32;
    uint8_t addr_uint8 [4];
    
    // configure configuration register
    SPIWrite(SpiInstancePtr, 0, LEN_CONF_REG * 4, Conf_reg_Buffer);
    
    // load model block index 
    addr_uint32 = binary_str_to_uint32(BLOCK_IDX_BANK_CONFIG_ADDR);
    for (int i = 0; i < 4; i++) {
        addr_uint8[i] = (addr_uint32 >> (24 - i * 8)) & 0xFF;
    }
    SPIWrite(SpiInstancePtr, 0, 4, addr_uint8);
    SPIWrite(SpiInstancePtr, 0, LEN_BLOCK_BANK * 4, Conf_block_Buffer);
    
    // load model row count
    // ------------------------bank0------------------------
    addr_uint32 = binary_str_to_uint32(ROW_CNT_BANK0_CONFIG_ADDR);
    for (int i = 0; i < 4; i++) {
        addr_uint8[i] = (addr_uint32 >> (24 - i * 8)) & 0xFF;
    }
    SPIWrite(SpiInstancePtr, 0, 4, addr_uint8);
    SPIWrite(SpiInstancePtr, 0, LEN_ROW_BANK0 * 4, Conf_row_Buffer0);
    // ------------------------bank1------------------------
    addr_uint32 = binary_str_to_uint32(ROW_CNT_BANK1_CONFIG_ADDR);
    for (int i = 0; i < 4; i++) {
        addr_uint8[i] = (addr_uint32 >> (24 - i * 8)) & 0xFF;
    }
    SPIWrite(SpiInstancePtr, 0, 4, addr_uint8);
    SPIWrite(SpiInstancePtr, 0, LEN_ROW_BANK1 * 4, Conf_row_Buffer1);
    // ------------------------bank2------------------------
    addr_uint32 = binary_str_to_uint32(ROW_CNT_BANK2_CONFIG_ADDR);
    for (int i = 0; i < 4; i++) {
        addr_uint8[i] = (addr_uint32 >> (24 - i * 8)) & 0xFF;
    }
    SPIWrite(SpiInstancePtr, 0, 4, addr_uint8);
    SPIWrite(SpiInstancePtr, 0, LEN_ROW_BANK2 * 4, Conf_row_Buffer2);
    // ------------------------bank3------------------------
    addr_uint32 = binary_str_to_uint32(ROW_CNT_BANK3_CONFIG_ADDR);
    for (int i = 0; i < 4; i++) {
        addr_uint8[i] = (addr_uint32 >> (24 - i * 8)) & 0xFF;
    }
    SPIWrite(SpiInstancePtr, 0, 4, addr_uint8);
    SPIWrite(SpiInstancePtr, 0, LEN_ROW_BANK3 * 4, Conf_row_Buffer3);
    // ------------------------bank4------------------------
    addr_uint32 = binary_str_to_uint32(ROW_CNT_BANK4_CONFIG_ADDR);
    for (int i = 0; i < 4; i++) {
        addr_uint8[i] = (addr_uint32 >> (24 - i * 8)) & 0xFF;
    }
    SPIWrite(SpiInstancePtr, 0, 4, addr_uint8);
    SPIWrite(SpiInstancePtr, 0, LEN_ROW_BANK4 * 4, Conf_row_Buffer4);
    

    // load model ccl index 
    // ------------------------bank0------------------------
    addr_uint32 = binary_str_to_uint32(CCL_IDX_BANK0_CONFIG_ADDR);
    for (int i = 0; i < 4; i++) {
        addr_uint8[i] = (addr_uint32 >> (24 - i * 8)) & 0xFF;
    }
    SPIWrite(SpiInstancePtr, 0, 4, addr_uint8);
    SPIWrite(SpiInstancePtr, 0, LEN_CCL_BANK0 * 4, Conf_ccl_Buffer0);
    // ------------------------bank1------------------------
    addr_uint32 = binary_str_to_uint32(CCL_IDX_BANK1_CONFIG_ADDR);
    for (int i = 0; i < 4; i++) {
        addr_uint8[i] = (addr_uint32 >> (24 - i * 8)) & 0xFF;
    }
    SPIWrite(SpiInstancePtr, 0, 4, addr_uint8);
    SPIWrite(SpiInstancePtr, 0, LEN_CCL_BANK1 * 4, Conf_ccl_Buffer1);
    // ------------------------bank2------------------------
    addr_uint32 = binary_str_to_uint32(CCL_IDX_BANK2_CONFIG_ADDR);
    for (int i = 0; i < 4; i++) {
        addr_uint8[i] = (addr_uint32 >> (24 - i * 8)) & 0xFF;
    }
    SPIWrite(SpiInstancePtr, 0, 4, addr_uint8);
    SPIWrite(SpiInstancePtr, 0, LEN_CCL_BANK2 * 4, Conf_ccl_Buffer2);
    // ------------------------bank3------------------------
    addr_uint32 = binary_str_to_uint32(CCL_IDX_BANK3_CONFIG_ADDR);
    for (int i = 0; i < 4; i++) {
        addr_uint8[i] = (addr_uint32 >> (24 - i * 8)) & 0xFF;
    }
    SPIWrite(SpiInstancePtr, 0, 4, addr_uint8);
    SPIWrite(SpiInstancePtr, 0, LEN_CCL_BANK3 * 4, Conf_ccl_Buffer3);
    // ------------------------bank4------------------------
    addr_uint32 = binary_str_to_uint32(CCL_IDX_BANK4_CONFIG_ADDR);
    for (int i = 0; i < 4; i++) {
        addr_uint8[i] = (addr_uint32 >> (24 - i * 8)) & 0xFF;
    }
    SPIWrite(SpiInstancePtr, 0, 4, addr_uint8);
    SPIWrite(SpiInstancePtr, 0, LEN_CCL_BANK4 * 4, Conf_ccl_Buffer4);
    
    // load model weight
    // ------------------------bank0------------------------
    addr_uint32 = binary_str_to_uint32(WEIGHT_BANK0_CONFIG_ADDR);
    for (int i = 0; i < 4; i++) {
        addr_uint8[i] = (addr_uint32 >> (24 - i * 8)) & 0xFF;
    }
    SPIWrite(SpiInstancePtr, 0, 4, addr_uint8);
    SPIWrite(SpiInstancePtr, 0, LEN_WEIGHT_BANK * 4, Conf_weight_Buffer);

    // load data to feature bank
    //SPIWrite(SpiInstancePtr, 0, 129 * 4, Conf_feature_bank_Buffer);
    
    // Start inference
    // addr
    addr_uint32 = binary_str_to_uint32(CONF_SPI_EN_INF_ADDR);
    for (int i = 0; i < 4; i++) {
        addr_uint8[i] = (addr_uint32 >> (24 - i * 8)) & 0xFF;
    }
    SPIWrite(SpiInstancePtr, 0, 4, addr_uint8);
    // data
    addr_uint32 = binary_str_to_uint32(CONF_SPI_EN_INF_DATA);
    for (int i = 0; i < 4; i++) {
        addr_uint8[i] = (addr_uint32 >> (24 - i * 8)) & 0xFF;
    }
    SPIWrite(SpiInstancePtr, 0, 4, addr_uint8);

    return XST_SUCCESS;
}


void SPIWrite(XSpiPs *SpiPtr, u32 Offset, u32 ByteCount, u8 *Buffer)
{   
    u8 *buffer_start;
    buffer_start = Buffer + Offset;
    XSpiPs_PolledTransfer(SpiPtr, buffer_start, NULL, ByteCount);
}
