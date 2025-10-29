#include "tf_card.h"

static FATFS fatfs;


// initial file system
int platform_init_fs()
{
    FRESULT status;
    TCHAR *Path = "0:/";
    BYTE work[FF_MAX_SS];

    status = f_mount(&fatfs, Path, 1); // mount SD card
    if (status != FR_OK) {
        xil_printf("Volume is not FAT-formatted; formatting FAT\r\n");
        status = f_mkfs(Path, FM_FAT32, 0, work, sizeof work);
        if (status != FR_OK) {
            xil_printf("Unable to format FAT\r\n");
            return -1;
        }
        status = f_mount(&fatfs, Path, 1);
        if (status != FR_OK) {
            xil_printf("Unable to mount SD card\r\n");
            return -1;
        }
    }
    return 0;
}

int sd_mount()
{
    FRESULT status;
    status = platform_init_fs();
    if(status){
        xil_printf("ERROR: f_mount returned %d!\n",status);
        return XST_FAILURE;
    }
    return XST_SUCCESS;
}

/**
 * @brief Read binary data from files: process comment lines, and write data to the target memory in 8-bit format.
 * @param file_name: File name.
 * @param dest_buf: Target memory address (written in 8-bit format).
 * @param bit_len: Bit per row(32/20/6/5).
 * @param max_lines: Maximum number of rows.
 * @return Success returns 0, failure returns -1.
 */
int sd_read_binary(char *file_name, u8 *dest_buf, u32 bit_len, u32 max_lines) {
    FIL fil;
    FRESULT res;
    char line_buf[128];  // store row data
    int line_count = 0;
    int error = 0;

    // open file
    res = f_open(&fil, file_name, FA_READ);
    if (res != FR_OK) {
        printf("Error: Failed to open file.\n");
        return -1;
    }

    // Read files line by line
    while (f_gets(line_buf, sizeof(line_buf), &fil) != NULL && line_count < max_lines) {
        
        // Check the first bit_len chars
        int valid = 1;
        for (int i = 0; i < bit_len; i++) {
            if (line_buf[i] != '0' && line_buf[i] != '1') {
                valid = 0;
                break;
            }
        }
        
        if (!valid) {
            // printf("Warning: Skipping invalid line: %s\n", line_buf);
            continue;
        }

        // Convert 0/1 string to 32-bit data
        uint32_t value = 0;
        for (int i = 0; i < bit_len; i++) {
            if (line_buf[i] == '1') {
                value |= (1U << (bit_len - 1 - i));
            }
        }

        // Write 8 bits (1 byte) to the target memory (SPI transfer requirement)
        for (int i = 0; i < 4; i++) {
            dest_buf[line_count * 4 + i] = (value >> (24 - i * 8)) & 0xFF;
        }

        line_count++;
    }

    f_close(&fil);
    return error ? -1 : 0;
}

/**
 * @brief Read data from a hexadecimal file, convert it to 32 bits, and write it into a buffer (4 bytes).
 * @param file_name: File name.
 * @param dest_buf: Target memory address (written in 8-bit format).
 * @param max_lines: Maximum number of rows.
 * @return Success returns 0, failure returns -1.
 */
int sd_read_hex(char *file_name, u8 *dest_buf, u32 max_lines) {
    FIL fil;
    FRESULT res;
    char line_buf[32];  // store row data
    int line_count = 0;
    int error = 0;

    // open file
    res = f_open(&fil, file_name, FA_READ);
    if (res != FR_OK) {
        printf("Error: Failed to open file.\n");
        return -1;
    }

    // Read files line by line
    while (f_gets(line_buf, sizeof(line_buf), &fil) != NULL && line_count < max_lines) {

        // Convert hexadecimal string to 32-bit integer
        uint32_t value = strtoul(line_buf, NULL, 16);
        
        // Write to the target buffer in 4-byte chunks (big end)
        for (int i = 0; i < 4; i++) {
            dest_buf[line_count * 4 + i] = (value >> (24 - i * 8)) & 0xFF;
        }

        line_count++;
    }

    f_close(&fil);
    return error ? -1 : 0;
}
