#ifndef __TF_CARD_H
#define __TF_CARD_H

#include "ff.h"
#include "xstatus.h"
#include "xil_printf.h"


int platform_init_fs();
int sd_mount();
int sd_read_binary(char *file_name, u8 *src_addr, u32 bit_len, u32 max_lines);
int sd_read_hex(char *file_name, u8 *dest_buf, u32 max_lines);

#endif
