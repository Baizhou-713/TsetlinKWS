//==============================================================================
// Copyright (c) 2024-2025 Baizhou Lin
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
// 
// Licensed under the Solderpad Hardware License v 2.1 (the “License”); 
// you may not use this file except in compliance with the License, or, 
// at your option, the Apache License version 2.0. 
// You may obtain a copy of the License at
// 
// https://solderpad.org/licenses/SHL-2.1/
// 
// Unless required by applicable law or agreed to in writing, any work 
// distributed under the License is distributed on an “AS IS” BASIS, 
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
// See the License for the specific language governing permissions and 
// limitations under the License.
//==============================================================================
//
// Project: TsetlinKWS - a keyword spotting accelerator based on Tsetlin Machine
//
// Module: "binarizer.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: Binarizing the MFSC-SF feature.
//
//==============================================================================

module binarizer #(
    parameter N_MEL                     = 32,
    parameter N_FRAME                   = 64,
    parameter BIT_WIDTH                 = 16
)(
    input logic                         clk,
    input logic                         rst_n,
    input logic                         valid_in,
    input logic [BIT_WIDTH-1:0]         mfcc_data_in,
    input logic [BIT_WIDTH-1:0]         flux_data_in,
    output logic                        ready_out,
    
    // spi_slave Configuration registers --------------------------------------
    input logic                         spi_en_fe_system_sync,
    input logic                         spi_en_inf_system_sync,
    input logic [15:0]                  SPI_FLUX_TH,
    
    // MFCC cricular buffer signals -------------------------------------------
    input logic [BIT_WIDTH-1:0]         mfcc_cirbuf_rdata0,
    input logic [BIT_WIDTH-1:0]         mfcc_cirbuf_rdata1,
    input logic [BIT_WIDTH-1:0]         mfcc_cirbuf_rdata2,
    input logic [BIT_WIDTH-1:0]         mfcc_cirbuf_rdata3,
    input logic [BIT_WIDTH-1:0]         mfcc_cirbuf_rdata4,
    input logic [BIT_WIDTH-1:0]         mfcc_cirbuf_rdata5,
    input logic [BIT_WIDTH-1:0]         mfcc_cirbuf_rdata6,
    input logic [BIT_WIDTH-1:0]         mfcc_cirbuf_rdata7,
    output logic                        MEM_MFCC_CIRBUF_BANK_CEB,
    output logic                        MEM_MFCC_CIRBUF_BANK0_WEB,
    output logic                        MEM_MFCC_CIRBUF_BANK1_WEB,
    output logic                        MEM_MFCC_CIRBUF_BANK2_WEB,
    output logic                        MEM_MFCC_CIRBUF_BANK3_WEB,
    output logic                        MEM_MFCC_CIRBUF_BANK4_WEB,
    output logic                        MEM_MFCC_CIRBUF_BANK5_WEB,
    output logic                        MEM_MFCC_CIRBUF_BANK6_WEB,
    output logic                        MEM_MFCC_CIRBUF_BANK7_WEB,
    output logic [$clog2(8*N_MEL)-1:0]  MEM_MFCC_CIRBUF_BANK_A,
    output logic [BIT_WIDTH-1:0]        MEM_MFCC_CIRBUF_WDATA,
    
    // FLUX cricular buffer signals -------------------------------------------
    input logic [N_FRAME-1:0]           flux_cirbuf_rdata,
    output logic                        MEM_FLUX_CIRBUF_BANK_CEB,
    output logic                        MEM_FLUX_CIRBUF_BANK_WEB,
    output logic [N_FRAME-1:0]          MEM_FLUX_CIRBUF_BANK_BWEB,
    output logic [$clog2(N_MEL)-1:0]    MEM_FLUX_CIRBUF_BANK_A,
    output logic [N_FRAME-1:0]          MEM_FLUX_CIRBUF_BANK_D,
    
    // Feature bank signals ---------------------------------------------------
    output logic                        MEM_FEBANK_CEB_binarizer,
    output logic                        MEM_FEBANK_WEB_binarizer,
    output logic [$clog2(2*N_MEL)-1:0]  MEM_FEBANK_A_binarizer,
    output logic [N_FRAME-1:0]          MEM_FEBANK_D_binarizer,
    
    // Accelerator signals ----------------------------------------------------
    output logic                        fe_complete
);
    
    typedef enum logic [2:0] {idle, padding, send_mfcc, send_finish, idle2, receive} state_t;
    
    // FSM signals
    state_t                     p_state, n_state;
    logic                       th_sub_zero_en;
    logic                       send_mfcc_en;
    logic                       send_flux_en;
    logic                       col_offset_inc_en;
    logic                       fsm_fe_complete;
                            
    logic                       spi_en_inf_system_pulse;
    logic                       spi_en_inf_system_sync_d1;
    logic                       spi_fe_complete;
    
    // handshaking signals
    logic                       handshaking_flag, handshaking_flag_d1;
    
    // threshold
    logic [15:0]                flux_threshold;
    
    // memory pointers
    logic [$clog2(N_MEL)-1:0]   row_wcnt;
    logic [$clog2(N_FRAME)-1:0] col_wcnt;
    logic [$clog2(N_MEL)-1:0]   mfcc_cirbuf_row_wptr;
    logic [$clog2(N_FRAME)-1:0] mfcc_cirbuf_col_wptr;
    logic [$clog2(N_MEL)-1:0]   mfcc_cirbuf_row_rptr;
    logic [$clog2(N_FRAME)-1:0] mfcc_cirbuf_col_rptr;
    logic [$clog2(8*N_MEL)-1:0] mfcc_cirbuf_rwptr;
    logic [$clog2(N_MEL)-1:0]   flux_cirbuf_row_wptr;
    logic [$clog2(N_FRAME)-1:0] flux_cirbuf_col_wptr;
    logic [$clog2(N_FRAME)-1:0] col_offset_cnt;
    logic [2:0]                 send_mfcc_col_rcnt;
    logic [2:0]                 send_mfcc_col_rcnt_d1;
    logic [$clog2(N_MEL)-1:0]   send_mfcc_row_rptr;
    logic [$clog2(N_MEL)-1:0]   send_mfcc_row_rptr_d1;
    logic [$clog2(8*N_MEL)-1:0] send_mfcc_cirbuf_rptr;
    logic [$clog2(N_MEL)-1:0]   send_flux_row_rptr;
    logic                       send_mfcc_en_d1;
    logic [$clog2(2*N_MEL)-1:0] feature_bank_row_wptr;
    
    // memory signals
    logic                       send_mfcc_wen;
    logic                       send_flux_wen;
    logic                       flux_cirbuf_wdata;
    logic                       MEM_FLUX_CIRBUF_BANK_D_int;
    
    // binarization signals
    logic [BIT_WIDTH-1:0]       threshold_bank          [0:N_MEL-1];            // RF (w16d32)
    logic [BIT_WIDTH-1:0]       threshold_sub_value;
    logic [BIT_WIDTH-1:0]       threshold_ori_value;
    logic [BIT_WIDTH-1:0]       mfcc_threshold_value;
    logic                       padding_threshold_bank;
    logic                       mfcc_bin_0, mfcc_bin_1, mfcc_bin_2, mfcc_bin_3;
    logic                       mfcc_bin_4, mfcc_bin_5, mfcc_bin_6, mfcc_bin_7;
    logic [7:0]                 mfcc_bin_spad;
    logic [N_FRAME-1:0]         mfcc_bin_reg, mfcc_bin;
    logic [N_FRAME-1:0]         flux_bin;
    logic [N_FRAME-1:0]         feature_bin;
    logic                       MEM_TH_BANK_CEB;
    logic                       MEM_TH_BANK_WEB;
    logic [$clog2(N_MEL)-1:0]   MEM_TH_BANK_A;
    logic [BIT_WIDTH-1:0]       MEM_TH_BANK_D;
    logic [BIT_WIDTH-1:0]       MEM_TH_BANK_Q;
    
    // assign for fe_complete
    assign fe_complete = fsm_fe_complete || spi_fe_complete;
    
    // assign for configuration registers
    assign flux_threshold = SPI_FLUX_TH;
    
    // Memory address logic
    assign MEM_MFCC_CIRBUF_BANK_CEB     = !(valid_in | handshaking_flag_d1 | send_mfcc_en); // 1st cycle:read, 2nd cycle:write
    assign MEM_MFCC_CIRBUF_BANK0_WEB    = !(handshaking_flag_d1 && col_wcnt[2:0] == 3'd0);
    assign MEM_MFCC_CIRBUF_BANK1_WEB    = !(handshaking_flag_d1 && col_wcnt[2:0] == 3'd1);
    assign MEM_MFCC_CIRBUF_BANK2_WEB    = !(handshaking_flag_d1 && col_wcnt[2:0] == 3'd2);
    assign MEM_MFCC_CIRBUF_BANK3_WEB    = !(handshaking_flag_d1 && col_wcnt[2:0] == 3'd3);
    assign MEM_MFCC_CIRBUF_BANK4_WEB    = !(handshaking_flag_d1 && col_wcnt[2:0] == 3'd4);
    assign MEM_MFCC_CIRBUF_BANK5_WEB    = !(handshaking_flag_d1 && col_wcnt[2:0] == 3'd5);
    assign MEM_MFCC_CIRBUF_BANK6_WEB    = !(handshaking_flag_d1 && col_wcnt[2:0] == 3'd6);
    assign MEM_MFCC_CIRBUF_BANK7_WEB    = !(handshaking_flag_d1 && col_wcnt[2:0] == 3'd7);
    assign MEM_MFCC_CIRBUF_BANK_A       = (send_mfcc_en) ? send_mfcc_cirbuf_rptr : mfcc_cirbuf_rwptr;
    
    assign MEM_FLUX_CIRBUF_BANK_CEB     = !(valid_in | handshaking_flag_d1);
    assign MEM_FLUX_CIRBUF_BANK_WEB     = !(handshaking_flag_d1);
    assign MEM_FLUX_CIRBUF_BANK_BWEB    = ~(1 << flux_cirbuf_col_wptr);
    assign MEM_FLUX_CIRBUF_BANK_A       = flux_cirbuf_row_wptr;
    

    assign MEM_FEBANK_CEB_binarizer     = !(send_mfcc_wen | send_flux_wen);
    assign MEM_FEBANK_WEB_binarizer     = !(send_mfcc_wen | send_flux_wen);
    assign MEM_FEBANK_A_binarizer       = feature_bank_row_wptr;
    assign MEM_FEBANK_D_binarizer       = feature_bin;
    
    // Modify the specity bit and writeback. When using SRAM macro, MEM_FLUX_CIRBUF_BANK_BWEB can replace this function.
    always_comb begin
        MEM_FLUX_CIRBUF_BANK_D = flux_cirbuf_rdata;
        MEM_FLUX_CIRBUF_BANK_D[flux_cirbuf_col_wptr] = MEM_FLUX_CIRBUF_BANK_D_int;
    end
    // ASIC version
    //assign MEM_FLUX_CIRBUF_BANK_D = {15'd0, MEM_FLUX_CIRBUF_BANK_D_int} << flux_cirbuf_col_wptr;
    

    // Circular buffer write address generation logic
    assign mfcc_cirbuf_row_wptr = row_wcnt;
    assign mfcc_cirbuf_col_wptr = col_wcnt;
    assign flux_cirbuf_row_wptr = row_wcnt;
    assign flux_cirbuf_col_wptr = col_wcnt;
    
    // Circular buffer read address generation logic
    assign mfcc_cirbuf_row_rptr = mfcc_cirbuf_row_wptr;   // read and write at the same position
    assign mfcc_cirbuf_col_rptr = mfcc_cirbuf_col_wptr;
    assign send_flux_row_rptr = flux_cirbuf_row_wptr;
    
    // Handshaking Circuit
    assign handshaking_flag = (valid_in && ready_out);
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            ready_out <= '1;
        end else if (valid_in && ready_out) begin
            ready_out <= '0;
        end else begin
            ready_out <= '1;
        end
    end
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            handshaking_flag_d1 <= 0;
        end else begin
            handshaking_flag_d1 <= handshaking_flag;
        end
    end
    
    // Memory pointers update
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            row_wcnt <= '0;
        end else if (!spi_en_inf_system_sync) begin
            row_wcnt <= '0;
        end else if (handshaking_flag_d1) begin
            row_wcnt <= row_wcnt + 1;
        end
    end
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            col_wcnt <= '0;
        end else if (!spi_en_inf_system_sync) begin
            col_wcnt <= '0;
        end else if (handshaking_flag_d1 && row_wcnt == N_MEL-1) begin
            col_wcnt <= col_wcnt + 1;
        end
    end
    
    // Before write the new data into the cir-mfcc-buffer, read the original data out.
    always_comb begin
        unique case(mfcc_cirbuf_col_wptr[2:0])
            0   :   threshold_sub_value = mfcc_cirbuf_rdata0;
            1   :   threshold_sub_value = mfcc_cirbuf_rdata1;
            2   :   threshold_sub_value = mfcc_cirbuf_rdata2;
            3   :   threshold_sub_value = mfcc_cirbuf_rdata3;
            4   :   threshold_sub_value = mfcc_cirbuf_rdata4;
            5   :   threshold_sub_value = mfcc_cirbuf_rdata5;
            6   :   threshold_sub_value = mfcc_cirbuf_rdata6;
            7   :   threshold_sub_value = mfcc_cirbuf_rdata7;
            default:threshold_sub_value = 0;
        endcase
        if (th_sub_zero_en || send_mfcc_en_d1) begin
            threshold_sub_value = 0;
        end
    end
    
    // According to the pointer, calculate the mfcc-cir-buffer address
    assign mfcc_cirbuf_rwptr        = (mfcc_cirbuf_col_wptr[5:3] << 5) | mfcc_cirbuf_row_wptr;
    assign send_mfcc_cirbuf_rptr    = (send_mfcc_col_rcnt        << 5) | send_mfcc_row_rptr;
    
    // When handshaking, read the data.
    assign flux_cirbuf_wdata = (flux_data_in >= flux_threshold) ? 1'b1 : 1'b0;
    
    always_ff @(posedge clk) begin
        if (handshaking_flag)
            MEM_MFCC_CIRBUF_WDATA <= mfcc_data_in;
    end
    
    always_ff @(posedge clk) begin
        if (handshaking_flag)
            MEM_FLUX_CIRBUF_BANK_D_int <= flux_cirbuf_wdata;
    end
    
    // Update mfcc threshold
    // Readout depends on send_mfcc & receive, and write depends on receive delay one cycle.
    assign MEM_TH_BANK_CEB      = !(handshaking_flag | handshaking_flag_d1 | send_mfcc_en);
    assign MEM_TH_BANK_WEB      = !(handshaking_flag_d1);
    assign MEM_TH_BANK_A        = (send_mfcc_en)? send_mfcc_row_rptr : mfcc_cirbuf_row_wptr;
    assign MEM_TH_BANK_D        = threshold_ori_value + (mfcc_data_in >> 6) - (threshold_sub_value >> 6);
    assign threshold_ori_value  = padding_threshold_bank ? 0 : MEM_TH_BANK_Q;
    assign mfcc_threshold_value = MEM_TH_BANK_Q;
    
    always_ff @(posedge clk) begin
        if (!MEM_TH_BANK_CEB) begin
            if (!MEM_TH_BANK_WEB)
                threshold_bank[MEM_TH_BANK_A] <= MEM_TH_BANK_D;
            else
                MEM_TH_BANK_Q <= threshold_bank[MEM_TH_BANK_A];
        end
    end
    
    // Give a initial signal to the threshold bank.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)             padding_threshold_bank <= 1;
        else if (col_wcnt != 0) padding_threshold_bank <= 0;
    end
    
    //-------------------------------------------------------------------------
    // Circular Buffer binary logic (MFCC use 8 comparators, FLUX use a 32x64 sram)
    //-------------------------------------------------------------------------
    assign send_mfcc_wen = (send_mfcc_col_rcnt_d1 == 3'd7);
    assign send_flux_wen = send_flux_en;
    
    // Binarize MFCC data
    assign mfcc_bin_0 = ( mfcc_cirbuf_rdata0 >= mfcc_threshold_value ) ? 1'b1 : 1'b0;
    assign mfcc_bin_1 = ( mfcc_cirbuf_rdata1 >= mfcc_threshold_value ) ? 1'b1 : 1'b0;
    assign mfcc_bin_2 = ( mfcc_cirbuf_rdata2 >= mfcc_threshold_value ) ? 1'b1 : 1'b0;
    assign mfcc_bin_3 = ( mfcc_cirbuf_rdata3 >= mfcc_threshold_value ) ? 1'b1 : 1'b0;
    assign mfcc_bin_4 = ( mfcc_cirbuf_rdata4 >= mfcc_threshold_value ) ? 1'b1 : 1'b0;
    assign mfcc_bin_5 = ( mfcc_cirbuf_rdata5 >= mfcc_threshold_value ) ? 1'b1 : 1'b0;
    assign mfcc_bin_6 = ( mfcc_cirbuf_rdata6 >= mfcc_threshold_value ) ? 1'b1 : 1'b0;
    assign mfcc_bin_7 = ( mfcc_cirbuf_rdata7 >= mfcc_threshold_value ) ? 1'b1 : 1'b0;
    
    assign mfcc_bin_spad = {mfcc_bin_7, mfcc_bin_6, mfcc_bin_5, mfcc_bin_4, 
                            mfcc_bin_3, mfcc_bin_2, mfcc_bin_1, mfcc_bin_0};
    
    assign feature_bin = send_mfcc_en_d1 ? mfcc_bin:
                            send_flux_en ? flux_bin : '0;
                
    assign feature_bank_row_wptr = send_mfcc_en_d1 ? send_mfcc_row_rptr_d1:
                                    send_flux_en ? send_flux_row_rptr + 6'd32: '0;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            send_mfcc_en_d1 <= 0;
        else
            send_mfcc_en_d1 <= send_mfcc_en;
    end
    
    // Use to counter which 8 data in a row should be read parallel
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            send_mfcc_col_rcnt <= '0;
        else if (!spi_en_inf_system_sync)
            send_mfcc_col_rcnt <= '0;
        else if (send_mfcc_en)
            send_mfcc_col_rcnt <= send_mfcc_col_rcnt + 1;
    end
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            send_mfcc_col_rcnt_d1 <= '0;
        else
            send_mfcc_col_rcnt_d1 <= send_mfcc_col_rcnt;
    end
    
    // Use to counter which row should be read
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            send_mfcc_row_rptr <= '0;
        else if (!spi_en_inf_system_sync)
            send_mfcc_row_rptr <= '0;
        else if (send_mfcc_col_rcnt == 3'd7)
            send_mfcc_row_rptr <= send_mfcc_row_rptr + 1;
    end
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            send_mfcc_row_rptr_d1 <= '0;
        else
            send_mfcc_row_rptr_d1 <= send_mfcc_row_rptr;
    end
    
    // Use to record which col is the first frame (0-63)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            col_offset_cnt <= '0;
        else if (!spi_en_inf_system_sync)
            col_offset_cnt <= '0;
        else if (col_offset_inc_en)
            col_offset_cnt <= col_offset_cnt + 1;
    end
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            mfcc_bin_reg <= '0;
        else if (send_mfcc_en_d1) begin
            unique case (send_mfcc_col_rcnt_d1)
                0   :   mfcc_bin_reg[ 7: 0] <= mfcc_bin_spad;
                1   :   mfcc_bin_reg[15: 8] <= mfcc_bin_spad;
                2   :   mfcc_bin_reg[23:16] <= mfcc_bin_spad;
                3   :   mfcc_bin_reg[31:24] <= mfcc_bin_spad;
                4   :   mfcc_bin_reg[39:32] <= mfcc_bin_spad;
                5   :   mfcc_bin_reg[47:40] <= mfcc_bin_spad;
                6   :   mfcc_bin_reg[55:48] <= mfcc_bin_spad;
                7   :   mfcc_bin_reg[63:56] <= mfcc_bin_spad;
                default:;
            endcase
        end
    end
    
    // MFCC rotation selection
    always_comb begin
        unique case(col_offset_cnt)
            0   :   mfcc_bin = {mfcc_bin_spad, mfcc_bin_reg[55:0]};
            1   :   mfcc_bin = {mfcc_bin_reg[0], mfcc_bin_spad, mfcc_bin_reg[55:1]};
            2   :   mfcc_bin = {mfcc_bin_reg[ 1:0], mfcc_bin_spad, mfcc_bin_reg[55: 2]};
            3   :   mfcc_bin = {mfcc_bin_reg[ 2:0], mfcc_bin_spad, mfcc_bin_reg[55: 3]};
            4   :   mfcc_bin = {mfcc_bin_reg[ 3:0], mfcc_bin_spad, mfcc_bin_reg[55: 4]};
            5   :   mfcc_bin = {mfcc_bin_reg[ 4:0], mfcc_bin_spad, mfcc_bin_reg[55: 5]};
            6   :   mfcc_bin = {mfcc_bin_reg[ 5:0], mfcc_bin_spad, mfcc_bin_reg[55: 6]};
            7   :   mfcc_bin = {mfcc_bin_reg[ 6:0], mfcc_bin_spad, mfcc_bin_reg[55: 7]};
            8   :   mfcc_bin = {mfcc_bin_reg[ 7:0], mfcc_bin_spad, mfcc_bin_reg[55: 8]};
            9   :   mfcc_bin = {mfcc_bin_reg[ 8:0], mfcc_bin_spad, mfcc_bin_reg[55: 9]};
            10  :   mfcc_bin = {mfcc_bin_reg[ 9:0], mfcc_bin_spad, mfcc_bin_reg[55:10]};
            11  :   mfcc_bin = {mfcc_bin_reg[10:0], mfcc_bin_spad, mfcc_bin_reg[55:11]};
            12  :   mfcc_bin = {mfcc_bin_reg[11:0], mfcc_bin_spad, mfcc_bin_reg[55:12]};
            13  :   mfcc_bin = {mfcc_bin_reg[12:0], mfcc_bin_spad, mfcc_bin_reg[55:13]};
            14  :   mfcc_bin = {mfcc_bin_reg[13:0], mfcc_bin_spad, mfcc_bin_reg[55:14]};
            15  :   mfcc_bin = {mfcc_bin_reg[14:0], mfcc_bin_spad, mfcc_bin_reg[55:15]};
            16  :   mfcc_bin = {mfcc_bin_reg[15:0], mfcc_bin_spad, mfcc_bin_reg[55:16]};
            17  :   mfcc_bin = {mfcc_bin_reg[16:0], mfcc_bin_spad, mfcc_bin_reg[55:17]};
            18  :   mfcc_bin = {mfcc_bin_reg[17:0], mfcc_bin_spad, mfcc_bin_reg[55:18]};
            19  :   mfcc_bin = {mfcc_bin_reg[18:0], mfcc_bin_spad, mfcc_bin_reg[55:19]};
            20  :   mfcc_bin = {mfcc_bin_reg[19:0], mfcc_bin_spad, mfcc_bin_reg[55:20]};
            21  :   mfcc_bin = {mfcc_bin_reg[20:0], mfcc_bin_spad, mfcc_bin_reg[55:21]};
            22  :   mfcc_bin = {mfcc_bin_reg[21:0], mfcc_bin_spad, mfcc_bin_reg[55:22]};
            23  :   mfcc_bin = {mfcc_bin_reg[22:0], mfcc_bin_spad, mfcc_bin_reg[55:23]};
            24  :   mfcc_bin = {mfcc_bin_reg[23:0], mfcc_bin_spad, mfcc_bin_reg[55:24]};
            25  :   mfcc_bin = {mfcc_bin_reg[24:0], mfcc_bin_spad, mfcc_bin_reg[55:25]};
            26  :   mfcc_bin = {mfcc_bin_reg[25:0], mfcc_bin_spad, mfcc_bin_reg[55:26]};
            27  :   mfcc_bin = {mfcc_bin_reg[26:0], mfcc_bin_spad, mfcc_bin_reg[55:27]};
            28  :   mfcc_bin = {mfcc_bin_reg[27:0], mfcc_bin_spad, mfcc_bin_reg[55:28]};
            29  :   mfcc_bin = {mfcc_bin_reg[28:0], mfcc_bin_spad, mfcc_bin_reg[55:29]};
            30  :   mfcc_bin = {mfcc_bin_reg[29:0], mfcc_bin_spad, mfcc_bin_reg[55:30]};
            31  :   mfcc_bin = {mfcc_bin_reg[30:0], mfcc_bin_spad, mfcc_bin_reg[55:31]};
            32  :   mfcc_bin = {mfcc_bin_reg[31:0], mfcc_bin_spad, mfcc_bin_reg[55:32]};
            33  :   mfcc_bin = {mfcc_bin_reg[32:0], mfcc_bin_spad, mfcc_bin_reg[55:33]};
            34  :   mfcc_bin = {mfcc_bin_reg[33:0], mfcc_bin_spad, mfcc_bin_reg[55:34]};
            35  :   mfcc_bin = {mfcc_bin_reg[34:0], mfcc_bin_spad, mfcc_bin_reg[55:35]};
            36  :   mfcc_bin = {mfcc_bin_reg[35:0], mfcc_bin_spad, mfcc_bin_reg[55:36]};
            37  :   mfcc_bin = {mfcc_bin_reg[36:0], mfcc_bin_spad, mfcc_bin_reg[55:37]};
            38  :   mfcc_bin = {mfcc_bin_reg[37:0], mfcc_bin_spad, mfcc_bin_reg[55:38]};
            39  :   mfcc_bin = {mfcc_bin_reg[38:0], mfcc_bin_spad, mfcc_bin_reg[55:39]};
            40  :   mfcc_bin = {mfcc_bin_reg[39:0], mfcc_bin_spad, mfcc_bin_reg[55:40]};
            41  :   mfcc_bin = {mfcc_bin_reg[40:0], mfcc_bin_spad, mfcc_bin_reg[55:41]};
            42  :   mfcc_bin = {mfcc_bin_reg[41:0], mfcc_bin_spad, mfcc_bin_reg[55:42]};
            43  :   mfcc_bin = {mfcc_bin_reg[42:0], mfcc_bin_spad, mfcc_bin_reg[55:43]};
            44  :   mfcc_bin = {mfcc_bin_reg[43:0], mfcc_bin_spad, mfcc_bin_reg[55:44]};
            45  :   mfcc_bin = {mfcc_bin_reg[44:0], mfcc_bin_spad, mfcc_bin_reg[55:45]};
            46  :   mfcc_bin = {mfcc_bin_reg[45:0], mfcc_bin_spad, mfcc_bin_reg[55:46]};
            47  :   mfcc_bin = {mfcc_bin_reg[46:0], mfcc_bin_spad, mfcc_bin_reg[55:47]};
            48  :   mfcc_bin = {mfcc_bin_reg[47:0], mfcc_bin_spad, mfcc_bin_reg[55:48]};
            49  :   mfcc_bin = {mfcc_bin_reg[48:0], mfcc_bin_spad, mfcc_bin_reg[55:49]};
            50  :   mfcc_bin = {mfcc_bin_reg[49:0], mfcc_bin_spad, mfcc_bin_reg[55:50]};
            51  :   mfcc_bin = {mfcc_bin_reg[50:0], mfcc_bin_spad, mfcc_bin_reg[55:51]};
            52  :   mfcc_bin = {mfcc_bin_reg[51:0], mfcc_bin_spad, mfcc_bin_reg[55:52]};
            53  :   mfcc_bin = {mfcc_bin_reg[52:0], mfcc_bin_spad, mfcc_bin_reg[55:53]};
            54  :   mfcc_bin = {mfcc_bin_reg[53:0], mfcc_bin_spad, mfcc_bin_reg[55:54]};
            55  :   mfcc_bin = {mfcc_bin_reg[54:0], mfcc_bin_spad, mfcc_bin_reg[55]};
            56  :   mfcc_bin = {mfcc_bin_reg[55:0], mfcc_bin_spad};
            57  :   mfcc_bin = {mfcc_bin_spad[  0], mfcc_bin_reg[55:0], mfcc_bin_spad[7:1]};
            58  :   mfcc_bin = {mfcc_bin_spad[1:0], mfcc_bin_reg[55:0], mfcc_bin_spad[7:2]};
            59  :   mfcc_bin = {mfcc_bin_spad[2:0], mfcc_bin_reg[55:0], mfcc_bin_spad[7:3]};
            60  :   mfcc_bin = {mfcc_bin_spad[3:0], mfcc_bin_reg[55:0], mfcc_bin_spad[7:4]};
            61  :   mfcc_bin = {mfcc_bin_spad[4:0], mfcc_bin_reg[55:0], mfcc_bin_spad[7:5]};
            62  :   mfcc_bin = {mfcc_bin_spad[5:0], mfcc_bin_reg[55:0], mfcc_bin_spad[7:6]};
            63  :   mfcc_bin = {mfcc_bin_spad[6:0], mfcc_bin_reg[55:0], mfcc_bin_spad[7]};
            default:mfcc_bin = '0;
        endcase
    end

    // FLUX rotation selection
    always_comb begin
        unique case(col_offset_cnt)
            0   :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[62:0]};    // write 63
            1   :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[63:1]};    // write 0
            2   :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[   0], flux_cirbuf_rdata[63: 2]};   // write 1
            3   :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[ 1:0], flux_cirbuf_rdata[63: 3]};   // write 2
            4   :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[ 2:0], flux_cirbuf_rdata[63: 4]};
            5   :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[ 3:0], flux_cirbuf_rdata[63: 5]};
            6   :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[ 4:0], flux_cirbuf_rdata[63: 6]};
            7   :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[ 5:0], flux_cirbuf_rdata[63: 7]};
            8   :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[ 6:0], flux_cirbuf_rdata[63: 8]};
            9   :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[ 7:0], flux_cirbuf_rdata[63: 9]};
            10  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[ 8:0], flux_cirbuf_rdata[63:10]};
            11  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[ 9:0], flux_cirbuf_rdata[63:11]};
            12  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[10:0], flux_cirbuf_rdata[63:12]};
            13  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[11:0], flux_cirbuf_rdata[63:13]};
            14  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[12:0], flux_cirbuf_rdata[63:14]};
            15  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[13:0], flux_cirbuf_rdata[63:15]};
            16  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[14:0], flux_cirbuf_rdata[63:16]};
            17  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[15:0], flux_cirbuf_rdata[63:17]};
            18  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[16:0], flux_cirbuf_rdata[63:18]};
            19  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[17:0], flux_cirbuf_rdata[63:19]};
            20  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[18:0], flux_cirbuf_rdata[63:20]};
            21  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[19:0], flux_cirbuf_rdata[63:21]};
            22  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[20:0], flux_cirbuf_rdata[63:22]};
            23  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[21:0], flux_cirbuf_rdata[63:23]};
            24  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[22:0], flux_cirbuf_rdata[63:24]};
            25  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[23:0], flux_cirbuf_rdata[63:25]};
            26  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[24:0], flux_cirbuf_rdata[63:26]};
            27  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[25:0], flux_cirbuf_rdata[63:27]};
            28  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[26:0], flux_cirbuf_rdata[63:28]};
            29  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[27:0], flux_cirbuf_rdata[63:29]};
            30  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[28:0], flux_cirbuf_rdata[63:30]};
            31  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[29:0], flux_cirbuf_rdata[63:31]};
            32  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[30:0], flux_cirbuf_rdata[63:32]};
            33  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[31:0], flux_cirbuf_rdata[63:33]};
            34  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[32:0], flux_cirbuf_rdata[63:34]};
            35  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[33:0], flux_cirbuf_rdata[63:35]};
            36  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[34:0], flux_cirbuf_rdata[63:36]};
            37  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[35:0], flux_cirbuf_rdata[63:37]};
            38  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[36:0], flux_cirbuf_rdata[63:38]};
            39  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[37:0], flux_cirbuf_rdata[63:39]};
            40  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[38:0], flux_cirbuf_rdata[63:40]};
            41  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[39:0], flux_cirbuf_rdata[63:41]};
            42  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[40:0], flux_cirbuf_rdata[63:42]};
            43  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[41:0], flux_cirbuf_rdata[63:43]};
            44  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[42:0], flux_cirbuf_rdata[63:44]};
            45  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[43:0], flux_cirbuf_rdata[63:45]};
            46  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[44:0], flux_cirbuf_rdata[63:46]};
            47  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[45:0], flux_cirbuf_rdata[63:47]};
            48  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[46:0], flux_cirbuf_rdata[63:48]};
            49  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[47:0], flux_cirbuf_rdata[63:49]};
            50  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[48:0], flux_cirbuf_rdata[63:50]};
            51  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[49:0], flux_cirbuf_rdata[63:51]};
            52  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[50:0], flux_cirbuf_rdata[63:52]};
            53  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[51:0], flux_cirbuf_rdata[63:53]};
            54  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[52:0], flux_cirbuf_rdata[63:54]};
            55  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[53:0], flux_cirbuf_rdata[63:55]};
            56  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[54:0], flux_cirbuf_rdata[63:56]};
            57  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[55:0], flux_cirbuf_rdata[63:57]};
            58  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[56:0], flux_cirbuf_rdata[63:58]};
            59  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[57:0], flux_cirbuf_rdata[63:59]};
            60  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[58:0], flux_cirbuf_rdata[63:60]};
            61  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[59:0], flux_cirbuf_rdata[63:61]};
            62  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[60:0], flux_cirbuf_rdata[63:62]};
            63  :   flux_bin = {flux_cirbuf_wdata, flux_cirbuf_rdata[61:0], flux_cirbuf_rdata[63]};   // write 62
            default: flux_bin = '0;
        endcase
    end
    
    

    //-------------------------------------------------------------------------
    // FSM
    //-------------------------------------------------------------------------
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            p_state <= idle;
        else
            p_state <= n_state;
    end
    
    always_comb begin
        n_state = p_state;
        unique case(p_state)
            idle        :   if      (handshaking_flag == 1)                                         n_state = padding;
            padding     :   if      (!spi_en_inf_system_sync)                                       n_state = idle;
                            else if (handshaking_flag_d1 == 1 && row_wcnt == 31 && col_wcnt != 63)  n_state = idle;
                            else if (handshaking_flag_d1 == 1 && row_wcnt == 31 && col_wcnt == 63)  n_state = send_mfcc;
            send_mfcc   :   if      (!spi_en_inf_system_sync)                                       n_state = idle;
                            else if (send_mfcc_row_rptr == 31 && send_mfcc_col_rcnt == 7)           n_state = send_finish;
            send_finish :                                                                           n_state = idle2;
            idle2       :   if      (!spi_en_inf_system_sync)                                       n_state = idle;
                            else if (handshaking_flag == 1)                                         n_state = receive;
            receive     :   if      (!spi_en_inf_system_sync)                                       n_state = idle;
                            else if (handshaking_flag_d1 == 1 && row_wcnt == 31)                    n_state = send_mfcc;
            default     :                                                                           n_state = idle;
        endcase
    end
    
    always_comb begin
        th_sub_zero_en      = 0;
        send_mfcc_en        = 0;
        send_flux_en        = 0;
        col_offset_inc_en   = 0;
        fsm_fe_complete     = 0;
        unique case(p_state)
            idle        :   begin
                                if (handshaking_flag_d1 == 1 && col_wcnt == 63)        send_flux_en = 1;
                            end
            padding     :   begin
                                th_sub_zero_en = 1;
                                if (handshaking_flag_d1 == 1 && col_wcnt == 63)        send_flux_en = 1;
                            end
            send_mfcc   :   send_mfcc_en = 1;
            send_finish :   begin
                                col_offset_inc_en = 1;
                                fsm_fe_complete = 1;
                            end
            idle2       :   if (handshaking_flag_d1 == 1)                              send_flux_en = 1;
            receive     :   if (handshaking_flag_d1 == 1)                              send_flux_en = 1;
            default     :   ;
        endcase
    end
    
    // Get inference pulse
    assign spi_en_inf_system_pulse = ~spi_en_inf_system_sync_d1 & spi_en_inf_system_sync;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_en_inf_system_sync_d1 <= 0;
        end else begin
            spi_en_inf_system_sync_d1 <= spi_en_inf_system_sync;
        end
    end
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_fe_complete <= 0;
        end else if (spi_en_fe_system_sync == 0 && spi_en_inf_system_pulse == 1) begin
            spi_fe_complete <= 1;
        end else begin
            spi_fe_complete <= 0;
        end
    end

endmodule
