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
// Module: "spi_slave.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: The SPI interface is used for the master control and configure 
//       the accelerator.
//
//==============================================================================

module spi_slave #(
    parameter N_PE_COL                  = 5,
    parameter DEPTH_BLOCK_BANK          = 2048,
    parameter DEPTH_ROW_BANK            = 2048,
    parameter DEPTH_CCL_BANK            = 4096,
    parameter DEPTH_WEIGHT_BANK         = 2048
    
)(
    
    // SPI interface ----------------------------------------------------------
    input logic         SCK,
    input logic         CS,
    input logic         MOSI,
    //output wire         MISO,
    
    // System inputs ----------------------------------------------------------
    input logic         rst_n,
    
    // outputs to system ------------------------------------------------------
    output logic        SPI_WEN_BLOCK_BANK,
    output logic        SPI_WEN_ROW_BANK        [N_PE_COL],
    output logic        SPI_WEN_CCL_BANK        [N_PE_COL],
    output logic        SPI_WEN_WEIGHT_BANK,
    output logic        SPI_WEN_FE_BANK,
    output logic [11:0] SPI_ADDR,
    output logic [31:0] SPI_DATA,
    
    // Configuration registers ------------------------------------------------
    output logic        SPI_EN_CONF,
    output logic        SPI_EN_INF,
    output logic        SPI_EN_FE,
    output logic [3:0]  SPI_NUM_CLASS,
    output logic [7:0]  SPI_NUM_CLAUSE,
    output logic [5:0]  SPI_NUM_SUM_TIME,
    output logic [15:0] SPI_FLUX_TH,
    output logic [$clog2(DEPTH_BLOCK_BANK)-1:0]     SPI_LEN_BLOCK_BANK,
    output logic [$clog2(DEPTH_ROW_BANK)-1:0]       SPI_LEN_ROW_BANK        [N_PE_COL],
    output logic [$clog2(DEPTH_CCL_BANK)-1:0]       SPI_LEN_CCL_BANK        [N_PE_COL],
    output logic [$clog2(DEPTH_WEIGHT_BANK)-1:0]    SPI_LEN_WEIGHT_BANK
    
);
    localparam cmd_conf_reg     = 3'b000;
    localparam cmd_block_bank   = 3'b001;
    localparam cmd_row_bank     = 3'b010;
    localparam cmd_ccl_bank     = 3'b011;
    localparam cmd_weight_bank  = 3'b100;
    localparam cmd_feature_bank = 3'b101;
    
    genvar i;
    typedef enum logic {addr_phase, data_phase} state_t;
    state_t p_state, n_state;
    
    logic FSM_update_spi_addr;
    logic FSM_update_spi_data;
    logic FSM_update_brust_len;
    logic FSM_flush_rec_num;
    logic FSM_inc_rec_num, FSM_inc_rec_num_reg;
    logic FSM_wen_conf_reg;
    logic wen_block_bank;
    logic wen_row_bank;
    logic wen_ccl_bank;
    logic wen_weight_bank;
    logic wen_feature_bank;
    
    logic [4:0]     spi_rcnt;
    logic [31:0]    mosi_buffer_comb;
    logic [31:0]    spi_addr;
    logic [31:0]    spi_data;
    logic [31:0]    spi_shift_reg_in;
    logic [11:0]    spi_receive_num;
    logic [11:0]    brust_len;      // MSB([24]) does not used.
    logic [5:0]     config_addr;
    logic [2:0]     bank_sel;
    
    //-------------------------------------------------------------------------
    // Inputs/Outputs logic
    //-------------------------------------------------------------------------
    assign SPI_DATA             = spi_data;
    assign SPI_ADDR             = spi_addr[11:0] + spi_receive_num;
    
    always_ff @(posedge SCK, negedge rst_n) begin
        if (!rst_n) begin
            SPI_WEN_BLOCK_BANK  <= 0;
            SPI_WEN_WEIGHT_BANK <= 0;
            SPI_WEN_FE_BANK     <= 0;
        end else begin
            SPI_WEN_BLOCK_BANK  <= wen_block_bank;
            SPI_WEN_WEIGHT_BANK <= wen_weight_bank;
            SPI_WEN_FE_BANK     <= wen_feature_bank;
        end
    end
    
for (i = 0; i < N_PE_COL; i++) begin
    always_ff @(posedge SCK, negedge rst_n) begin
        if (!rst_n) begin
            SPI_WEN_ROW_BANK[i] <= 0;
            SPI_WEN_CCL_BANK[i] <= 0;
        end else begin
            SPI_WEN_ROW_BANK[i] <= (bank_sel == i)? wen_row_bank : 0;
            SPI_WEN_CCL_BANK[i] <= (bank_sel == i)? wen_ccl_bank : 0;
        end
    end
end

    //-------------------------------------------------------------------------
    // SPI receive logic
    //-------------------------------------------------------------------------
    assign mosi_buffer_comb = {spi_shift_reg_in[30:0], MOSI};
    assign config_addr      = spi_addr[4:0] + spi_receive_num[4:0];
    assign bank_sel         = spi_addr[27:25];
    
    always_ff @(posedge SCK, negedge rst_n) begin
        if (!rst_n)     spi_rcnt <= '0;
        else if (!CS)   spi_rcnt <= spi_rcnt + 1'b1;
    end
    
    always_ff @(posedge SCK) begin
        if (!CS) spi_shift_reg_in <= {spi_shift_reg_in[30:0], MOSI};
    end
    
    always_ff @(posedge SCK, negedge rst_n) begin
        if (!rst_n) begin
            spi_addr <= 0;
        end else if (FSM_update_spi_addr) begin
            spi_addr <= {spi_shift_reg_in[30:0], MOSI};
        end
    end
    
    always_ff @(posedge SCK, negedge rst_n) begin
        if (!rst_n) begin
            spi_data <= 0;
        end else if (FSM_update_spi_data) begin
            spi_data <= {spi_shift_reg_in[30:0], MOSI};
        end
    end
    
    always_ff @(posedge SCK, negedge rst_n) begin
        if (!rst_n) begin
            brust_len <= 0;
        end else if (FSM_update_brust_len) begin
            brust_len <= mosi_buffer_comb[23:12];
        end
    end
    
    always_ff @(posedge SCK, negedge rst_n) begin
        if (!rst_n) begin
            FSM_inc_rec_num_reg <= 0;
        end else begin
            FSM_inc_rec_num_reg <= FSM_inc_rec_num;
        end
    end
    
    always_ff @(posedge SCK, negedge rst_n) begin
        if (!rst_n) begin
            spi_receive_num <= '0;
        end else if (FSM_flush_rec_num) begin
            spi_receive_num <= '0;
        end else if (FSM_inc_rec_num_reg) begin
            spi_receive_num <= spi_receive_num + 1'b1;
        end
    end
    
    //-------------------------------------------------------------------------
    // Configuration registers
    //-------------------------------------------------------------------------
    
    // SPI_EN_CONF(1-bit), config_addr: 0
    always @(posedge SCK, negedge rst_n) begin
        if (!rst_n)                                         SPI_EN_CONF <= '1;
        else if (FSM_wen_conf_reg && config_addr == 0)      SPI_EN_CONF <= mosi_buffer_comb[0];
    end
    
    // SPI_EN_INF(1-bit), config_addr: 1
    always @(posedge SCK, negedge rst_n) begin
        if (!rst_n)                                                     SPI_EN_INF <= '0;
        else if (SPI_EN_CONF && FSM_wen_conf_reg && config_addr == 1)   SPI_EN_INF <= mosi_buffer_comb[0];
    end
    
    // SPI_EN_FE(1-bit), config_addr: 2
    always @(posedge SCK, negedge rst_n) begin
        if (!rst_n)                                                     SPI_EN_FE <= '1;
        else if (SPI_EN_CONF && FSM_wen_conf_reg && config_addr == 2)   SPI_EN_FE <= mosi_buffer_comb[0];
    end
    
    // SPI_NUM_CLASS(4-bit), config_addr: 3
    always @(posedge SCK, negedge rst_n) begin
        if (!rst_n)                                                     SPI_NUM_CLASS <= '0;
        else if (SPI_EN_CONF && FSM_wen_conf_reg && config_addr == 3)   SPI_NUM_CLASS <= mosi_buffer_comb[3:0];
    end
    
    // SPI_NUM_CLAUSE(8-bit), config_addr: 4
    always @(posedge SCK, negedge rst_n) begin
        if (!rst_n)                                                     SPI_NUM_CLAUSE <= '0;
        else if (SPI_EN_CONF && FSM_wen_conf_reg && config_addr == 4)   SPI_NUM_CLAUSE <= mosi_buffer_comb[7:0];
    end
    
    // SPI_NUM_SUM_TIME(6-bit), config_addr: 5
    always @(posedge SCK, negedge rst_n) begin
        if (!rst_n)                                                     SPI_NUM_SUM_TIME <= '0;
        else if (SPI_EN_CONF && FSM_wen_conf_reg && config_addr == 5)   SPI_NUM_SUM_TIME <= mosi_buffer_comb[5:0];
    end
    
    // SPI_FLUX_TH(16-bit), config_addr: 6
    always @(posedge SCK, negedge rst_n) begin
        if (!rst_n)                                                     SPI_FLUX_TH <= '0;
        else if (SPI_EN_CONF && FSM_wen_conf_reg && config_addr == 6)   SPI_FLUX_TH <= mosi_buffer_comb[15:0];
    end
    
    // SPI_LEN_BLOCK_BANK(11-bit), config_addr: 7
    always @(posedge SCK, negedge rst_n) begin
        if (!rst_n)                                                     SPI_LEN_BLOCK_BANK <= '0;
        else if (SPI_EN_CONF && FSM_wen_conf_reg && config_addr == 7)   SPI_LEN_BLOCK_BANK <= mosi_buffer_comb[$clog2(DEPTH_BLOCK_BANK)-1:0];
    end
    
generate
for (i = 0; i < N_PE_COL; i++) begin
    
    // SPI_LEN_ROW_BANK(10-bit), config_addr: 8-12
    always @(posedge SCK, negedge rst_n) begin
        if (!rst_n)                                                         SPI_LEN_ROW_BANK[i] <= '0;
        else if (SPI_EN_CONF && FSM_wen_conf_reg && config_addr == (8+i))   SPI_LEN_ROW_BANK[i] <= mosi_buffer_comb[$clog2(DEPTH_ROW_BANK)-1:0];
    end
    
    // SPI_LEN_CCL_BANK(11-bit), config_addr: 13-17
    always @(posedge SCK, negedge rst_n) begin
        if (!rst_n)                                                         SPI_LEN_CCL_BANK[i] <= '0;
        else if (SPI_EN_CONF && FSM_wen_conf_reg && config_addr == (13+i))  SPI_LEN_CCL_BANK[i] <= mosi_buffer_comb[$clog2(DEPTH_CCL_BANK)-1:0];
    end
    
end
endgenerate
    
    // SPI_LEN_WEIGHT_BANK(12-bit), config_addr: 18
    always @(posedge SCK, negedge rst_n) begin
        if (!rst_n)                                                         SPI_LEN_WEIGHT_BANK <= '0;
        else if (SPI_EN_CONF && FSM_wen_conf_reg && config_addr == 18)      SPI_LEN_WEIGHT_BANK <= mosi_buffer_comb[$clog2(DEPTH_WEIGHT_BANK)-1:0];
    end
    
    //-------------------------------------------------------------------------
    // SPI FSM
    //-------------------------------------------------------------------------
    
    always_ff @(posedge SCK, negedge rst_n) begin
        if (!rst_n)
            p_state <= addr_phase;
        else
            p_state <= n_state;
    end
    
    always_comb begin
        n_state = p_state;
        unique case(p_state)
            addr_phase  :   if      (!CS && spi_rcnt == 5'd31 &&
                                     (mosi_buffer_comb[30:28] == cmd_conf_reg     ||
                                      mosi_buffer_comb[30:28] == cmd_block_bank   || 
                                      mosi_buffer_comb[30:28] == cmd_row_bank     ||
                                      mosi_buffer_comb[30:28] == cmd_ccl_bank     ||
                                      mosi_buffer_comb[30:28] == cmd_weight_bank  ||
                                      mosi_buffer_comb[30:28] == cmd_feature_bank))     n_state = data_phase;
            data_phase  :   if      (!CS && spi_rcnt == 5'd31 && 
                                     spi_receive_num == brust_len)                      n_state = addr_phase;
            default     :                                                               n_state = addr_phase;
        endcase
    end
    
    always_comb begin
        FSM_update_spi_addr     = 0;
        FSM_update_spi_data     = 0;
        FSM_update_brust_len    = 0;
        FSM_flush_rec_num       = 0;
        FSM_inc_rec_num         = 0;
        FSM_wen_conf_reg        = 0;
        wen_block_bank          = 0;
        wen_row_bank            = 0;
        wen_ccl_bank            = 0;
        wen_weight_bank         = 0;
        wen_feature_bank        = 0;
        
        unique case(p_state)
            addr_phase  :   begin 
                                if (!CS && spi_rcnt == 5'd31) begin
                                    FSM_update_spi_addr     = 1;
                                    FSM_update_brust_len    = 1;
                                    FSM_flush_rec_num       = 1;
                                end
                            end
            data_phase  :   begin
                                if (!CS && SPI_EN_CONF && spi_rcnt == 5'd31 && 
                                            spi_receive_num != brust_len)                       FSM_inc_rec_num     = 1;
                                if (!CS && spi_rcnt == 5'd31)                                   FSM_update_spi_data = 1;
                                if (!CS && spi_rcnt == 5'd31) begin
                                    if      (spi_addr[30:28] == cmd_conf_reg)                   FSM_wen_conf_reg    = 1;
                                    else if (spi_addr[30:28] == cmd_block_bank)                 wen_block_bank      = 1;
                                    else if (spi_addr[30:28] == cmd_row_bank)                   wen_row_bank        = 1;
                                    else if (spi_addr[30:28] == cmd_ccl_bank)                   wen_ccl_bank        = 1;
                                    else if (spi_addr[30:28] == cmd_weight_bank)                wen_weight_bank     = 1;
                                    else if (spi_addr[30:28] == cmd_feature_bank)               wen_feature_bank    = 1;
                                end
                            end
            default: ;
        endcase
    end
    
endmodule
