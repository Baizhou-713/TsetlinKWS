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
// Module: "mem_row_cnt_bank.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: Every 4 columns of PEs share the same ROW memory.
//
//==============================================================================

module mem_row_cnt_bank #(
    parameter N_PE_COL                  = 5,
    parameter DEPTH_ROW_BANK            = 2048
    
)(
    input logic                                 clk,
    input logic [$clog2(DEPTH_ROW_BANK)-1:0]    raddr_row_cnt_bank  [N_PE_COL],
    input logic [N_PE_COL-1:0]                  ren_row_cnt_bank,
    
    // spi slave signals ------------------------------------------------------
    input logic [N_PE_COL-1:0]                  spi_wen_row_bank_sync,
    input logic [11:0]                          SPI_ADDR,
    input logic [31:0]                          SPI_DATA,
    
    output logic [5:0]                          row_cnt_data        [N_PE_COL]
);
    
    logic                                       MEM_ROW_BANK_CE     [N_PE_COL];
    logic                                       MEM_ROW_BANK_WE     [N_PE_COL];
    logic [$clog2(DEPTH_ROW_BANK)-1:0]          MEM_ROW_BANK_ADDR   [N_PE_COL];
    
    logic [5:0] row_cnt_bank0    [DEPTH_ROW_BANK];
    logic [5:0] row_cnt_bank1    [DEPTH_ROW_BANK];
    logic [5:0] row_cnt_bank2    [DEPTH_ROW_BANK];
    logic [5:0] row_cnt_bank3    [DEPTH_ROW_BANK];
    logic [5:0] row_cnt_bank4    [DEPTH_ROW_BANK];
    
    always_comb begin
        for (int i = 0; i < N_PE_COL; i++) begin
            MEM_ROW_BANK_CE[i]      = !(spi_wen_row_bank_sync[i] || ren_row_cnt_bank[i]);
            MEM_ROW_BANK_WE[i]      = !(spi_wen_row_bank_sync[i]);
            MEM_ROW_BANK_ADDR[i]    = (!MEM_ROW_BANK_WE[i])? SPI_ADDR[$clog2(DEPTH_ROW_BANK)-1:0] : 
                                                            raddr_row_cnt_bank[i];
        end
    end
    
    always_ff @(posedge clk) begin
        if (!MEM_ROW_BANK_CE[0]) begin
            if (!MEM_ROW_BANK_WE[0])    row_cnt_bank0[MEM_ROW_BANK_ADDR[0]] <= SPI_DATA[5:0];
            else                        row_cnt_data[0] <= row_cnt_bank0[MEM_ROW_BANK_ADDR[0]];
        end
    end
    
    always_ff @(posedge clk) begin
        if (!MEM_ROW_BANK_CE[1]) begin
            if (!MEM_ROW_BANK_WE[1])    row_cnt_bank1[MEM_ROW_BANK_ADDR[1]] <= SPI_DATA[5:0];
            else                        row_cnt_data[1] <= row_cnt_bank1[MEM_ROW_BANK_ADDR[1]];
        end
    end
    
    always_ff @(posedge clk) begin
        if (!MEM_ROW_BANK_CE[2]) begin
            if (!MEM_ROW_BANK_WE[2])    row_cnt_bank2[MEM_ROW_BANK_ADDR[2]] <= SPI_DATA[5:0];
            else                        row_cnt_data[2] <= row_cnt_bank2[MEM_ROW_BANK_ADDR[2]];
        end
    end
    
    always_ff @(posedge clk) begin
        if (!MEM_ROW_BANK_CE[3]) begin
            if (!MEM_ROW_BANK_WE[3])    row_cnt_bank3[MEM_ROW_BANK_ADDR[3]] <= SPI_DATA[5:0];
            else                        row_cnt_data[3] <= row_cnt_bank3[MEM_ROW_BANK_ADDR[3]];
        end
    end
    
    always_ff @(posedge clk) begin
        if (!MEM_ROW_BANK_CE[4]) begin
            if (!MEM_ROW_BANK_WE[4])    row_cnt_bank4[MEM_ROW_BANK_ADDR[4]] <= SPI_DATA[5:0];
            else                        row_cnt_data[4] <= row_cnt_bank4[MEM_ROW_BANK_ADDR[4]];
        end
    end
    
endmodule
