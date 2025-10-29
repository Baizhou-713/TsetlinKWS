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
// Module: "mem_col_clause_idx_bank.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: Every 4 columns of PEs share the same CCL memory.
//
//==============================================================================

module mem_col_clause_idx_bank #(
    parameter N_PE_COL                  = 5,
    parameter DEPTH_CCL_BANK            = 4096
    
)(
    input logic                                 clk,
    input logic [$clog2(DEPTH_CCL_BANK)-1:0]    raddr_col_clause_idx_bank   [N_PE_COL],
    input logic [N_PE_COL-1:0]                  ren_col_clause_idx_bank,
    
    // spi slave signals ------------------------------------------------------
    input logic [N_PE_COL-1:0]                  spi_wen_ccl_bank_sync,
    input logic [11:0]                          SPI_ADDR,
    input logic [31:0]                          SPI_DATA,
    
    output logic [4:0]                          col_clause_idx_data         [N_PE_COL]
);
    
    logic                                       MEM_CCL_BANK_CE     [N_PE_COL];
    logic                                       MEM_CCL_BANK_WE     [N_PE_COL];
    logic [$clog2(DEPTH_CCL_BANK)-1:0]          MEM_CCL_BANK_ADDR   [N_PE_COL];
    
    logic [4:0] col_clause_idx_bank0    [DEPTH_CCL_BANK];
    logic [4:0] col_clause_idx_bank1    [DEPTH_CCL_BANK];
    logic [4:0] col_clause_idx_bank2    [DEPTH_CCL_BANK];
    logic [4:0] col_clause_idx_bank3    [DEPTH_CCL_BANK];
    logic [4:0] col_clause_idx_bank4    [DEPTH_CCL_BANK];

    always_comb begin
        for (int i = 0; i < N_PE_COL; i++) begin
            MEM_CCL_BANK_CE[i]      = !(spi_wen_ccl_bank_sync[i] || ren_col_clause_idx_bank[i]);
            MEM_CCL_BANK_WE[i]      = !(spi_wen_ccl_bank_sync[i]);
            MEM_CCL_BANK_ADDR[i]    = (!MEM_CCL_BANK_WE[i])? SPI_ADDR[$clog2(DEPTH_CCL_BANK)-1:0] : 
                                                            raddr_col_clause_idx_bank[i];
        end
    end
    
    always_ff @(posedge clk) begin
        if (!MEM_CCL_BANK_CE[0]) begin 
            if (!MEM_CCL_BANK_WE[0])    col_clause_idx_bank0[MEM_CCL_BANK_ADDR[0]] <= SPI_DATA[4:0];
            else                        col_clause_idx_data[0] <= col_clause_idx_bank0[MEM_CCL_BANK_ADDR[0]];
        end
    end
    
    always_ff @(posedge clk) begin
        if (!MEM_CCL_BANK_CE[1]) begin
            if (!MEM_CCL_BANK_WE[1])    col_clause_idx_bank1[MEM_CCL_BANK_ADDR[1]] <= SPI_DATA[4:0];
            else                        col_clause_idx_data[1] <= col_clause_idx_bank1[MEM_CCL_BANK_ADDR[1]];
        end
    end
    
    always_ff @(posedge clk) begin
        if (!MEM_CCL_BANK_CE[2]) begin
            if (!MEM_CCL_BANK_WE[2])    col_clause_idx_bank2[MEM_CCL_BANK_ADDR[2]] <= SPI_DATA[4:0];
            else                        col_clause_idx_data[2] <= col_clause_idx_bank2[MEM_CCL_BANK_ADDR[2]];
        end
    end
    
    always_ff @(posedge clk) begin
        if (!MEM_CCL_BANK_CE[3]) begin
            if (!MEM_CCL_BANK_WE[3])    col_clause_idx_bank3[MEM_CCL_BANK_ADDR[3]] <= SPI_DATA[4:0];
            else                        col_clause_idx_data[3] <= col_clause_idx_bank3[MEM_CCL_BANK_ADDR[3]];
        end
    end
    
    always_ff @(posedge clk) begin
        if (!MEM_CCL_BANK_CE[4]) begin
            if (!MEM_CCL_BANK_WE[4])    col_clause_idx_bank4[MEM_CCL_BANK_ADDR[4]] <= SPI_DATA[4:0];
            else                        col_clause_idx_data[4] <= col_clause_idx_bank4[MEM_CCL_BANK_ADDR[4]];
        end
    end

endmodule
