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
// Module: "mem_block_idx_bank.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: .
//
//==============================================================================

module mem_block_idx_bank #(
    parameter N_PE_CLUSTER              = 20,
    parameter DEPTH_BLOCK_BANK          = 2048

)(
    input logic                                 clk,
    input logic [$clog2(DEPTH_BLOCK_BANK)-1:0]  raddr_block_idx_bank,
    input logic                                 ren_block_idx_bank,
    
    // spi slave signals ------------------------------------------------------
    input logic                                 spi_wen_block_bank_sync,
    input logic [11:0]                          SPI_ADDR,
    input logic [31:0]                          SPI_DATA,
    
    output logic [N_PE_CLUSTER-1:0]             block_idx_data
);

    logic                                       MEM_BLOCK_BANK_CE;
    logic                                       MEM_BLOCK_BANK_WE;
    logic [$clog2(DEPTH_BLOCK_BANK)-1:0]        MEM_BLOCK_BANK_ADDR;
    logic [N_PE_CLUSTER-1:0]                    block_idx_bank      [DEPTH_BLOCK_BANK];
    
    
    assign MEM_BLOCK_BANK_CE    = !(spi_wen_block_bank_sync || ren_block_idx_bank);
    assign MEM_BLOCK_BANK_WE    = !(spi_wen_block_bank_sync);
    assign MEM_BLOCK_BANK_ADDR  = (!MEM_BLOCK_BANK_WE)? SPI_ADDR[$clog2(DEPTH_BLOCK_BANK)-1:0] : raddr_block_idx_bank;
    
    always_ff @(posedge clk) begin
        if (!MEM_BLOCK_BANK_CE) begin
            if(!MEM_BLOCK_BANK_WE)  block_idx_bank[MEM_BLOCK_BANK_ADDR] <= SPI_DATA[N_PE_CLUSTER-1:0];
            else                    block_idx_data  <= block_idx_bank[MEM_BLOCK_BANK_ADDR];
        end   
    end
    
endmodule
