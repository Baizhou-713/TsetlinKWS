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
// Module: "mem_weight_bank.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: .
//
//==============================================================================

module mem_weight_bank#(
    parameter DEPTH_WEIGHT_BANK            = 2048

)(
    input logic                                 clk,
    input logic [$clog2(DEPTH_WEIGHT_BANK)-1:0] raddr_weight_bank,
    input logic                                 ren_weight_bank,
    
    // spi slave signals ------------------------------------------------------
    input logic                                 spi_wen_weight_bank_sync,
    input logic [11:0]                          SPI_ADDR,
    input logic [31:0]                          SPI_DATA,
    
    output logic signed [8:0]                   weight_data
);
    
    logic                                       MEM_WEIGHT_BANK_CE;
    logic                                       MEM_WEIGHT_BANK_WE;
    logic [$clog2(DEPTH_WEIGHT_BANK)-1:0]       MEM_WEIGHT_BANK_ADDR;
    
    logic signed [8:0]                          weight_bank     [DEPTH_WEIGHT_BANK];
    
    assign MEM_WEIGHT_BANK_CE    = !(spi_wen_weight_bank_sync || ren_weight_bank);
    assign MEM_WEIGHT_BANK_WE    = !(spi_wen_weight_bank_sync);
    assign MEM_WEIGHT_BANK_ADDR  = (!MEM_WEIGHT_BANK_WE)? SPI_ADDR[$clog2(DEPTH_WEIGHT_BANK)-1:0] : raddr_weight_bank;
    
    always_ff @(posedge clk) begin
        if (!MEM_WEIGHT_BANK_CE) begin
            if(!MEM_WEIGHT_BANK_WE)     weight_bank[MEM_WEIGHT_BANK_ADDR] <= SPI_DATA[8:0];
            else                        weight_data  <= weight_bank[MEM_WEIGHT_BANK_ADDR];
        end   
    end
    
endmodule
