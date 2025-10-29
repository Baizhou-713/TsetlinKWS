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
// Module: "twiddle_bank.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: The twiddle factor in FFT.
//
//==============================================================================

module twiddle_bank #(
    parameter TW_BIT_WIDTH          = 8,
    parameter N_FFT                 = 256,
    parameter NO_STAGE              = 0,
    parameter BANK_DEPTH            = N_FFT / (1 << (NO_STAGE + 1)),
    parameter BANK_ADDR_WIDTH       = (BANK_DEPTH > 1) ? $clog2(BANK_DEPTH) : 1,
    parameter USE_ROM               = 1
)(
    input logic                                                 clk,
    input logic                                                 tw_ren,
    input logic         [BANK_ADDR_WIDTH - 1:0]                 tw_addr,
    
    output logic signed [TW_BIT_WIDTH - 1:0]                    Re_twddle,
    output logic signed [TW_BIT_WIDTH - 1:0]                    Im_twddle
);
    
    logic signed [2*TW_BIT_WIDTH - 1:0] twiddle_memory [0:BANK_DEPTH - 1];
    logic signed [2*TW_BIT_WIDTH - 1:0] twiddle_factor;
    
    generate
        if (BANK_DEPTH == 128) begin
            initial $readmemh("twiddle128_table.dat", twiddle_memory);
        end else if (BANK_DEPTH == 64) begin
            initial $readmemh("twiddle64_table.dat", twiddle_memory);
        end else if (BANK_DEPTH == 32) begin
            initial $readmemh("twiddle32_table.dat", twiddle_memory);
        end else if (BANK_DEPTH == 16) begin
            initial $readmemh("twiddle16_table.dat", twiddle_memory);
        end else if (BANK_DEPTH == 8) begin
            initial $readmemh("twiddle8_table.dat", twiddle_memory);
        end else if (BANK_DEPTH == 4) begin
            initial $readmemh("twiddle4_table.dat", twiddle_memory);
        end else if (BANK_DEPTH == 2) begin
            initial $readmemh("twiddle2_table.dat", twiddle_memory);
        end else if (BANK_DEPTH == 1) begin
            initial $readmemh("twiddle1_table.dat", twiddle_memory);
        end else begin
            $fatal("Unsupported configuration: BANK_DEPTH=%d", BANK_DEPTH);
        end
    endgenerate

    assign {Re_twddle, Im_twddle} = twiddle_factor;
    
    generate
        if (USE_ROM == 1) begin
        
            always_ff @(posedge clk) begin
                if (tw_ren)
                    twiddle_factor <= twiddle_memory[tw_addr];
            end
            
        end else begin
        
            assign twiddle_factor = twiddle_memory[tw_addr];
            
        end
    endgenerate
    
endmodule
