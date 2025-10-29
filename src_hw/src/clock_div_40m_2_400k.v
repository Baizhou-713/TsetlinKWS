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
// Module: "clock_div_40m_2_400k.v"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: Since the MMCM on Zynq cannot generate a low-frequency clock, 
//       this module is used to divide the 40MHz clock to 400Khz.
//
//==============================================================================

module clock_div_40m_2_400k(
    input wire clk,
    input wire rst_n,
    
    output reg clk_out
);

    reg [5:0] cnt;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt     <= 0;
            clk_out <= 0;
        end else if (cnt != 6'd49) begin
            cnt     <= cnt + 1'b1;
        end else begin
            cnt     <= 0;
            clk_out <= ~clk_out;
        end
    end

endmodule
