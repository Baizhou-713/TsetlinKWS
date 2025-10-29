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
// Module: "argmax.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: Multi-classification Tsetlin Machine argmax component.
//
//==============================================================================

module argmax (
    input logic                         clk, rst_n,
    input logic                         argmax_ena,
    input logic signed [13:0]           class_summation,
    input logic [3:0]                   class_idx,
    
    // spi slave Configuration registers --------------------------------------
    input logic [3:0]                   SPI_NUM_CLASS,
    
    output logic [3:0]                  result,
    output logic                        argmax_done
);
    
    logic signed [13:0]     max_summation;
    logic [3:0]             max_class;
    logic [3:0]             n_class;
    
    // assign for configuration registers
    assign n_class = SPI_NUM_CLASS;
    
    // When enable argmax, read "class_summation".
    always_ff @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            max_summation   <= {1'b1,13'd0};
            max_class       <= 0;
        end else if(argmax_ena && class_summation > max_summation) begin
            max_summation   <= class_summation;
            max_class       <= class_idx;
        end else if(argmax_done) begin
            max_summation   <= {1'b1,13'd0};
            max_class       <= 0;
        end
    end
    
    // When finish an inference, send a "argmax_done" signal.
    always_ff @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            argmax_done <= 0;
            result      <= 0;
        end else if (argmax_ena && class_idx == n_class - 1) begin
            argmax_done <= 1;
            result      <= (class_summation > max_summation)? class_idx : max_class; 
        end else if (argmax_ena && class_idx == 0) begin    // When start a new inference, set "argmax_done" to 0.
            argmax_done <= 0;
            result      <= 0;
        end
    end

endmodule
