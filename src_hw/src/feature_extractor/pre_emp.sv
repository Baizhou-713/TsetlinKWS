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
// Module: "pre_emp.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: Pre-processing to enhance high-frequency components.
//
//==============================================================================

module pre_emp #(
    parameter DATA_WIDTH = 12

)(
    input logic                             clk,
    input logic                             rst_n,
    input logic signed [DATA_WIDTH-1:0]     audio_data,
    input logic                             full,
    
    // spi_slave Configuration registers --------------------------------------
    input logic                             spi_en_inf_sample_sync,
    input logic                             spi_en_fe_sample_sync,
    
    output logic signed [DATA_WIDTH-1:0]    emp_data,
    output logic                            w_req
);
    
    logic                           inf_fe_ena;
    logic signed [DATA_WIDTH-1:0]   data_reg;
    logic signed [DATA_WIDTH:0]     data_sub;
    logic signed [DATA_WIDTH+1:0]   emp_data_temp;
    
    assign inf_fe_ena       = spi_en_inf_sample_sync & spi_en_fe_sample_sync;
    assign data_sub         = audio_data - data_reg;
    assign emp_data_temp    = data_sub + (data_reg >>> 4);
    assign emp_data         = {emp_data_temp[DATA_WIDTH + 1], emp_data_temp[DATA_WIDTH - 2:0]};
    assign w_req            = (~full && inf_fe_ena) ;
    
    always_ff @(posedge clk, negedge rst_n) begin
        if(!rst_n)
            data_reg <= '0;
        else
            data_reg <= audio_data;
    end
    
endmodule
