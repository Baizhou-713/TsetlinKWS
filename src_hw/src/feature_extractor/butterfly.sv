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
// Module: "butterfly.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: Perform FFT butterfly operation.
//
//==============================================================================

module butterfly #(
    parameter LAST_WIDTH         = 12

)(
    // complex inputs
    input logic                                 bf_en,
    input logic signed  [   LAST_WIDTH:0]       Re_a,
    input logic signed  [   LAST_WIDTH:0]       Im_a,
    input logic signed  [   LAST_WIDTH - 1:0]   Re_b,
    input logic signed  [   LAST_WIDTH - 1:0]   Im_b,

    // complex outputs
    output logic signed [   LAST_WIDTH:0]       Re_c,
    output logic signed [   LAST_WIDTH:0]       Im_c,
    output logic signed [   LAST_WIDTH:0]       Re_d,
    output logic signed [   LAST_WIDTH:0]       Im_d
);
    logic signed [LAST_WIDTH + 1:0] Re_c_temp;
    logic signed [LAST_WIDTH + 1:0] Im_c_temp;
    logic signed [LAST_WIDTH + 1:0] Re_d_temp;
    logic signed [LAST_WIDTH + 1:0] Im_d_temp;
    
    assign Re_c_temp = bf_en ? Re_a - Re_b : 0;
    assign Im_c_temp = bf_en ? Im_a - Im_b : 0;
    assign Re_d_temp = bf_en ? Re_a + Re_b : 0;
    assign Im_d_temp = bf_en ? Im_a + Im_b : 0;
    
    assign Re_c = Re_c_temp[LAST_WIDTH:0];
    assign Im_c = Im_c_temp[LAST_WIDTH:0];
    assign Re_d = Re_d_temp[LAST_WIDTH:0];
    assign Im_d = Im_d_temp[LAST_WIDTH:0];
    
    
endmodule
