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
// Module: "complex_multiplier.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: Perform FFT complex multiplication.
//
//==============================================================================

module complex_multiplier#(
    parameter LAST_WIDTH        = 12,
    parameter TW_BIT_WIDTH      = 8
)(
    // complex inputs
    input logic                                         mult_en,
    input logic signed  [   LAST_WIDTH:0]               Re_a,
    input logic signed  [   LAST_WIDTH:0]               Im_a,
    input logic signed  [   TW_BIT_WIDTH - 1:0]         Re_b,
    input logic signed  [   TW_BIT_WIDTH - 1:0]         Im_b,
    
    // complex outputs
    output logic signed [   LAST_WIDTH + TW_BIT_WIDTH + 1:0] Re_mult,
    output logic signed [   LAST_WIDTH + TW_BIT_WIDTH + 1:0] Im_mult

);
    
    logic signed [LAST_WIDTH + TW_BIT_WIDTH:0] rarb, iaib, raib, iarb;
    
    assign rarb = mult_en ? Re_a * Re_b : 0;
    assign iaib = mult_en ? Im_a * Im_b : 0;
    assign raib = mult_en ? Re_a * Im_b : 0;
    assign iarb = mult_en ? Im_a * Re_b : 0;
    
    assign Re_mult = mult_en ? rarb - iaib : 0;
    assign Im_mult = mult_en ? raib + iarb : 0;


endmodule
