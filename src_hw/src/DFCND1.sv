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
// Module: "DFCND1.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: D Flip-Flop model with asynchronous reset.
//
//==============================================================================

module DFCND1(
    input logic CP,
    input logic CDN,
    input logic D,
    
    output logic Q
);

    always_ff @(posedge CP, negedge CDN) begin
        if (!CDN)   Q <= 0;
        else        Q <= D;
    end
    
endmodule
