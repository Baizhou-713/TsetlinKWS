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
// Module: "tma_controller.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: The controller for Tsetlin Machine accelerator.
//
//==============================================================================

module tma_controller(
    input logic             clk,
    input logic             rst_n,
    input logic             fe_complete,
    input logic             decoder_finish,
    input logic             argmax_done,
    
    output logic            decode_en,
    output logic            tail_flush_en,
    output logic            inf_done
);

    typedef enum logic [1:0] {idle, inference, wait_finish} state_t;

    state_t p_state, n_state;
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            p_state <= idle;
        else
            p_state <= n_state;
    end
    
    always_comb begin
        n_state = p_state;
        unique case (p_state)
            idle        :   if (fe_complete == 1)       n_state = inference;
            inference   :   if (decoder_finish == 1)    n_state = wait_finish;
            wait_finish :   if (argmax_done == 1)       n_state = idle;
            default:                                    n_state = idle;
        endcase
    end
    
    always_comb begin
        decode_en       = 0;
        tail_flush_en   = 0;
        inf_done        = 0;
        unique case (p_state)
            idle        : ;
            inference   : decode_en     = 1;
            wait_finish : begin 
                            tail_flush_en = 1;
                            if (argmax_done == 1) inf_done = 1;
                          end
            default     : ;
        endcase
    end

endmodule
