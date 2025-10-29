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
// Module: "pe_array.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: logic computation module.
//
//==============================================================================

module pe_array #(
    parameter N_MEL                     = 32,
    parameter N_FRAME                   = 64,
    parameter NUMBER_OF_PATCH           = 58,
    parameter N_PE_COL                  = 5,
    parameter N_ELEMENT                 = 4
    
)(
    input logic                         clk, rst_n,
    input logic [1:0]                   code_pe_stage           [N_PE_COL],
    input logic                         pe_ena                  [N_PE_COL],
    input logic [2*N_ELEMENT-1:0]       next_clause_flag        [N_PE_COL],
    input logic                         clause_index            [N_PE_COL],
    input logic                         inv_en                  [N_PE_COL],
    input logic [NUMBER_OF_PATCH-1:0]   literal_data            [N_PE_COL],

    output logic                        patch0_result           [N_ELEMENT*N_PE_COL],
    output logic                        patch1_result           [N_ELEMENT*N_PE_COL]
);
    
    genvar i;
    
    typedef struct {
        logic pe_col_patch0_result  [N_ELEMENT];
        logic pe_col_patch1_result  [N_ELEMENT];
    } pe_col_output_t;

    pe_col_output_t     pe_out  [N_PE_COL];
    
    generate
    for (i = 0; i < N_PE_COL; i++) begin
        pe_col #(
            .NUMBER_OF_PATCH        (NUMBER_OF_PATCH                ),
            .N_ELEMENT              (N_ELEMENT                      )
            
        ) pe_col_inst(  
            .clk                    (clk                            ),
            .rst_n                  (rst_n                          ),
            .code_pe_stage          (code_pe_stage[i]               ),
            .pe_ena                 (pe_ena[i]                      ),
            .next_clause_flag       (next_clause_flag[i]            ),
            .clause_index           (clause_index[i]                ),
            .inv_en                 (inv_en[i]                      ),
            .literal_data           (literal_data[i]                ),
            
            .patch0_result          (pe_out[i].pe_col_patch0_result ),
            .patch1_result          (pe_out[i].pe_col_patch1_result )
        );
        
        // Map struct output to the original unpacked array
        always_comb begin
            for (int j = 0; j < N_ELEMENT; j++) begin
                patch0_result[i*N_ELEMENT + j] = pe_out[i].pe_col_patch0_result[j];
                patch1_result[i*N_ELEMENT + j] = pe_out[i].pe_col_patch1_result[j];
            end
        end
        
    end
    endgenerate
    
endmodule



module pe_col #(
    parameter NUMBER_OF_PATCH           = 58,
    parameter N_ELEMENT                 = 4
)(
    input logic                         clk, rst_n,
    input logic [1:0]                   code_pe_stage,
    input logic                         pe_ena,
    input logic [2*N_ELEMENT-1:0]       next_clause_flag,
    input logic                         clause_index,
    input logic                         inv_en,
    input logic [NUMBER_OF_PATCH-1:0]   literal_data,
    
    output logic                        patch0_result       [N_ELEMENT],
    output logic                        patch1_result       [N_ELEMENT]        
);
    
    genvar i;
    
    // spad to store partial "and" result
    logic [NUMBER_OF_PATCH-1:0] Pand0_SPad [N_ELEMENT];
    logic [NUMBER_OF_PATCH-1:0] Pand1_SPad [N_ELEMENT];
    
    logic [NUMBER_OF_PATCH-1:0] patch_window_result, patch_window_result_next;
    
    logic [NUMBER_OF_PATCH-1:0] literal_data_post;
    logic                       next_clause_sel;
    
    assign literal_data_post = (inv_en) ? ~literal_data : literal_data;
    
    // select valid spad to calculate
    assign patch_window_result = (clause_index == 0)? Pand0_SPad[code_pe_stage] : Pand1_SPad[code_pe_stage];
    
    // select valid next_clause_flag
    // Deocder
    always_comb begin
        next_clause_sel = 0;
        unique case({code_pe_stage, clause_index})
            3'b000: next_clause_sel = next_clause_flag[0];
            3'b001: next_clause_sel = next_clause_flag[1];
            3'b010: next_clause_sel = next_clause_flag[2];
            3'b011: next_clause_sel = next_clause_flag[3];
            3'b100: next_clause_sel = next_clause_flag[4];
            3'b101: next_clause_sel = next_clause_flag[5];
            3'b110: next_clause_sel = next_clause_flag[6];
            3'b111: next_clause_sel = next_clause_flag[7];
            default: next_clause_sel = 0;
        endcase
    end
    
    always_comb begin
        patch_window_result_next = 0;
        if (next_clause_sel) begin
            patch_window_result_next = literal_data_post;
        end else begin
            patch_window_result_next = literal_data_post & patch_window_result;
        end
    end
    
    // All spad registers are mapped to non-reset D Flip-Flop.
    always_ff @(posedge clk) begin
        if (pe_ena) begin
            if (clause_index == 0)  Pand0_SPad[code_pe_stage] <= patch_window_result_next;
            else                    Pand1_SPad[code_pe_stage] <= patch_window_result_next;
        end
    end
    
    // 58b-OR tree
    always_ff @(posedge clk) begin
        if (pe_ena) begin
            // seperate 58 bits -> 32+26 bits
            if (clause_index == 0)  patch0_result[code_pe_stage] <= (|patch_window_result_next[31:0]) || (|patch_window_result_next[57:32]);
            else                    patch1_result[code_pe_stage] <= (|patch_window_result_next[31:0]) || (|patch_window_result_next[57:32]);
        end
    end
    
endmodule
