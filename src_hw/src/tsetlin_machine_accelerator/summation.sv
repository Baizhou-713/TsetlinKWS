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
// Module: "summation.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: Multi-classification Tsetlin Machine summation component.
//
//==============================================================================

module summation #(
    parameter N_PE_CLUSTER                  = 20,
    parameter DEPTH_WEIGHT_BANK             = 2048
)(
    input logic                                 clk, rst_n,
    input logic                                 decode_en,
    input logic                                 tail_flush_en,
    input logic                                 summation_ena,
    input logic                                 patch0_result       [N_PE_CLUSTER],
    input logic                                 patch1_result       [N_PE_CLUSTER],
    input logic signed [8:0]                    weight_data,
    
    // spi slave Configuration registers --------------------------------------
    input logic [3:0]                           SPI_NUM_CLASS,
    input logic [5:0]                           SPI_NUM_SUM_TIME,
    input logic [$clog2(DEPTH_WEIGHT_BANK)-1:0] SPI_LEN_WEIGHT_BANK,
    
    output logic                                ren_weight_bank,
    output logic [$clog2(DEPTH_WEIGHT_BANK)-1:0]raddr_weight_bank,
    output logic                                argmax_ena,
    output logic signed [13:0]                  class_summation,
    output logic [3:0]                          class_idx
);
    
    genvar i;
    logic                                   clause0_sat         [N_PE_CLUSTER];
    logic                                   clause1_sat         [N_PE_CLUSTER];
    logic                                   one_class_done;
    logic [5:0]                             summation_cnt;
    logic [5:0]                             clause_cnt;
    logic [$clog2(DEPTH_WEIGHT_BANK)-1:0]   raddr_weight_bank_int;
    logic                                   ren_weight_bank_d1;
    logic                                   read_weight_flag;
    logic signed [13:0]                     weight_bank_data_ext;
    logic [3:0]                             n_class;
    logic [5:0]                             n_sum_time;
    logic [$clog2(DEPTH_WEIGHT_BANK)-1:0]   len_weight_bank;
    
    
    // assign for configuration registers
    assign n_class          = SPI_NUM_CLASS;
    assign n_sum_time       = SPI_NUM_SUM_TIME;
    assign len_weight_bank  = SPI_LEN_WEIGHT_BANK;
    
    assign argmax_ena = one_class_done;
    // Extend sign-bit.
    assign weight_bank_data_ext = weight_data;
    
    
generate
for (i = 0; i < N_PE_CLUSTER; i++) begin
    
    // When enabling the module, store the patch results.
    always_ff @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            clause0_sat[i] <= 0;
            clause1_sat[i] <= 0;
        end else if (summation_ena) begin
            clause0_sat[i] <= patch0_result[i];
            clause1_sat[i] <= patch1_result[i];
        end
    end
    
end
endgenerate
    
    // After calculating all the results of the PE array, summation_cnt ++.
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            summation_cnt <= 0;
        end else if (clause_cnt == 2 * N_PE_CLUSTER - 1) begin
            summation_cnt <= summation_cnt + 1;
        end else if (summation_cnt == n_sum_time) begin
            summation_cnt <= 0;
        end
    end
    
    // After calculating the result of a class, set "one_class_done" to 1.
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            one_class_done <= 0;
        end else if ((decode_en || tail_flush_en) && summation_cnt == n_sum_time) begin
            one_class_done <= 1;
        end else begin
            one_class_done <= 0;
        end
    end
    
    // Update "class_idx"
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            class_idx <= 0;
        end else if (one_class_done == 1) begin
            class_idx <= class_idx + 1;
        end else if (class_idx == n_class) begin
            class_idx <= 0;
        end
    end
    
    // When enable summation, set "read_weight_flag" to 1 until process all the results of the PE array.
    always_ff @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            read_weight_flag <= 0;
        end else if (summation_ena) begin
            read_weight_flag <= 1;
        end else if (clause_cnt == 2 * N_PE_CLUSTER - 1) begin
            read_weight_flag <= 0;
        end
    end
    
    always_ff @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            clause_cnt <= 0;
        end else if (read_weight_flag) begin
            clause_cnt <= clause_cnt + 1;
        end else begin
            clause_cnt <= 0;
        end
    end
    
    // ren + addr
    // If the clause is satisfied, read the weight bank.
    always_comb begin
        ren_weight_bank = 0;
        if (read_weight_flag == 1) begin
            if (clause_cnt[0] == 0 && clause0_sat[clause_cnt[5:1]] == 1) begin
                ren_weight_bank = 1;
            end else if (clause_cnt[0] == 1 && clause1_sat[clause_cnt[5:1]] == 1) begin
                ren_weight_bank = 1;
            end
        end
    end
    
    assign raddr_weight_bank = (ren_weight_bank == 1)? raddr_weight_bank_int : '0;
    
    always_ff @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            raddr_weight_bank_int <= 0;
        end else if (read_weight_flag) begin
            raddr_weight_bank_int <= raddr_weight_bank_int + 1;
        end else if (raddr_weight_bank_int == len_weight_bank) begin
            raddr_weight_bank_int <= 0;
        end
    end
    
    // Update "class_summation".
    always_ff @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            ren_weight_bank_d1 <= 0;
        end else if (ren_weight_bank) begin
            ren_weight_bank_d1 <= 1;
        end else begin
            ren_weight_bank_d1 <= 0;
        end
    end
    
    always_ff @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            class_summation <= '0;
        end else if (ren_weight_bank_d1) begin
            class_summation <= class_summation + weight_data;
        end else if (one_class_done) begin
            class_summation <= 0;
        end
    end
    
endmodule
