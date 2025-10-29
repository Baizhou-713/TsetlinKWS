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
// Module: "tsetlin_machine_accelerator.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: Tsetlin Machine accelerator top module.
//
//==============================================================================

module tsetlin_machine_accelerator #(
    parameter N_PE_COL                  = 5,
    parameter N_ELEMENT                 = 4,
    parameter N_MEL                     = 32,
    parameter N_FRAME                   = 64,
    parameter NUMBER_OF_PATCH           = 58,
    parameter DEPTH_BLOCK_BANK          = 2048,
    parameter DEPTH_ROW_BANK            = 2048,
    parameter DEPTH_CCL_BANK            = 4096,
    parameter DEPTH_WEIGHT_BANK         = 2048
    
)(
    input logic                                     clk,
    input logic                                     rst_n,
    
    // feature bank signals ---------------------------------------------------
    input logic                                     fe_complete,
    input logic [N_FRAME-1:0]                       feature_bank_rdata,
    output logic                                    feature_bank_ren,
    output logic [$clog2(2*N_MEL)-1:0]              feature_rptr,
    
    // spi_slave signals ------------------------------------------------------
    input logic                                     SPI_WEN_BLOCK_BANK,
    input logic                                     SPI_WEN_ROW_BANK        [N_PE_COL],
    input logic                                     SPI_WEN_CCL_BANK        [N_PE_COL],
    input logic                                     SPI_WEN_WEIGHT_BANK,
    input logic [11:0]                              SPI_ADDR,
    input logic [31:0]                              SPI_DATA,
    
    // spi_slave Configuration registers --------------------------------------
    input logic [3:0]                               SPI_NUM_CLASS,
    input logic [7:0]                               SPI_NUM_CLAUSE,
    input logic [5:0]                               SPI_NUM_SUM_TIME,
    input logic [$clog2(DEPTH_BLOCK_BANK)-1:0]      SPI_LEN_BLOCK_BANK,
    input logic [$clog2(DEPTH_ROW_BANK)-1:0]        SPI_LEN_ROW_BANK        [N_PE_COL],
    input logic [$clog2(DEPTH_CCL_BANK)-1:0]        SPI_LEN_CCL_BANK        [N_PE_COL],
    input logic [$clog2(DEPTH_WEIGHT_BANK)-1:0]     SPI_LEN_WEIGHT_BANK,
    
    // result signals ---------------------------------------------------------
    output logic [3:0]                              Result,
    output logic                                    Inf_Done
    
);
    
    // spi_slave sync signals
    logic                                   spi_wen_block_bank_d1, spi_wen_block_bank_d2, spi_wen_block_bank_d3;
    logic [ N_PE_COL-1:0]                   spi_wen_row_bank_d1, spi_wen_row_bank_d2, spi_wen_row_bank_d3;
    logic [ N_PE_COL-1:0]                   spi_wen_ccl_bank_d1, spi_wen_ccl_bank_d2, spi_wen_ccl_bank_d3;
    logic                                   spi_wen_weight_bank_d1, spi_wen_weight_bank_d2, spi_wen_weight_bank_d3;
    
    logic                                   spi_wen_block_bank_sync;
    logic [ N_PE_COL-1:0]                   spi_wen_row_bank_sync;
    logic [ N_PE_COL-1:0]                   spi_wen_ccl_bank_sync;
    logic                                   spi_wen_weight_bank_sync;
    
    // tma controller signals
    logic                                   argmax_done;
    logic                                   decode_en;
    logic                                   tail_flush_en;
    
    // ogbcsr decoder signals 
    logic                                   decoder_finish;
    logic [N_ELEMENT*N_PE_COL-1:0]          block_idx_data;
    logic [$clog2(DEPTH_BLOCK_BANK)-1:0]    raddr_block_idx_bank;
    logic                                   ren_block_idx_bank;
    logic [5:0]                             row_cnt_data                [N_PE_COL];
    logic [$clog2(DEPTH_ROW_BANK)-1:0]      raddr_row_cnt_bank          [N_PE_COL];
    logic [N_PE_COL-1:0]                    ren_row_cnt_bank;
    logic [4:0]                             col_clause_idx_data         [N_PE_COL];
    logic [$clog2(DEPTH_CCL_BANK)-1:0]      raddr_col_clause_idx_bank   [N_PE_COL];
    logic [N_PE_COL-1:0]                    ren_col_clause_idx_bank;
    logic [$clog2(DEPTH_WEIGHT_BANK)-1:0]   raddr_weight_bank;
    logic                                   ren_weight_bank;
    logic signed [8:0]                      weight_data;
        
    logic                                   block_stage_ready;
    logic [N_PE_COL-1:0]                    block_stage_valid;
    logic                                   row_stage_ready;
    logic                                   row_stage_valid             [N_PE_COL];
    logic                                   row_spad_index              [N_PE_COL];
    logic                                   col_clause_stage_valid      [N_PE_COL];
    logic                                   col_clause_stage_ready      [N_PE_COL];
    logic [1:0]                             code_ccl_stage              [N_PE_COL];
    logic [4:0]                             col_clause_index            [N_PE_COL];
    logic [$clog2(DEPTH_CCL_BANK)-1:0]      raddr_col_clause_idx_bank_int[N_PE_COL];
    
    // PE array singals
    logic [1:0]                             code_pe_stage               [N_PE_COL];
    logic                                   pe_ena                      [N_PE_COL];
    logic [2*N_ELEMENT-1:0]                 next_clause_flag            [N_PE_COL];
    logic                                   clause_index                [N_PE_COL];
    logic                                   inv_en                      [N_PE_COL];
    logic [NUMBER_OF_PATCH-1:0]             literal_data                [N_PE_COL];
    
    // summation singals
    logic                                   summation_ena;
    logic                                   patch0_result               [N_ELEMENT*N_PE_COL];
    logic                                   patch1_result               [N_ELEMENT*N_PE_COL];
    
    // argmax singals
    logic                                   argmax_ena;
    logic signed [13:0]                     class_summation;
    logic [3:0]                             class_idx;
    
    
    // sync process
    assign spi_wen_block_bank_sync      = ~spi_wen_block_bank_d3 & spi_wen_block_bank_d2;
    assign spi_wen_weight_bank_sync     = ~spi_wen_weight_bank_d3 & spi_wen_weight_bank_d2;
    
    always_comb begin
        for (int i = 0; i < N_PE_COL; i++) begin
            spi_wen_row_bank_sync[i]    = ~spi_wen_row_bank_d3[i] & spi_wen_row_bank_d2[i];
            spi_wen_ccl_bank_sync[i]    = ~spi_wen_ccl_bank_d3[i] & spi_wen_ccl_bank_d2[i];
        end
    end
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            spi_wen_block_bank_d1  <= 0;
            spi_wen_block_bank_d2  <= 0;
            spi_wen_block_bank_d3  <= 0;
            spi_wen_weight_bank_d1 <= 0;
            spi_wen_weight_bank_d2 <= 0;
            spi_wen_weight_bank_d3 <= 0;
        end else begin
            spi_wen_block_bank_d1  <= SPI_WEN_BLOCK_BANK;
            spi_wen_block_bank_d2  <= spi_wen_block_bank_d1;
            spi_wen_block_bank_d3  <= spi_wen_block_bank_d2;
            spi_wen_weight_bank_d1 <= SPI_WEN_WEIGHT_BANK;
            spi_wen_weight_bank_d2 <= spi_wen_weight_bank_d1;
            spi_wen_weight_bank_d3 <= spi_wen_weight_bank_d2;
        end
    end
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < N_PE_COL; i++) begin
                spi_wen_row_bank_d1[i] <= 0;
                spi_wen_row_bank_d2[i] <= 0;
                spi_wen_row_bank_d3[i] <= 0;
                spi_wen_ccl_bank_d1[i] <= 0;
                spi_wen_ccl_bank_d2[i] <= 0;
                spi_wen_ccl_bank_d3[i] <= 0;
            end
        end else begin
            for (int i = 0; i < N_PE_COL; i++) begin
                spi_wen_row_bank_d1[i] <= SPI_WEN_ROW_BANK[i];
                spi_wen_row_bank_d2[i] <= spi_wen_row_bank_d1[i];
                spi_wen_row_bank_d3[i] <= spi_wen_row_bank_d2[i];
                spi_wen_ccl_bank_d1[i] <= SPI_WEN_CCL_BANK[i];
                spi_wen_ccl_bank_d2[i] <= spi_wen_ccl_bank_d1[i];
                spi_wen_ccl_bank_d3[i] <= spi_wen_ccl_bank_d2[i];
            end
        end
    end
    
    tma_controller tma_controller_inst(
        .clk                            (clk                            ),
        .rst_n                          (rst_n                          ),
        .fe_complete                    (fe_complete                    ),
        .decoder_finish                 (decoder_finish                 ),
        .argmax_done                    (argmax_done                    ),
        
        .decode_en                      (decode_en                      ),
        .tail_flush_en                  (tail_flush_en                  ),
        .inf_done                       (Inf_Done                       )
    );

    ogbcsr_decoder #(
        .DEPTH_BLOCK_BANK               (DEPTH_BLOCK_BANK               ),
        .DEPTH_ROW_BANK                 (DEPTH_ROW_BANK                 ),
        .DEPTH_CCL_BANK                 (DEPTH_CCL_BANK                 ),
        .N_PE_COL                       (N_PE_COL                       )
    
    ) ogbcsr_decoder_inst (
        .clk                            (clk                            ),
        .rst_n                          (rst_n                          ),
        .decode_en                      (decode_en                      ),
        .SPI_LEN_BLOCK_BANK             (SPI_LEN_BLOCK_BANK             ),
        .SPI_LEN_ROW_BANK               (SPI_LEN_ROW_BANK               ),
        .SPI_LEN_CCL_BANK               (SPI_LEN_CCL_BANK               ),
        
        .block_idx_data                 (block_idx_data                 ),
        .raddr_block_idx_bank           (raddr_block_idx_bank           ),
        .ren_block_idx_bank             (ren_block_idx_bank             ),
        .row_cnt_data                   (row_cnt_data                   ),
        .raddr_row_cnt_bank             (raddr_row_cnt_bank             ),
        .ren_row_cnt_bank               (ren_row_cnt_bank               ),
        .col_clause_idx_data            (col_clause_idx_data            ),
        .raddr_col_clause_idx_bank      (raddr_col_clause_idx_bank      ),
        .ren_col_clause_idx_bank        (ren_col_clause_idx_bank        ),
                                                                        
        .decoder_finish                 (decoder_finish                 ),
        .block_stage_ready              (block_stage_ready              ),
        .block_stage_valid              (block_stage_valid              ),
        .row_stage_ready                (row_stage_ready                ),
        .row_stage_valid                (row_stage_valid                ),
        .row_spad_index                 (row_spad_index                 ),
        .col_clause_stage_ready         (col_clause_stage_ready         ),
        .col_clause_stage_valid         (col_clause_stage_valid         ),
        .code_ccl_stage                 (code_ccl_stage                 ),
        .col_clause_index               (col_clause_index               ),
        .raddr_col_clause_idx_bank_int  (raddr_col_clause_idx_bank_int  )
    );
    
    mem_block_idx_bank #(
        .N_PE_CLUSTER                   (N_ELEMENT*N_PE_COL             ),
        .DEPTH_BLOCK_BANK               (DEPTH_BLOCK_BANK               )
        
    ) mem_block_idx_bank_inst (
        .clk                            (clk                            ),
        .raddr_block_idx_bank           (raddr_block_idx_bank           ),
        .ren_block_idx_bank             (ren_block_idx_bank             ),
        .spi_wen_block_bank_sync        (spi_wen_block_bank_sync        ),
        .SPI_ADDR                       (SPI_ADDR                       ),
        .SPI_DATA                       (SPI_DATA                       ),
        
        .block_idx_data                 (block_idx_data                 )
    );
    
    mem_row_cnt_bank #(
        .N_PE_COL                       (N_PE_COL                       ),
        .DEPTH_ROW_BANK                 (DEPTH_ROW_BANK                 )
        
    ) mem_row_cnt_bank_inst (
        .clk                            (clk                            ),
        .raddr_row_cnt_bank             (raddr_row_cnt_bank             ),
        .ren_row_cnt_bank               (ren_row_cnt_bank               ),
        .spi_wen_row_bank_sync          (spi_wen_row_bank_sync          ),
        .SPI_ADDR                       (SPI_ADDR                       ),
        .SPI_DATA                       (SPI_DATA                       ),
        
        .row_cnt_data                   (row_cnt_data                   )
    );

    mem_col_clause_idx_bank #(
        .N_PE_COL                       (N_PE_COL                       ),
        .DEPTH_CCL_BANK                 (DEPTH_CCL_BANK                 )
        
    ) mem_col_clause_idx_bank_inst (
        .clk                            (clk                            ),
        .raddr_col_clause_idx_bank      (raddr_col_clause_idx_bank      ),
        .ren_col_clause_idx_bank        (ren_col_clause_idx_bank        ),
        .spi_wen_ccl_bank_sync          (spi_wen_ccl_bank_sync          ),
        .SPI_ADDR                       (SPI_ADDR                       ),
        .SPI_DATA                       (SPI_DATA                       ),
        
        .col_clause_idx_data            (col_clause_idx_data            )
    );
    
    mem_weight_bank #(
        .DEPTH_WEIGHT_BANK              (DEPTH_WEIGHT_BANK              )
        
    ) mem_weight_bank_inst(
        .clk                            (clk                            ),
        .raddr_weight_bank              (raddr_weight_bank              ),
        .ren_weight_bank                (ren_weight_bank                ),
        .spi_wen_weight_bank_sync       (spi_wen_weight_bank_sync       ),
        .SPI_ADDR                       (SPI_ADDR                       ),
        .SPI_DATA                       (SPI_DATA                       ),
        
        .weight_data                    (weight_data                    )
    );
    
    distributor #(
        .N_MEL                          (N_MEL                          ),
        .N_FRAME                        (N_FRAME                        ),
        .NUMBER_OF_PATCH                (NUMBER_OF_PATCH                ),
        .N_PE_COL                       (N_PE_COL                       ),
        .N_ELEMENT                      (N_ELEMENT                      ),
        .DEPTH_BLOCK_BANK               (DEPTH_BLOCK_BANK               ),
        .DEPTH_CCL_BANK                 (DEPTH_CCL_BANK                 )
        
    ) distributor_inst (
        .clk                            (clk                            ),
        .rst_n                          (rst_n                          ),
        .decode_en                      (decode_en                      ),
        .raddr_block_idx_bank           (raddr_block_idx_bank           ),
        .block_stage_ready              (block_stage_ready              ),
        .block_stage_valid              (block_stage_valid              ),
        .row_stage_ready                (row_stage_ready                ),
        .row_stage_valid                (row_stage_valid                ),
        .row_spad_index                 (row_spad_index                 ),
        .col_clause_stage_valid         (col_clause_stage_valid         ),
        .col_clause_stage_ready         (col_clause_stage_ready         ),
        .col_clause_index               (col_clause_index               ),
        .code_ccl_stage                 (code_ccl_stage                 ),
        .raddr_col_clause_idx_bank_int  (raddr_col_clause_idx_bank_int  ),
        
        .feature_bank_rdata             (feature_bank_rdata             ),
        .feature_bank_ren               (feature_bank_ren               ),
        .feature_rptr                   (feature_rptr                   ),
        .code_pe_stage                  (code_pe_stage                  ),
        .pe_ena                         (pe_ena                         ),
        .next_clause_flag_to_PE         (next_clause_flag               ),
        .clause_index                   (clause_index                   ),
        .inv_en                         (inv_en                         ),
        .literal_data                   (literal_data                   ),
        .summation_ena                  (summation_ena                  )
    );
    
    pe_array #(
        .N_MEL                          (N_MEL                          ),
        .N_FRAME                        (N_FRAME                        ),
        .NUMBER_OF_PATCH                (NUMBER_OF_PATCH                ),
        .N_PE_COL                       (N_PE_COL                       ),
        .N_ELEMENT                      (N_ELEMENT                      )
        
    ) pe_array_inst (
        .clk                            (clk                            ),
        .rst_n                          (rst_n                          ),
        .code_pe_stage                  (code_pe_stage                  ),
        .pe_ena                         (pe_ena                         ),
        .next_clause_flag               (next_clause_flag               ),
        .clause_index                   (clause_index                   ),
        .inv_en                         (inv_en                         ),
        .literal_data                   (literal_data                   ),
        
        .patch0_result                  (patch0_result                  ),
        .patch1_result                  (patch1_result                  )
    );
        
        
    summation #(
        .N_PE_CLUSTER                   (N_ELEMENT*N_PE_COL             ),
        .DEPTH_WEIGHT_BANK              (DEPTH_WEIGHT_BANK              )
        
    ) summation_inst (
        .clk                            (clk                            ),
        .rst_n                          (rst_n                          ),
        .decode_en                      (decode_en                      ),
        .tail_flush_en                  (tail_flush_en                  ),
        .summation_ena                  (summation_ena                  ),
        .patch0_result                  (patch0_result                  ),
        .patch1_result                  (patch1_result                  ),
        .weight_data                    (weight_data                    ),
        .SPI_NUM_CLASS                  (SPI_NUM_CLASS                  ),
        .SPI_NUM_SUM_TIME               (SPI_NUM_SUM_TIME               ),
        .SPI_LEN_WEIGHT_BANK            (SPI_LEN_WEIGHT_BANK            ),
        
        .ren_weight_bank                (ren_weight_bank                ),
        .raddr_weight_bank              (raddr_weight_bank              ),
        .argmax_ena                     (argmax_ena                     ),
        .class_summation                (class_summation                ),
        .class_idx                      (class_idx                      )
    );
    
    argmax argmax_inst (
        .clk                            (clk                            ),
        .rst_n                          (rst_n                          ),
        .argmax_ena                     (argmax_ena                     ),
        .class_summation                (class_summation                ),
        .class_idx                      (class_idx                      ),
        .SPI_NUM_CLASS                  (SPI_NUM_CLASS                  ),
        
        .result                         (Result                         ),
        .argmax_done                    (argmax_done                    )
    );
    

endmodule
