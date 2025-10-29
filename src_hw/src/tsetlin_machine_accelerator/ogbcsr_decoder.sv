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
// Module: "ogbcsr_decoder.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: 3-stage pipeline decompression module.
//
//==============================================================================

module ogbcsr_decoder #(
    parameter DEPTH_BLOCK_BANK          = 2048,
    parameter DEPTH_ROW_BANK            = 2048,
    parameter DEPTH_CCL_BANK            = 4096,
    parameter N_PE_COL                  = 5
    
)(
    input logic                                 clk, rst_n,
    input logic                                 decode_en,
    
    // spi slave Configuration registers --------------------------------------
    input logic [$clog2(DEPTH_BLOCK_BANK)-1:0]  SPI_LEN_BLOCK_BANK,
    input logic [$clog2(DEPTH_ROW_BANK)-1:0]    SPI_LEN_ROW_BANK            [N_PE_COL],
    input logic [$clog2(DEPTH_CCL_BANK)-1:0]    SPI_LEN_CCL_BANK            [N_PE_COL],
    
    // block index bank signals -----------------------------------------------
    input  logic [4*N_PE_COL-1:0]               block_idx_data,
    output logic [$clog2(DEPTH_BLOCK_BANK)-1:0] raddr_block_idx_bank,
    output logic                                ren_block_idx_bank,
    
    // row count bank signals -------------------------------------------------
    input  logic [5:0]                          row_cnt_data                [N_PE_COL],
    output logic [$clog2(DEPTH_ROW_BANK)-1:0]   raddr_row_cnt_bank          [N_PE_COL],
    output logic [N_PE_COL-1:0]                 ren_row_cnt_bank,
    
    // col and clause index bank signals --------------------------------------
    input  logic [4:0]                          col_clause_idx_data         [N_PE_COL],
    output logic [$clog2(DEPTH_CCL_BANK)-1:0]   raddr_col_clause_idx_bank   [N_PE_COL],
    output logic [N_PE_COL-1:0]                 ren_col_clause_idx_bank,
    
    // signals to controller --------------------------------------------------
    output logic                                decoder_finish, 
    
    // signals to distributor -------------------------------------------------
    output logic                                block_stage_ready,
    output logic [N_PE_COL-1:0]                 block_stage_valid,
    output logic                                row_stage_ready,
    output logic                                row_stage_valid             [N_PE_COL],
    output logic                                row_spad_index              [N_PE_COL],
    output logic                                col_clause_stage_ready      [N_PE_COL],
    output logic                                col_clause_stage_valid      [N_PE_COL],
    output logic [1:0]                          code_ccl_stage              [N_PE_COL],
    output logic [4:0]                          col_clause_index            [N_PE_COL],
    output logic [$clog2(DEPTH_CCL_BANK)-1:0]   raddr_col_clause_idx_bank_int[N_PE_COL]
);
    
    genvar i;
    
    // block stage signals
    logic [4*N_PE_COL-1:0]                      block_stage_data;
    logic [4*N_PE_COL-1:0]                      block_stage_data_comb;
    logic                                       block_stage_valid_in_fwpipe;
    logic                                       block_stage_busy;
    logic [$clog2(DEPTH_BLOCK_BANK)-1:0]        raddr_block_idx_bank_int;
    logic                                       block_wait_sram;
    logic [1:0]                                 code_block_stage            [N_PE_COL];    
    
    // row stage signals
    logic [$clog2(DEPTH_ROW_BANK)-1:0]          raddr_row_cnt_bank_int      [N_PE_COL];
    logic                                       row_ren_d1                  [N_PE_COL];
    logic [2:0]                                 ta_counter1_int             [N_PE_COL];
    logic [2:0]                                 ta_counter2_int             [N_PE_COL];
    logic [N_PE_COL-1:0]                        last_processing_matrix;
    logic [N_PE_COL-1:0]                        row_stage_ready_sub;
    logic [N_PE_COL-1:0]                        row_stage_ready_forwarding;
    logic                                       row_stage_almost_done       [N_PE_COL];
    logic [2:0]                                 ta_counter1                 [N_PE_COL];
    logic [2:0]                                 ta_counter2                 [N_PE_COL];
    logic [1:0]                                 code_row_stage              [N_PE_COL];

    // col stage signals
    logic [4:0]                                 col_clause_stage_data           [N_PE_COL];
    logic                                       col_clause_stage_valid_in_fwpipe[N_PE_COL];
    
    // bank length
    logic [$clog2(DEPTH_BLOCK_BANK)-1:0]        len_block_bank;
    logic [$clog2(DEPTH_ROW_BANK)-1:0]          len_row_bank                    [N_PE_COL];
    logic [$clog2(DEPTH_CCL_BANK)-1:0]          len_col_clause_bank             [N_PE_COL];
    
    // assign for configuration registers
    assign len_block_bank           = SPI_LEN_BLOCK_BANK;
    assign len_row_bank[0 ]         = SPI_LEN_ROW_BANK[0 ];
    assign len_row_bank[1 ]         = SPI_LEN_ROW_BANK[1 ];
    assign len_row_bank[2 ]         = SPI_LEN_ROW_BANK[2 ];
    assign len_row_bank[3 ]         = SPI_LEN_ROW_BANK[3 ];
    assign len_row_bank[4 ]         = SPI_LEN_ROW_BANK[4 ];
    assign len_col_clause_bank[0 ]  = SPI_LEN_CCL_BANK[0 ];
    assign len_col_clause_bank[1 ]  = SPI_LEN_CCL_BANK[1 ];
    assign len_col_clause_bank[2 ]  = SPI_LEN_CCL_BANK[2 ];
    assign len_col_clause_bank[3 ]  = SPI_LEN_CCL_BANK[3 ];
    assign len_col_clause_bank[4 ]  = SPI_LEN_CCL_BANK[4 ];
    
    
generate
for (i = 0; i < N_PE_COL; i++) begin
    
    assign col_clause_index[i] = col_clause_stage_data[i];

end
endgenerate
    
    //-------------------------------------------------------------------------
    // Block index bank stage
    //-------------------------------------------------------------------------
    assign block_stage_busy = (block_wait_sram == 1);
    assign block_stage_ready = (!block_stage_busy && row_stage_ready);  // if next stage ready
    
generate
for (i = 0; i < N_PE_COL; i++) begin

    assign block_stage_valid[i] = block_stage_valid_in_fwpipe & (|block_stage_data_comb[(4*i) +: 4] );
    
    // Encoder
    always_comb begin
        code_block_stage[i] = 0;
        unique casez(block_stage_data_comb[(4*i) +: 4])
            4'b???1: code_block_stage[i] = 0;
            4'b??10: code_block_stage[i] = 1;
            4'b?100: code_block_stage[i] = 2;
            4'b1000: code_block_stage[i] = 3;
            default: code_block_stage[i] = 0;
        endcase
    end
    
end
endgenerate
    
    // fw-pipelined
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            block_stage_valid_in_fwpipe <= 0;
        end else if (block_stage_ready == 1) begin
            block_stage_valid_in_fwpipe <= decode_en;
        end
    end
    
    // ren + addr
    assign ren_block_idx_bank = decode_en && block_stage_ready;
    assign raddr_block_idx_bank = raddr_block_idx_bank_int;
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            raddr_block_idx_bank_int <= '0;
        end else if (decode_en && block_stage_ready) begin
            raddr_block_idx_bank_int <= raddr_block_idx_bank_int + 1'b1;
        end else if (raddr_block_idx_bank_int == len_block_bank) begin
            raddr_block_idx_bank_int <= '0;
        end else begin
            raddr_block_idx_bank_int <= raddr_block_idx_bank_int;
        end
    end
    
    // when finish decode, send the "decoder_finish" signal to the controller.
    assign decoder_finish = (decode_en && raddr_block_idx_bank_int == len_block_bank);
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            block_wait_sram <= 0;
        else if (ren_block_idx_bank)
            block_wait_sram <= 1;
        else
            block_wait_sram <= 0;
    end

generate
for (i = 0; i < N_PE_COL; i++) begin

    // data
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            block_stage_data[(4*i) +: 4] <= '0;
        end else if (block_wait_sram == 1) begin
            unique case(code_block_stage[i])
                2'd0: block_stage_data[(4*i) +: 4] <= {block_idx_data[(4*i+1) +: 3], 1'd0};
                2'd1: block_stage_data[(4*i) +: 4] <= {block_idx_data[(4*i+2) +: 2], 2'd0};
                2'd2: block_stage_data[(4*i) +: 4] <= {block_idx_data[(4*i+3) +: 1], 3'd0};
                2'd3: block_stage_data[(4*i) +: 4] <= 4'd0;
            endcase
        end else if (row_stage_ready_sub[i]) begin
            unique case(code_block_stage[i])
                2'd0: block_stage_data[(4*i)   +: 1] <= 0;
                2'd1: block_stage_data[(4*i+1) +: 1] <= 0;
                2'd2: block_stage_data[(4*i+2) +: 1] <= 0;
                2'd3: block_stage_data[(4*i+3) +: 1] <= 0;
            endcase
        end else begin
            block_stage_data[(4*i) +: 4] <= block_stage_data[(4*i) +: 4];
        end
    end
    
    assign block_stage_data_comb[(4*i) +: 4] = (block_wait_sram == 1)? block_idx_data[(4*i) +: 4] : block_stage_data[(4*i) +: 4];
    
end
endgenerate

    
    //-------------------------------------------------------------------------
    // Row count bank stage
    //-------------------------------------------------------------------------
    assign row_stage_ready = (&row_stage_ready_forwarding) && (&last_processing_matrix);

generate
for (i = 0; i < N_PE_COL; i++) begin
    
    // addr
    assign raddr_row_cnt_bank[i] = raddr_row_cnt_bank_int[i];
    
    // addr register
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            raddr_row_cnt_bank_int[i] <= '0;
        end else if (ren_row_cnt_bank[i]) begin
            raddr_row_cnt_bank_int[i] <= raddr_row_cnt_bank_int[i] + 1'b1;
        end else if (raddr_row_cnt_bank_int[i] == len_row_bank[i]) begin
            raddr_row_cnt_bank_int[i] <= '0;
        end else begin
            raddr_row_cnt_bank_int[i] <= raddr_row_cnt_bank_int[i];
        end
    end

end
endgenerate


generate
for (i = 0; i < N_PE_COL; i++) begin
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            code_row_stage[i] <= 0;
        end else if (row_stage_ready_sub[i] == 1) begin
            code_row_stage[i] <= code_block_stage[i];
        end
    end
    
    assign row_stage_ready_forwarding[i] = row_stage_almost_done[i] || (ta_counter1[i] == 1 && ta_counter2[i] == 1) ||
                                        (ta_counter1[i] == 0 && ta_counter2[i] == 2) || (ta_counter1[i] == 2 && ta_counter2[i] == 0);
    
    assign row_stage_almost_done[i] = (ta_counter1[i] == 0 && ta_counter2[i] == 1) || (ta_counter1[i] == 1 && ta_counter2[i] == 0) || 
                                        (ta_counter1[i] == 0 && ta_counter2[i] == 0);   // multi-cycle stage
    
    assign row_stage_ready_sub[i] = (row_stage_almost_done[i] == 1 && col_clause_stage_ready[i]);
    
    assign row_stage_valid[i] = (ta_counter1[i] != 0) || (ta_counter2[i] != 0);
    assign row_spad_index[i] = (ta_counter1[i] != 0)? 0 : 1;
    
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            last_processing_matrix[i] <= 0;
        end else if (row_stage_ready_sub[i]) begin 
            unique case(block_stage_data_comb[(4*i) +: 4])
                4'b0000: last_processing_matrix[i] <= 1;
                4'b0001: last_processing_matrix[i] <= 1;
                4'b0010: last_processing_matrix[i] <= 1;
                4'b0100: last_processing_matrix[i] <= 1;
                4'b1000: last_processing_matrix[i] <= 1;
                default: last_processing_matrix[i] <= 0;
            endcase
        end else begin
            last_processing_matrix[i] <= last_processing_matrix[i];
        end
    end
    
    // ren
    //assign ren_row_cnt_bank[(4*i) +: 4] = (4'd1 << code_block_stage[i]) & {4{block_stage_valid[i]}} & {4{row_stage_ready_sub[i]}};
    assign ren_row_cnt_bank[i] = block_stage_valid[i] & row_stage_ready_sub[i];
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            row_ren_d1[i] <= 0;
        end else begin
            row_ren_d1[i] <= ren_row_cnt_bank[i];
        end
    end
    
    // data
    assign ta_counter1[i] = (row_ren_d1[i] == 1)? row_cnt_data[i][2:0] : ta_counter1_int[i];
    assign ta_counter2[i] = (row_ren_d1[i] == 1)? row_cnt_data[i][5:3] : ta_counter2_int[i];
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            ta_counter1_int[i] <= '0;
        end else if (col_clause_stage_ready[i] == 1) begin
            if (row_ren_d1[i] == 1 && ta_counter1[i] != 0) begin
                ta_counter1_int[i] <= ta_counter1[i] - 1;
            end else if (row_ren_d1[i] == 1 && ta_counter1[i] == 0) begin
                ta_counter1_int[i] <= '0;
            end else if (ta_counter1[i] != 0) begin
                ta_counter1_int[i] <= ta_counter1_int[i] - 1'b1;
            end
        end
    end
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            ta_counter2_int[i] <= '0;
        end else if (col_clause_stage_ready[i] == 1 && row_ren_d1[i] == 1) begin  // the first cycle
            if (ta_counter1[i] != 0) begin  // if cnt1 should be decreased in the first cycle, cnt2 doesn't need to be decreased
                ta_counter2_int[i] <= ta_counter2[i];
            end else if (ta_counter1[i] == 0 && ta_counter2[i] != 0) begin     // if cnt1 is zero and cnt2 is not zero, cnt2 should be decreased
                ta_counter2_int[i] <= ta_counter2[i] - 1'b1;
            end else begin  // else, cnt2 is zero, doesn't need to be decreased
                ta_counter2_int[i] <= ta_counter2[i];
            end
        end else if (col_clause_stage_ready[i] == 1) begin
            if(ta_counter2_int[i] != 0 && ta_counter1[i] == 0)
                ta_counter2_int[i] <= ta_counter2_int[i] - 1'b1;
            else
                ta_counter2_int[i] <= ta_counter2_int[i];
        end
    end
    
end
endgenerate
    
    //-------------------------------------------------------------------------
    // Column index and clause index bank stage
    //-------------------------------------------------------------------------    

generate
for (i = 0; i < N_PE_COL; i++) begin
    
    // addr
    assign raddr_col_clause_idx_bank[i] = raddr_col_clause_idx_bank_int[i];
    
    // addr register
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            raddr_col_clause_idx_bank_int[i] <= '0;
        end else if (ren_col_clause_idx_bank[i]) begin
            raddr_col_clause_idx_bank_int[i] <= raddr_col_clause_idx_bank_int[i] + 1'b1;
        end else if (raddr_col_clause_idx_bank_int[i] == len_col_clause_bank[i]) begin
            raddr_col_clause_idx_bank_int[i] <= '0;
        end else begin
            raddr_col_clause_idx_bank_int[i] <= raddr_col_clause_idx_bank_int[i];
        end
    end
    
end
endgenerate  

generate
for (i = 0; i < N_PE_COL; i++) begin
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            code_ccl_stage[i] <= 0;
        end else begin
            code_ccl_stage[i] <= code_row_stage[i];
        end
    end
    
    //assign col_clause_stage_almost_done[i] = 1;   // sigle-cycle stage (ignore this signal)
    assign col_clause_stage_ready[i] = (col_clause_stage_valid[i] == 0) || 1'b1;    // if idle or next cycle can finish operations
    assign col_clause_stage_valid[i] = col_clause_stage_valid_in_fwpipe[i];
    
    // fw-pipelined
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            col_clause_stage_valid_in_fwpipe[i] <= 0;
        end else if (col_clause_stage_ready[i] == 1) begin
            col_clause_stage_valid_in_fwpipe[i] <= row_stage_valid[i];
        end
    end
    
    // ren
    //assign ren_col_clause_idx_bank[(4*i) +: 4] = (4'd1 << code_row_stage[i]) & {4{row_stage_valid[i]}} & {4{col_clause_stage_ready[i]}};
    assign ren_col_clause_idx_bank[i] = row_stage_valid[i] & col_clause_stage_ready[i];
    
    // data
    assign col_clause_stage_data[i] = col_clause_idx_data[i];
    
end
endgenerate  

endmodule

