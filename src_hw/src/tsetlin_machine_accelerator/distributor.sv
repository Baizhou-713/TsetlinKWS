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
// Module: "distributor.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: The distributor is used to distribute data between the feature module 
//       and the PE array.
//
//==============================================================================

module distributor #(
    parameter N_MEL                     = 32,
    parameter N_FRAME                   = 64,
    parameter NUMBER_OF_PATCH           = 58,
    parameter N_PE_COL                  = 5,
    parameter N_ELEMENT                 = 4,
    parameter DEPTH_BLOCK_BANK          = 2048,
    parameter DEPTH_CCL_BANK            = 4096
    
)(
    input logic                                 clk,
    input logic                                 rst_n,
    
    // tma controller signals -------------------------------------------------
    input logic                                 decode_en,
    
    // OG-BCSR decoder signals ------------------------------------------------
    input logic [$clog2(DEPTH_BLOCK_BANK)-1:0]  raddr_block_idx_bank,
    input logic                                 block_stage_ready,
    input logic [N_PE_COL-1:0]                  block_stage_valid,
    input logic                                 row_stage_ready,
    input logic                                 row_stage_valid         [N_PE_COL],
    input logic                                 row_spad_index          [N_PE_COL],
    input logic                                 col_clause_stage_valid  [N_PE_COL],
    input logic                                 col_clause_stage_ready  [N_PE_COL],
    input logic [4:0]                           col_clause_index        [N_PE_COL], // sync with col_valid
    input logic [1:0]                           code_ccl_stage          [N_PE_COL],
    input logic [$clog2(DEPTH_CCL_BANK)-1:0]    raddr_col_clause_idx_bank_int [N_PE_COL],
    
    // feature bank signals ---------------------------------------------------
    input logic [N_FRAME-1:0]                   feature_bank_rdata,
    output logic                                feature_bank_ren,
    output logic [$clog2(2*N_MEL)-1:0]          feature_rptr,
    
    // downstream signals -----------------------------------------------------
    output logic [1:0]                          code_pe_stage           [N_PE_COL],
    output logic                                pe_ena                  [N_PE_COL],
    output logic [2*N_ELEMENT-1:0]              next_clause_flag_to_PE  [N_PE_COL],
    output logic                                clause_index            [N_PE_COL],
    output logic                                inv_en                  [N_PE_COL],
    output logic [NUMBER_OF_PATCH-1:0]          literal_data            [N_PE_COL],
    output logic                                summation_ena
);
    

    genvar i;
    
    // feature bank accessment signals
    logic                       wait_sram;
    logic [$clog2(2*N_MEL)-1:0] block_stage_row_index;
    logic                       r_ctrl_cnt;
    logic                       w_ctrl_cnt;
    
    // feature bank spad signals
    logic                       row_spad_index_d1           [N_PE_COL];
    logic                       spad_w_sel;
    logic                       spad_r_sel;
    logic                       block_stage_handshaking_d1;
    logic                       block_stage_handshaking_d2;
    logic                       update_block_index_flag;
    logic [N_FRAME-1:0]         feature_bank_row_data0;
    logic [N_FRAME-1:0]         feature_bank_row_data1;
    logic [N_FRAME-1:0]         feature_bank_row_data2;
    logic [N_FRAME-1:0]         feature_bank_row_data3;
    logic [N_FRAME-1:0]         feature_bank_data0;
    logic [N_FRAME-1:0]         feature_bank_data1;
    
    // position spad signals
    logic [NUMBER_OF_PATCH-1:0] position_bank_row_data0;
    logic [NUMBER_OF_PATCH-1:0] position_bank_row_data1;
    logic                       reset_pos_data_flag;
    logic                       update_pos_data_flag;
    
    // internal signals
    logic [2:0]                 col_index                   [N_PE_COL];
    logic [N_FRAME-1:0]         row_data                    [N_PE_COL];
    logic [NUMBER_OF_PATCH-1:0] pos_data                    [N_PE_COL];
    logic [NUMBER_OF_PATCH-1:0] literal_data_int            [N_PE_COL];
    logic                       next_clause_flag_r;
    logic                       next_clause_flag_c;
    logic                       next_clause_flag_c_d1;
    logic                       col_stage_handshaking_d1    [N_PE_COL];
    logic                       col_stage_handshaking_d1_total;
    logic                       next_clause_flag;
    logic                       next_clause_last_flag;
    logic                       updata_last_clause_flag;

    //-------------------------------------------------------------------------
    // Distribute to PE array
    //-------------------------------------------------------------------------
    
generate
for (i = 0; i < N_PE_COL; i++) begin
    
    assign pe_ena[i]        = (col_clause_stage_valid[i] == 1)? 1 : 0;
    assign clause_index[i]  = col_clause_index[i][4];
    assign inv_en[i]        = (col_clause_index[i][3] == 1'b1);
    assign literal_data[i]  = literal_data_int[i];
    
    assign col_index[i] = col_clause_index[i][2:0];
    assign row_data[i] = (row_spad_index_d1[i] == 0)? feature_bank_data0 : feature_bank_data1;
    assign pos_data[i] = (row_spad_index_d1[i] == 0)? position_bank_row_data0 : position_bank_row_data1;
    
    // According to the column index, get the literal_data
    always_comb begin
        literal_data_int[i] = '1;
        unique case(col_index[i])
            0: literal_data_int[i] = pos_data[i];
            1: literal_data_int[i] = row_data[i][57:0];
            2: literal_data_int[i] = row_data[i][58:1];
            3: literal_data_int[i] = row_data[i][59:2];
            4: literal_data_int[i] = row_data[i][60:3];
            5: literal_data_int[i] = row_data[i][61:4];
            6: literal_data_int[i] = row_data[i][62:5];
            7: literal_data_int[i] = row_data[i][63:6];
            default: ;
        endcase
    end
    
    assign code_pe_stage[i] = code_ccl_stage[i];
    
    // When process a new TA matrix, send a "next_clause_flag_to_PE" signal to flush PEs' internal register data.
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            next_clause_flag_to_PE[i] <= '1;
        end else if (next_clause_flag)begin
            next_clause_flag_to_PE[i] <= '1;
        end else if (pe_ena[i]) begin
            unique case({code_pe_stage[i], clause_index[i]})
                3'b00_0: next_clause_flag_to_PE[i][0] <= 0;
                3'b00_1: next_clause_flag_to_PE[i][1] <= 0;
                3'b01_0: next_clause_flag_to_PE[i][2] <= 0;
                3'b01_1: next_clause_flag_to_PE[i][3] <= 0;
                3'b10_0: next_clause_flag_to_PE[i][4] <= 0;
                3'b10_1: next_clause_flag_to_PE[i][5] <= 0;
                3'b11_0: next_clause_flag_to_PE[i][6] <= 0;
                3'b11_1: next_clause_flag_to_PE[i][7] <= 0;
                default:   ;
            endcase
        end
    end
    
    // Delay and store row_spad_index by one cycle to synchronize with the column stage
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            row_spad_index_d1[i] <= '0;
        end else if (row_stage_valid[i] && col_clause_stage_ready[i]) begin
            row_spad_index_d1[i] <= row_spad_index[i];
        end
    end
    
end
endgenerate
  
    //-------------------------------------------------------------------------
    // Update feature bank row data spad
    //-------------------------------------------------------------------------
    
    // When block stage handshaking with input stage, update the row address.
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            block_stage_row_index   <= 0;
            r_ctrl_cnt              <= 0;
        end else if (decode_en && block_stage_ready && r_ctrl_cnt == 0) begin
            block_stage_row_index   <= block_stage_row_index + 1'b1;
            r_ctrl_cnt              <= 1;
        end else if (r_ctrl_cnt == 1) begin
            block_stage_row_index   <= block_stage_row_index + 1'b1;
            r_ctrl_cnt              <= 0;
        end else begin
            block_stage_row_index   <= block_stage_row_index;
            r_ctrl_cnt              <= r_ctrl_cnt;
        end
    end
    
    // Access the feature bank
    assign feature_bank_ren = (decode_en && block_stage_ready) || (r_ctrl_cnt == 1);
    assign feature_rptr = block_stage_row_index;
    
    // wait for sram
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            wait_sram <= 0;
        end else begin
            wait_sram <= feature_bank_ren;
        end
    end
    
    // Update feature bank row data to spad
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            feature_bank_row_data0  <= 0;
            feature_bank_row_data1  <= 0;
            feature_bank_row_data2  <= 0;
            feature_bank_row_data3  <= 0;
            w_ctrl_cnt              <= 0;
            spad_w_sel              <= 0;
        end else if (wait_sram == 1 && w_ctrl_cnt == 0 && spad_w_sel == 0) begin
            feature_bank_row_data0  <= feature_bank_rdata;
            w_ctrl_cnt              <= ~w_ctrl_cnt;
        end else if (wait_sram == 1 && w_ctrl_cnt == 1 && spad_w_sel == 0) begin
            feature_bank_row_data1  <= feature_bank_rdata;
            w_ctrl_cnt              <= ~w_ctrl_cnt;
            spad_w_sel              <= ~spad_w_sel;
        end else if (wait_sram == 1 && w_ctrl_cnt == 0 && spad_w_sel == 1) begin
            feature_bank_row_data2  <= feature_bank_rdata;
            w_ctrl_cnt              <= ~w_ctrl_cnt;
        end else if (wait_sram == 1 && w_ctrl_cnt == 1 && spad_w_sel == 1) begin
            feature_bank_row_data3  <= feature_bank_rdata;
            w_ctrl_cnt              <= ~w_ctrl_cnt;
            spad_w_sel              <= ~spad_w_sel;
        end
    end
  
    // When "update_block_index_flag" == 1, update "spad_r_sel".
    assign update_block_index_flag = block_stage_handshaking_d2;
    
    // Get the block stage handshaking delay signal.
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            block_stage_handshaking_d1 <= 0;
        end else if (decode_en && block_stage_ready && block_stage_row_index != 0) begin
            block_stage_handshaking_d1 <= 1;
        end else begin
            block_stage_handshaking_d1 <= 0;
        end
    end
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            block_stage_handshaking_d2 <= 0;
        end else begin
            block_stage_handshaking_d2 <= block_stage_handshaking_d1;
        end
    end
    
    // When block stage handshaking, update the spad read selection signal. 
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            spad_r_sel <= 0;
        end else if (update_block_index_flag == 1) begin
            spad_r_sel <= ~spad_r_sel;
        end else if (next_clause_flag) begin
            spad_r_sel <= 0;
        end
    end
    
    assign feature_bank_data0 = (spad_r_sel == 0)? feature_bank_row_data0 : feature_bank_row_data2;
    assign feature_bank_data1 = (spad_r_sel == 0)? feature_bank_row_data1 : feature_bank_row_data3;
    
    
    //-------------------------------------------------------------------------
    // Update position row data spad
    //-------------------------------------------------------------------------
    assign reset_pos_data_flag  = next_clause_flag;
    assign update_pos_data_flag = block_stage_handshaking_d2;
    
    // FORMAT: {WIN 57, WIN 56, ..., WIN 1, WIN 0}.
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            position_bank_row_data0 <= 58'h3FF_FFFF_FFFF_FFFE;
            position_bank_row_data1 <= 58'h3FF_FFFF_FFFF_FFFC;
        end else if (reset_pos_data_flag) begin
            position_bank_row_data0 <= 58'h3FF_FFFF_FFFF_FFFE;
            position_bank_row_data1 <= 58'h3FF_FFFF_FFFF_FFFC;
        end else if (update_pos_data_flag) begin
            position_bank_row_data0 <= position_bank_row_data0 << 2;
            position_bank_row_data1 <= position_bank_row_data1 << 2;
        end
    end
    
    
    //-------------------------------------------------------------------------
    // Signals logic to downstream circuit
    //-------------------------------------------------------------------------
    
    // When process a new TA matrix, send a "next_clause_flag" to reset position data.
    assign next_clause_flag = next_clause_flag_c || next_clause_last_flag;
    assign summation_ena    = next_clause_flag_c_d1 || next_clause_last_flag;
    
    // When the block stage handshaking, if row_index == 0, it means a new clause starts to be processed.
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            next_clause_flag_r <= 0;
        end else if (decode_en && block_stage_ready && block_stage_row_index == 0 && raddr_block_idx_bank != 0) begin
            next_clause_flag_r <= 1;
        end else begin
            next_clause_flag_r <= 0;
        end
    end
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            next_clause_flag_c <= 0;
            next_clause_flag_c_d1 <= 0;
        end else begin
            next_clause_flag_c <= next_clause_flag_r;
            next_clause_flag_c_d1 <= next_clause_flag_c;
        end
    end
    
    
generate
for (i = 0; i < N_PE_COL; i++) begin
    
    // Delay one cycles to alige with the COL BANK read address update. 
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            col_stage_handshaking_d1[i] <= 0;
        end else begin
            col_stage_handshaking_d1[i] <= col_clause_stage_valid[i];
        end
    end

end
endgenerate
    
    assign col_stage_handshaking_d1_total = col_stage_handshaking_d1[0]  || col_stage_handshaking_d1[1] ||
                                            col_stage_handshaking_d1[2]  || col_stage_handshaking_d1[3] ||
                                            col_stage_handshaking_d1[4];
    
    always_comb begin
        updata_last_clause_flag = 0;
        if (col_stage_handshaking_d1_total == 1 && 
                raddr_col_clause_idx_bank_int[0] == 0 && raddr_col_clause_idx_bank_int[1] == 0 &&
                raddr_col_clause_idx_bank_int[2] == 0 && raddr_col_clause_idx_bank_int[3] == 0 &&
                raddr_col_clause_idx_bank_int[4] == 0) begin
            updata_last_clause_flag = 1;
        end
    end
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            next_clause_last_flag <= 0;
        end else if (updata_last_clause_flag) begin
            next_clause_last_flag <= 1;
        end else begin
            next_clause_last_flag <= 0;
        end
    end

endmodule
