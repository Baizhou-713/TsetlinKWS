//==============================================================================
// Copyright (c) 2024-2025 Baizhou Lin
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
// 
// Licensed under the Solderpad Hardware License v 2.1 (the "License"); 
// you may not use this file except in compliance with the License, or, 
// at your option, the Apache License version 2.0. 
// You may obtain a copy of the License at
// 
// https://solderpad.org/licenses/SHL-2.1/
// 
// Unless required by applicable law or agreed to in writing, any work 
// distributed under the License is distributed on an "AS IS" BASIS, 
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
// See the License for the specific language governing permissions and 
// limitations under the License.
//==============================================================================
//
// Project: TsetlinKWS - a keyword spotting accelerator based on Tsetlin Machine
//
// Module: "i2s_master.v"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: I2S interface for receiving digital audio signals.
//
//==============================================================================

module i2s_master #(
    parameter   I2S_DATA_WIDTH          = 24,
    parameter   Input_INT_BIT_WIDTH     = 12,
    parameter   Input_FRA_BIT_WIDTH     = 0,
    
    localparam  DATAIN_WIDTH            = Input_INT_BIT_WIDTH + Input_FRA_BIT_WIDTH
    
)(

    // global signals ---------------------------------------------------------
    input wire                      MCLK,       // 8.192MHz
    input wire                      MCLK_rst_n,
    
    // i2s signals ------------------------------------------------------------
    input wire                      ADC_SDATA,
    output wire                     BCLK,       // 1.024MHz
    output wire                     LRCLK,      // 16KHz
    
    // signals to pre-emphassis module ----------------------------------------
    output wire [DATAIN_WIDTH-1:0]  audio_data
);

    reg                         BCLK_reg;
    reg                         LRCLK_reg;
    reg [8:0]                   div_cnt;
    reg [4:0]                   i2s_rcnt;
    reg [I2S_DATA_WIDTH-1:0]    shift_data;
    reg [I2S_DATA_WIDTH-1:0]    audio_left_data;
    
    assign audio_data = audio_left_data[18:7];
    
    //-------------------------------------------------------------------------
    // I2S clock generate logic
    //-------------------------------------------------------------------------
    assign BCLK     = BCLK_reg;
    assign LRCLK    = LRCLK_reg;

    always @(posedge MCLK or negedge MCLK_rst_n) begin
        if (!MCLK_rst_n) begin
            div_cnt <= 9'd0;
        end else begin
            div_cnt <= div_cnt + 1'd1;
        end
    end
    
    always @(posedge MCLK or negedge MCLK_rst_n) begin
        if (!MCLK_rst_n) begin
            BCLK_reg    <= 0;
            LRCLK_reg   <= 0;
        end else begin
            BCLK_reg    <= div_cnt[2];
            LRCLK_reg   <= div_cnt[8];
        end
    end
    
    //-------------------------------------------------------------------------
    // I2S receive logic
    //-------------------------------------------------------------------------
    
    // I2S counter
    always @(posedge BCLK or negedge MCLK_rst_n) begin
        if (!MCLK_rst_n)    i2s_rcnt <= 0;
        else                i2s_rcnt <= i2s_rcnt + 1'b1;
    end
    
    // shift register in
    always @(posedge BCLK) begin
        if (i2s_rcnt != 0 && i2s_rcnt <= I2S_DATA_WIDTH)
            shift_data <= {shift_data[I2S_DATA_WIDTH-2:0], ADC_SDATA};
    end
    
    always @(posedge LRCLK or negedge MCLK_rst_n) begin
        if (!MCLK_rst_n) begin
            audio_left_data <= 0;
        end else begin
            audio_left_data <= shift_data;
        end
    end
    
endmodule
