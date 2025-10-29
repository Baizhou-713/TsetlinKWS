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
// Module: "mel_filter.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: Rectangular MEL filter.
//
//==============================================================================

module mel_filter #(
    parameter N_FFT                 = 256,
    parameter N_MEL                 = 32,
    parameter DATAOUT_WIDTH         = 16,
    parameter STAGE8_INT_BIT_WIDTH  = 15,
    parameter STAGE8_FRA_BIT_WIDTH  = 0
    
)(
    input logic                         clk,
    input logic                         rst_n,
    
    // FFT signals ------------------------------------------------------------
    input logic                         data_in_valid,
    input logic [STAGE8_INT_BIT_WIDTH + STAGE8_FRA_BIT_WIDTH - 1:0]    data_in,
    
    // spi_slave Configuration registers --------------------------------------
    input logic                         spi_en_inf_system_sync,
    
    // ping-pong buffer signals -----------------------------------------------
    output logic                        even_mel_valid,
    output logic                        odd_mel_valid,
    output logic [DATAOUT_WIDTH-1:0]    mel_value
);

    // MEL filter bank
    logic [$clog2(N_FFT/2)-1:0] even_bank   [0:N_MEL/2];
    logic [$clog2(N_FFT/2)-1:0] odd_bank    [0:N_MEL/2];
    
    logic [DATAOUT_WIDTH-1:0]   even_mel_value, odd_mel_value;
    logic [$clog2(N_FFT/2)-1:0] mel_cnt;
    logic [$clog2(N_MEL)-1:0]   even_mel_cnt, odd_mel_cnt;
    
    assign mel_value = even_mel_valid? even_mel_value:
                        odd_mel_valid? odd_mel_value:'0;
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            mel_cnt <= 0;
        else if (!spi_en_inf_system_sync)
            mel_cnt <= 0;
        else if (data_in_valid)
            mel_cnt <= mel_cnt + 1;
    end    
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            even_mel_cnt    <= 0;
            even_mel_value  <= 0;
            even_mel_valid  <= 0;
        end else if (data_in_valid) begin
            if (even_mel_cnt != 16) begin
                if(mel_cnt == even_bank[even_mel_cnt] + 1) begin
                    even_mel_value <= data_in;
                    even_mel_valid <= 0;
                end else if(mel_cnt < even_bank[even_mel_cnt + 1]) begin
                    even_mel_value <= even_mel_value + data_in;
                    even_mel_valid <= 0;
                end else if (mel_cnt == even_bank[even_mel_cnt + 1]) begin
                    even_mel_value <= even_mel_value + data_in;
                    even_mel_valid <= 1;
                    even_mel_cnt   <= even_mel_cnt + 1;
                end
            end else begin
                even_mel_valid <= 0;
            end
        end else begin
            even_mel_cnt    <= 0;
            even_mel_value  <= 0;
            even_mel_valid  <= 0;
        end
    end
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            odd_mel_cnt    <= 0;
            odd_mel_value  <= 0;
            odd_mel_valid  <= 0;
        end else if (data_in_valid) begin
            if (odd_mel_cnt != 16) begin
                if(mel_cnt == odd_bank[odd_mel_cnt] + 1) begin
                    odd_mel_value <= data_in;
                    odd_mel_valid <= 0;
                end else if(mel_cnt < odd_bank[odd_mel_cnt + 1]) begin
                    odd_mel_value <= odd_mel_value + data_in;
                    odd_mel_valid <= 0;
                end else if (mel_cnt == odd_bank[odd_mel_cnt + 1]) begin
                    odd_mel_value <= odd_mel_value + data_in;
                    odd_mel_valid <= 1;
                    odd_mel_cnt   <= odd_mel_cnt + 1;
                end
            end else begin
                odd_mel_valid <= 0;
            end
        end else begin
            odd_mel_cnt    <= 0;
            odd_mel_value  <= 0;
            odd_mel_valid  <= 0;
        end
    end

    assign even_bank[ 0] = 127;
    assign even_bank[ 1] = 2;
    assign even_bank[ 2] = 5;
    assign even_bank[ 3] = 8;
    assign even_bank[ 4] = 11;
    assign even_bank[ 5] = 14;
    assign even_bank[ 6] = 17;
    assign even_bank[ 7] = 21;
    assign even_bank[ 8] = 25;
    assign even_bank[ 9] = 31;
    assign even_bank[10] = 37;
    assign even_bank[11] = 45;
    assign even_bank[12] = 54;
    assign even_bank[13] = 65;
    assign even_bank[14] = 79;
    assign even_bank[15] = 96;
    assign even_bank[16] = 116;
    
    
    assign odd_bank[ 0] = 127;
    assign odd_bank[ 1] = 4;
    assign odd_bank[ 2] = 7;
    assign odd_bank[ 3] = 10;
    assign odd_bank[ 4] = 13;
    assign odd_bank[ 5] = 16;
    assign odd_bank[ 6] = 19;
    assign odd_bank[ 7] = 23;
    assign odd_bank[ 8] = 28;
    assign odd_bank[ 9] = 34;
    assign odd_bank[10] = 41;
    assign odd_bank[11] = 49;
    assign odd_bank[12] = 59;
    assign odd_bank[13] = 72;
    assign odd_bank[14] = 87;
    assign odd_bank[15] = 105;
    assign odd_bank[16] = 127;

endmodule
