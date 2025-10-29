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
// Module: "data_buf.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: An asynchronous FIFO used for storing audio frame data.
//
//==============================================================================

module data_buf #(
    parameter DATA_WIDTH    = 12,
    parameter N_FFT         = 256,
    
    localparam ADDR_WIDTH   = $clog2(N_FFT)
)(
    input logic                             wclk, wrst_n,
    input logic                             rclk, rrst_n,
    input logic                             w_req,
    input logic                             r_req,
    input logic signed [DATA_WIDTH-1:0]     w_data,
    
    // spi_slave Configuration registers --------------------------------------
    input logic                             spi_en_inf_sample_sync,
    input logic                             spi_en_inf_system_sync,
    
    output logic signed [DATA_WIDTH-1:0]    r_data,
    output logic                            wfull,
    output logic                            almost_rfull,
    output logic                            rempty

);
    
    logic [DATA_WIDTH-1:0]  data_buffer     [0:N_FFT-1];
    logic [ADDR_WIDTH-1:0]  w_addr, r_addr;
    logic                   wfull_next, rempty_next;
    
    // Write pointer signals
    logic [ADDR_WIDTH:0]    w2r_w_ptr;
    logic [ADDR_WIDTH:0]    w_ptr_bin, w_ptr_gray;
    logic [ADDR_WIDTH:0]    w_ptr_bin_next, w_ptr_gray_next;
    logic [ADDR_WIDTH:0]    w_ptr_sync1, w_ptr_sync2;
    
    // Read pointer signals
    logic [ADDR_WIDTH:0]    r2w_r_ptr;
    logic [ADDR_WIDTH:0]    r_ptr_bin, r_ptr_gray;
    logic [ADDR_WIDTH:0]    r_ptr_bin_next, r_ptr_gray_next;
    logic [ADDR_WIDTH:0]    r_ptr_sync1, r_ptr_sync2;
    
    logic [ADDR_WIDTH:0]    r2w_r_ptr_bin;
    logic [ADDR_WIDTH-1:0]  element_num;
    logic                   almost_wfull_next;
    logic                   almost_rfull_sync1, almost_rfull_sync2;
    
    //-------------------------------------------------------------------------
    // Read-write logic
    //-------------------------------------------------------------------------
    assign r_addr = r_ptr_bin[ADDR_WIDTH-1:0];
    assign w_addr = w_ptr_bin[ADDR_WIDTH-1:0];
    
    always_ff @(posedge wclk) begin
        if (w_req && ~wfull)
            data_buffer[w_addr] <= w_data;
    end
    
    always_ff @(posedge rclk) begin
        if (r_req && ~rempty)
            r_data <= data_buffer[r_addr];
    end
    
    //-------------------------------------------------------------------------
    // Write pointer logic
    //-------------------------------------------------------------------------
    always_ff @(posedge wclk, negedge wrst_n) begin
        if (!wrst_n) begin
            w_ptr_bin <= '0;
            w_ptr_gray <= '0;
        end else if (!spi_en_inf_sample_sync) begin
            w_ptr_bin <= '0;
            w_ptr_gray <= '0;
        end else begin
            w_ptr_bin <= w_ptr_bin_next;
            w_ptr_gray <= w_ptr_gray_next;
        end
    end
    
    assign w_ptr_gray_next = (w_ptr_bin_next >> 1) ^ w_ptr_bin_next;
    
    always_comb begin
        w_ptr_bin_next = w_ptr_bin;
        if (w_req && ~wfull)
            w_ptr_bin_next = w_ptr_bin + 1;
    end
    
    // Full detection
    assign wfull_next = (w_ptr_gray_next == {~r2w_r_ptr[ADDR_WIDTH:ADDR_WIDTH-1], r2w_r_ptr[ADDR_WIDTH-2:0]});
    
    always_ff @(posedge wclk, negedge wrst_n) begin
        if (!wrst_n)
            wfull <= 0;
        else
            wfull <= wfull_next;
    end
    
    // Almost full detection
    always_comb begin
        r2w_r_ptr_bin[ADDR_WIDTH] = r2w_r_ptr[ADDR_WIDTH];  // MSB
        for (int i = ADDR_WIDTH-1; i >= 0; i = i - 1) begin
            r2w_r_ptr_bin[i] = r2w_r_ptr[i] ^ r2w_r_ptr_bin[i+1]; // Recursive XOR
        end
    end
    
    assign element_num = w_ptr_bin_next[ADDR_WIDTH-1:0] - r2w_r_ptr_bin[ADDR_WIDTH-1:0];
    assign almost_wfull_next = (element_num >= N_FFT - 4);
    assign almost_rfull = almost_rfull_sync2;
    
    always_ff @(posedge rclk, negedge rrst_n) begin
        if (!rrst_n)begin
            almost_rfull_sync1 <= 0;
            almost_rfull_sync2 <= 0;
        end else begin
            almost_rfull_sync1 <= almost_wfull_next;
            almost_rfull_sync2 <= almost_rfull_sync1;
        end
    end
    
    // Synchronize pointer
    assign w2r_w_ptr = w_ptr_sync2;
    
    always_ff @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) begin
            w_ptr_sync1 <= '0;
            w_ptr_sync2 <= '0; 
        end else begin
            w_ptr_sync1 <= w_ptr_gray;
            w_ptr_sync2 <= w_ptr_sync1;
        end
    end
    
    //-------------------------------------------------------------------------
    // Read pointer logic
    //-------------------------------------------------------------------------
    always_ff @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) begin
            r_ptr_bin <= '0;
            r_ptr_gray <= 0;
        end else if (!spi_en_inf_system_sync) begin
            r_ptr_bin <= '0;
            r_ptr_gray <= '0;
        end else begin
            r_ptr_bin <= r_ptr_bin_next;
            r_ptr_gray <= r_ptr_gray_next;
        end
    end
    
    assign r_ptr_gray_next = (r_ptr_bin_next >> 1) ^ r_ptr_bin_next;
    
    always_comb begin
        r_ptr_bin_next = r_ptr_bin;
        if(r_req && ~rempty)
            r_ptr_bin_next = r_ptr_bin + 1;
    end
    
    // Empty detection
    assign rempty_next = (r_ptr_gray_next == w2r_w_ptr);
    
    always_ff @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n)
            rempty <= 1;
        else
            rempty <= rempty_next;
    end
    
    // Synchronize read pointer
    assign r2w_r_ptr = r_ptr_sync2;
    
    always_ff @(posedge wclk, negedge wrst_n) begin
        if (!wrst_n) begin
            r_ptr_sync1 <= 0;
            r_ptr_sync2 <= 0;
        end else begin
            r_ptr_sync1 <= r_ptr_gray;
            r_ptr_sync2 <= r_ptr_sync1;
        end
    end
    
endmodule
