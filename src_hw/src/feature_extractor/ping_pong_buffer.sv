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
// Module: "ping_pong_buffer.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: .
//
//==============================================================================

module ping_pong_buffer #(
    parameter N_MEL                     = 32,
    parameter BIT_WIDTH                 = 16
    
)(
    input logic                         clk,
    input logic                         rst_n,
    
    // mel filter signals -----------------------------------------------------
    input logic                         even_mel_valid,
    input logic                         odd_mel_valid,
    input logic [BIT_WIDTH-1:0]         mel_value,
    
    // spi_slave Configuration registers --------------------------------------
    input logic                         spi_en_inf_system_sync,
    
    // binarizer signals ------------------------------------------------------
    input logic                         ready_in,
    output logic                        valid_out,
    output logic [BIT_WIDTH-1:0]        mfcc_data_out,
    output logic [BIT_WIDTH-1:0]        flux_data_out
);

    typedef enum logic [1:0] {idle, padding, idle2, receive} state_t;
    
    // FSM signals
    state_t                     p_state, n_state;
    logic                       switch_en;
    logic                       cnt_reset;
    logic                       send_en;
    
    // handshaking signals
    logic                       valid_in;
    logic                       valid_read_flag;
    logic                       valid_read_flag_d1;
    logic  [$clog2(N_MEL)+1:0]  buffer_cnt;
    logic        [BIT_WIDTH:0]  mfcc;
    logic signed [BIT_WIDTH:0]  flux;
    
    // ping-pong buffer signals
    logic                       MEM_BUF0_CEB;
    logic                       MEM_BUF0_WEB;
    logic                       MEM_BUF1_CEB;
    logic                       MEM_BUF1_WEB;
    logic [$clog2(N_MEL)-1:0]   MEM_BUF_A;
    logic [BIT_WIDTH-1:0]       buffer0 [0:N_MEL-1];
    logic [BIT_WIDTH-1:0]       buffer1 [0:N_MEL-1];
    logic [BIT_WIDTH-1:0]       rdata0, rdata1;
    logic                       buffer_sel;
    logic [$clog2(N_MEL):0]     mel_waddr, mel_raddr;
    
    // handshaking logic
    assign valid_in         = even_mel_valid | odd_mel_valid;
    assign valid_read_flag  = (buffer_cnt != 0) && !valid_in && ready_in && !valid_read_flag_d1;
    assign buffer_cnt       = mel_waddr - mel_raddr;
    
    // memory logic
    assign MEM_BUF0_CEB     = !((valid_in && buffer_sel == 0) || (send_en && valid_read_flag));
    assign MEM_BUF0_WEB     = !(valid_in && buffer_sel == 0);
    assign MEM_BUF1_CEB     = !((valid_in && buffer_sel == 1) || (send_en && valid_read_flag));
    assign MEM_BUF1_WEB     = !(valid_in && buffer_sel == 1);
    assign MEM_BUF_A        = (!MEM_BUF0_WEB || !MEM_BUF1_WEB)? mel_waddr[$clog2(N_MEL)-1:0] : mel_raddr[$clog2(N_MEL)-1:0];
    
    // Saturation truncation processing
    always_comb begin
        mfcc = rdata0 + rdata1;
        if (mfcc[BIT_WIDTH] == 1)
            mfcc = {1'b0, {(BIT_WIDTH){1'b1}}};
    end
    
    // Absolute value processing
    always_comb begin
        flux = rdata0 - rdata1;
        if (flux[BIT_WIDTH] == 1)
            flux = ~flux + 1;
    end
    
    // When the data is valid, write address + 1
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            mel_waddr <= 0;
        else if (!spi_en_inf_system_sync)
            mel_waddr <= 0;
        else if (valid_in)
            mel_waddr <= mel_waddr + 1;
        else if (cnt_reset)
            mel_waddr <= 0;
    end
    
    // When binarizer readout the data, read address + 1
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            mel_raddr <= 0;
        else if (!spi_en_inf_system_sync)
            mel_raddr <= 0;
        else if (send_en && valid_read_flag)
            mel_raddr <= mel_raddr + 1;
        else if (cnt_reset)
            mel_raddr <= 0;
    end
    
    // change buffer
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            buffer_sel <= 0;
        else if(switch_en)
            buffer_sel <= ~buffer_sel;
    end
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            valid_read_flag_d1 <= 0;
        else if (send_en && valid_read_flag)
            valid_read_flag_d1 <= 1;
        else
            valid_read_flag_d1 <= 0;
    end
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            valid_out <= 0;
        else if (valid_read_flag_d1)
            valid_out <= 1;
        else
            valid_out <= 0;
    end
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            mfcc_data_out <= 0;
            flux_data_out <= 0;
        end else if (valid_read_flag_d1) begin
            mfcc_data_out <= mfcc[BIT_WIDTH-1:0];
            flux_data_out <= flux[BIT_WIDTH-1:0];
        end
    end
    
    //-------------------------------------------------------------------------
    // Ping-pong buffer
    //-------------------------------------------------------------------------    
    always_ff @(posedge clk) begin
        if (!MEM_BUF0_CEB) begin
            if (!MEM_BUF0_WEB)
                buffer0[MEM_BUF_A] <= mel_value;
            else
                rdata0 <= buffer0[MEM_BUF_A];
        end
    end
    
    always_ff @(posedge clk) begin
        if (!MEM_BUF1_CEB) begin
            if (!MEM_BUF1_WEB)
                buffer1[MEM_BUF_A] <= mel_value;
            else
                rdata1 <= buffer1[MEM_BUF_A];
        end
    end
    
    //-------------------------------------------------------------------------
    // FSM
    //-------------------------------------------------------------------------
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            p_state <= idle;
        else
            p_state <= n_state;
    end
    
    always_comb begin
        n_state = p_state;
        unique case(p_state)
            idle    :   if (valid_in == 1)              n_state = padding;
            padding :   if (!spi_en_inf_system_sync)    n_state = idle;
                        else if (mel_waddr == N_MEL)    n_state = idle2;
            idle2   :   if (!spi_en_inf_system_sync)    n_state = idle;
                        else if (valid_in == 1)         n_state = receive;
            receive :   if (!spi_en_inf_system_sync)    n_state = idle;
                        else if (mel_raddr == N_MEL)    n_state = idle2;
            default :                                   n_state = idle;
        endcase
    end
    
    always_comb begin
        switch_en   = 0;
        cnt_reset   = 0;
        send_en     = 0;
        unique case(p_state)
            idle    :   ;
            padding :   if (mel_waddr == N_MEL) begin
                            switch_en = 1;
                            cnt_reset = 1;
                        end
            idle2   :   if (valid_in) begin
                            send_en = 1;
                        end
            receive :   begin
                            send_en = 1;
                            if (mel_raddr == N_MEL) begin
                                switch_en   = 1;
                                cnt_reset   = 1;
                                send_en     = 0;
                            end
                        end
            default :   ;
        endcase
    end
        
endmodule
