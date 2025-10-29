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
// Module: "fft_swap.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: Reorder the frequency domain FFT data.
//
//==============================================================================

module fft_swap #(
    parameter N_FFT                 = 256,
    parameter STAGE8_INT_BIT_WIDTH  = 15,
    parameter STAGE8_FRA_BIT_WIDTH  = 0,
    
    localparam BIT_WIDTH = STAGE8_INT_BIT_WIDTH + STAGE8_FRA_BIT_WIDTH
)(
    input logic                             clk,
    input logic                             rst_n,
    
    // FFT signals ------------------------------------------------------------
    input logic                             valid_in,
    input logic signed [BIT_WIDTH - 1:0]    Re_result,
    input logic signed [BIT_WIDTH - 1:0]    Im_result,
    
    // spi_slave Configuration registers --------------------------------------
    input logic                             spi_en_inf_system_sync,
    
    // MEL filter signals -----------------------------------------------------
    output logic                            data_out_valid,
    output logic [BIT_WIDTH - 1:0]          data_out
    
);

    typedef enum logic [1:0] {idle, receive, send} state_t;
    
    // FSM signals
    state_t p_state, n_state;
    logic                           counter_en;
    logic                           skip_en;
    logic                           valid_out;
    
    // swap and sum signals
    logic                           valid_skip;
    logic [$clog2(N_FFT) - 1:0]     counter;
    logic [BIT_WIDTH - 2:0]         Re_result_abs, Im_result_abs;
    logic [BIT_WIDTH - 1:0]         spectrum;
    logic [$clog2(N_FFT) - 1:0]     addr;
    logic [$clog2(N_FFT/2) - 1:0]   reversed_addr;
    
    // memory signal
    logic [BIT_WIDTH - 1:0]         fft_mem     [0:N_FFT / 2 - 1];
    logic                           MEM_FFT_CEB;
    logic                           MEM_FFT_WEB;
    logic [$clog2(N_FFT/2) - 1:0]   MEM_FFT_ADDRESS;
    
    
    assign Re_result_abs = (Re_result[BIT_WIDTH-1] == 0)? Re_result[BIT_WIDTH - 2:0]:
                                                    (~Re_result[BIT_WIDTH - 2:0] + 1);
    assign Im_result_abs = (Im_result[BIT_WIDTH-1] == 0)? Im_result[BIT_WIDTH - 2:0]:
                                                    (~Im_result[BIT_WIDTH - 2:0] + 1);
    assign spectrum = Re_result_abs + Im_result_abs;
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            counter <= 0;
        end else if (counter_en) begin
            counter <= counter + 1;
        end else begin
            counter <= 0;
        end
    end
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            valid_skip <= 0;
        end else if (skip_en) begin
            valid_skip <= ~valid_skip;
        end else begin
            valid_skip <= 0;
        end
    end
    
    // reverse the index
    assign addr = counter;
    always_comb begin
        reversed_addr = 0;
        for (int i = 0; i <= $clog2(N_FFT/2) - 1; i++) begin
            reversed_addr[i] = addr[$clog2(N_FFT) - 1 - i];
        end
    end
    
    // According to the reversed_addr, write the data to the fft_mem.
    assign MEM_FFT_CEB      = !((valid_in && !valid_skip) | valid_out);
    assign MEM_FFT_WEB      = !(valid_in && !valid_skip);
    assign MEM_FFT_ADDRESS  = (valid_in && !valid_skip)? reversed_addr : counter[$clog2(N_FFT/2) - 1:0];
    
    always_ff @(posedge clk) begin
        if (!MEM_FFT_CEB) begin
            if (!MEM_FFT_WEB)
                fft_mem[reversed_addr] <= spectrum;
            else
                data_out <= fft_mem[counter[$clog2(N_FFT/2) - 1:0]];
        end
    end
    
    always_ff @(posedge clk, negedge rst_n) begin
        if(!rst_n)
            data_out_valid <= 0;
        else
            data_out_valid <= valid_out;
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
            idle    :   if (valid_in == 1)                      n_state = receive;
            receive :   if (!spi_en_inf_system_sync)            n_state = idle;
                        else if (counter == N_FFT - 1)          n_state = send;
            send    :   if (!spi_en_inf_system_sync ||            
                                (counter == N_FFT/2 - 1))       n_state = idle;
            default :                                           n_state = idle;
        endcase
    end
    
    always_comb begin
        counter_en  = 0;
        skip_en     = 0;
        valid_out   = 0;
        unique case(p_state)
            idle    :   if (valid_in == 1) begin
                            counter_en  = 1;
                            skip_en     = 1;
                        end
            receive :   begin
                            counter_en  = 1;
                            skip_en     = 1;
                        end
            send    :   begin
                            counter_en  = 1;
                            valid_out   = 1;
                        end
            default :   ;
        endcase
    end

endmodule
