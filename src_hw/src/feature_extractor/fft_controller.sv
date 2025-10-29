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
// Module: "fft_controller.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: .
//
//==============================================================================

module fft_controller #(
    parameter N_FFT             = 256,
    
    localparam N_STAGE          = $clog2(N_FFT),    // 8
    localparam CNT_WIDTH        = $clog2(N_FFT) + 1 // 9
)(
    input logic         clk,
    input logic         rst_n,
    
    // data buffer signals ----------------------------------------------------
    input logic         buf_almost_rfull,
    input logic         buf_rempty,
    output logic        r_req,
    
    // spi_slave Configuration registers --------------------------------------
    input logic         spi_en_inf_system_sync,
    
    // FFT signals ------------------------------------------------------------
    output logic        valid,
    output logic        FSM_pre_store_en    [N_STAGE],
    output logic        FSM_calc_en         [N_STAGE],
    output logic        FSM_data_in_buf_ren [N_STAGE]
);
    
    
    typedef enum logic [1:0] {idle, data_in, data_out} state_t;
    
    // FSM signals
    state_t p_state, n_state;
    logic cnt_en;
    
    logic [CNT_WIDTH-1:0] counter;  // [8:0]
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            p_state <= idle;
        else
            p_state <= n_state;
    end
    
    always_comb begin
        n_state = p_state;
        unique case(p_state)
            idle    :if (buf_almost_rfull == 1)                              n_state = data_in;
            data_in :if (!spi_en_inf_system_sync || counter == N_FFT - 1)   n_state = data_out;
            data_out:if (!spi_en_inf_system_sync || counter == N_FFT*2 - 1) n_state = idle;
            default:                                                        n_state = idle;
        endcase
    end
    
    always_comb begin
        r_req   = 0;
        cnt_en  = 0;
        unique case(p_state)
            idle    : ;
            data_in : begin
                        if (counter != N_FFT - 1)
                            r_req = ~buf_rempty;
                        if (valid == 1)
                            cnt_en = 1;
                      end
            data_out:   cnt_en = 1;
            default : ;
        endcase
    end
    
    
    for (genvar i = 0; i < N_STAGE; i++) begin
        always_comb begin
            FSM_pre_store_en[i]     = (counter[$clog2(N_FFT) - 1 - i] == 1'b0);
            FSM_calc_en[i]          = (counter[$clog2(N_FFT) - 1 - i] == 1'b1);
        end
    end
    
    always_comb begin
        FSM_data_in_buf_ren[0] = counter >= 128 - 1;
        FSM_data_in_buf_ren[1] = counter >= 128 + 64 - 1;
        FSM_data_in_buf_ren[2] = counter >= 128 + 64 + 32 - 1;
        FSM_data_in_buf_ren[3] = counter >= 128 + 64 + 32 + 16;
        FSM_data_in_buf_ren[4] = counter >= 128 + 64 + 32 + 16 + 8;
        FSM_data_in_buf_ren[5] = counter >= 128 + 64 + 32 + 16 + 8 + 4;
        FSM_data_in_buf_ren[6] = counter >= 128 + 64 + 32 + 16 + 8 + 4 + 2;
        FSM_data_in_buf_ren[7] = counter >= 128 + 64 + 32 + 16 + 8 + 4 + 2 + 1;
    end
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            counter <= 0;
        else if (cnt_en)
            counter <= counter + 1'b1;
        else
            counter <= 0;
    end
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            valid <= 0;
        else
            valid <= r_req;
    end
    
endmodule
