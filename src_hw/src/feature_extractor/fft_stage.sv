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
// Module: "fft_stage.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: The radix-2 single-path delay feedback (R2-SDF) FFT stage architecture.
//
//==============================================================================

module fft_stage #(
    parameter Last_INT_BIT_WIDTH    = 12,
    parameter Last_FRA_BIT_WIDTH    = 0,
    parameter INT_BIT_WIDTH         = 12,
    parameter FRA_BIT_WIDTH         = 1,
    parameter TW_BIT_WIDTH          = 8,
    parameter N_FFT                 = 256,
    parameter NO_STAGE              = 0,
    parameter USE_RAM               = 1,
    parameter USE_ROM               = 1
    
)(
    input logic         clk,
    input logic         rst_n,
    
    // fft controller signals -------------------------------------------------
    input logic         valid_in,
    input logic         FSM_pre_store_en,
    input logic         FSM_calc_en,
    input logic         FSM_data_in_buf_ren,
    
    // spi_slave Configuration registers --------------------------------------
    input logic         spi_en_inf_system_sync,
    
    // data buffer signals ----------------------------------------------------
    input logic signed  [Last_INT_BIT_WIDTH + Last_FRA_BIT_WIDTH - 1:0] Re_in,
    input logic signed  [Last_INT_BIT_WIDTH + Last_FRA_BIT_WIDTH - 1:0] Im_in,
    
    // signals to next stage --------------------------------------------------
    output logic                                                        valid_out,
    output logic signed [INT_BIT_WIDTH + FRA_BIT_WIDTH - 1:0]           Re_out,
    output logic signed [INT_BIT_WIDTH + FRA_BIT_WIDTH - 1:0]           Im_out
);
    
    localparam BUFFER_DEPTH = N_FFT / (1 << (NO_STAGE + 1));
    localparam BUFFER_ADDR_WIDTH = (BUFFER_DEPTH > 1) ? $clog2(BUFFER_DEPTH) : 0;
    localparam TWIDDLE_ADDR_WIDTH = (BUFFER_DEPTH > 1) ? $clog2(BUFFER_DEPTH): 1;
    
    localparam LAST_WIDTH = Last_INT_BIT_WIDTH + Last_FRA_BIT_WIDTH;
    localparam CURR_WIDTH = INT_BIT_WIDTH + FRA_BIT_WIDTH;
    
    logic signed    [LAST_WIDTH:0]       bf_Re_a;
    logic signed    [LAST_WIDTH:0]       bf_Im_a;
    logic signed    [LAST_WIDTH - 1:0]   bf_Re_b;
    logic signed    [LAST_WIDTH - 1:0]   bf_Im_b;
    logic signed    [LAST_WIDTH:0]       bf_Re_c;
    logic signed    [LAST_WIDTH:0]       bf_Im_c;
    logic signed    [LAST_WIDTH:0]       bf_Re_d;
    logic signed    [LAST_WIDTH:0]       bf_Im_d;
    
    // control signals
    logic                                               pre_store_en;   
    logic                                               calc_en;
    logic                                               result_store_en;
    logic                                               send_en;
    
    // data buffer signals
    logic signed    [2 * (LAST_WIDTH + 1) - 1:0]        buffer  [0:BUFFER_DEPTH - 1];
    logic           [BUFFER_ADDR_WIDTH:0]               r_ptr, w_ptr;
    logic                                               buffer_ren, buffer_wen;
    logic                                               buffer_empty, buffer_full;
    logic signed    [2 * (LAST_WIDTH + 1) - 1:0]        buffer_rdata, buffer_wdata;
    logic signed    [LAST_WIDTH:0]                      Re_buffer_wdata;
    logic signed    [LAST_WIDTH:0]                      Im_buffer_wdata;
    logic signed    [LAST_WIDTH:0]                      Re_buffer_rdata;
    logic signed    [LAST_WIDTH:0]                      Im_buffer_rdata;
    
    // multiplier signals
    logic                                               mul_en;
    logic signed    [LAST_WIDTH:0]                      mult_Re_a;
    logic signed    [LAST_WIDTH:0]                      mult_Im_a;
    logic signed    [TW_BIT_WIDTH - 1:0]                mult_Re_b;
    logic signed    [TW_BIT_WIDTH - 1:0]                mult_Im_b;
    
    // twiddle bank signals
    logic                                               tw_ren;
    logic           [TWIDDLE_ADDR_WIDTH - 1:0]          tw_addr;
    logic signed    [TW_BIT_WIDTH - 1:0]                Re_twddle;
    logic signed    [TW_BIT_WIDTH - 1:0]                Im_twddle;
    logic signed    [LAST_WIDTH + TW_BIT_WIDTH + 1:0]   Re_mult;
    logic signed    [LAST_WIDTH + TW_BIT_WIDTH + 1:0]   Im_mult;
    
    assign valid_out = send_en;
    
    // butterfly module
    butterfly #(
        .LAST_WIDTH         (LAST_WIDTH         )
   
    ) butterfly_inst(
        .bf_en              (calc_en            ),
        .Re_a               (bf_Re_a            ),
        .Im_a               (bf_Im_a            ),
        .Re_b               (bf_Re_b            ),
        .Im_b               (bf_Im_b            ),
                        
        .Re_c               (bf_Re_c            ),
        .Im_c               (bf_Im_c            ),
        .Re_d               (bf_Re_d            ),
        .Im_d               (bf_Im_d            )
    );
    
    
generate
if (NO_STAGE != $clog2(N_FFT) - 1) begin

    // twiddle bank and complex multiplier module
    twiddle_bank #(
        .TW_BIT_WIDTH       (TW_BIT_WIDTH       ),
        .N_FFT              (N_FFT              ),
        .NO_STAGE           (NO_STAGE           ),
        .BANK_DEPTH         (BUFFER_DEPTH       ),
        .BANK_ADDR_WIDTH    (TWIDDLE_ADDR_WIDTH ),
        .USE_ROM            (USE_ROM            )
    
    ) twiddle_bank_inst(  
        .clk                (clk                ),
        .tw_ren             (tw_ren             ),
        .tw_addr            (tw_addr            ),
            
        .Re_twddle          (Re_twddle          ),
        .Im_twddle          (Im_twddle          )
    );
    
    complex_multiplier #(
        .LAST_WIDTH         (LAST_WIDTH         ),
        .TW_BIT_WIDTH       (TW_BIT_WIDTH       )
    
    ) complex_multiplier_inst(
        .mult_en            (mul_en             ),
        .Re_a               (mult_Re_a          ),
        .Im_a               (mult_Im_a          ),
        .Re_b               (mult_Re_b          ),
        .Im_b               (mult_Im_b          ),
                        
        .Re_mult            (Re_mult            ),
        .Im_mult            (Im_mult            )
    );
    
end else begin
    
    // do not need multipliers

end    
endgenerate

    //control logic
    assign pre_store_en     = valid_in & FSM_pre_store_en;
    assign calc_en          = valid_in & FSM_calc_en;
    assign result_store_en  = calc_en;
    assign buffer_empty     = r_ptr == w_ptr;
generate if (NO_STAGE != $clog2(N_FFT) - 1) begin
    assign buffer_full      = w_ptr == {~r_ptr[BUFFER_ADDR_WIDTH], r_ptr[BUFFER_ADDR_WIDTH-1:0]};
end else begin
    assign buffer_full      = w_ptr == ~r_ptr;
end endgenerate
    assign buffer_wen       = valid_in;
    assign buffer_ren       = valid_in ? FSM_data_in_buf_ren : ~buffer_empty;
    
    generate
        if (USE_RAM == 1) begin
            
            always_ff @(posedge clk, negedge rst_n) begin
                if (!rst_n)
                    send_en <= 0;
                else
                    send_en <= buffer_ren;
            end
            
        end else begin
        
            assign send_en = buffer_ren;
            
        end
    endgenerate
    
    generate
        if (USE_ROM == 1)   assign tw_ren = buffer_ren;
        else                assign tw_ren = (~calc_en) & send_en;
    endgenerate
    
    assign bf_Re_a = Re_buffer_rdata;
    assign bf_Im_a = Im_buffer_rdata;
    assign bf_Re_b = Re_in;
    assign bf_Im_b = Im_in;
    assign buffer_wdata = {Re_buffer_wdata, Im_buffer_wdata};
    assign {Re_buffer_rdata, Im_buffer_rdata} = buffer_rdata;
    
    generate 
        if (CURR_WIDTH > LAST_WIDTH) begin
            assign Re_buffer_wdata = pre_store_en? {{(CURR_WIDTH - LAST_WIDTH){Re_in[LAST_WIDTH - 1]}}, Re_in[LAST_WIDTH - 1:0]} : 
                                    result_store_en? bf_Re_c : '0;
            assign Im_buffer_wdata = pre_store_en? {{(CURR_WIDTH - LAST_WIDTH){Im_in[LAST_WIDTH - 1]}}, Im_in[LAST_WIDTH - 1:0]} : 
                                    result_store_en? bf_Im_c : '0;
        end else if (CURR_WIDTH <= LAST_WIDTH) begin
            assign Re_buffer_wdata = pre_store_en? {{Re_in[LAST_WIDTH - 1]}, Re_in[LAST_WIDTH - 1:0]} : 
                                    result_store_en? bf_Re_c : '0;
            assign Im_buffer_wdata = pre_store_en? {{Im_in[LAST_WIDTH - 1]}, Im_in[LAST_WIDTH - 1:0]} : 
                                    result_store_en? bf_Im_c : '0;
        end
    endgenerate
    
    // multiplier control logic
    generate
        if (USE_ROM == 1) begin

             always_ff @(posedge clk, negedge rst_n) begin
                if(!rst_n)
                    mul_en <= 0;
                else
                    mul_en <= tw_ren;
            end
        
        end else begin
        
            assign mul_en = tw_ren;
        
        end
    endgenerate
    
    assign mult_Re_a = Re_buffer_rdata;
    assign mult_Im_a = Im_buffer_rdata;
    assign mult_Re_b = Re_twddle;
    assign mult_Im_b = Im_twddle;
    
    generate 
        if (NO_STAGE != $clog2(N_FFT) - 1) begin
            
            if (FRA_BIT_WIDTH >= Last_FRA_BIT_WIDTH) begin
                assign Re_out = calc_en? bf_Re_d <<< (FRA_BIT_WIDTH - Last_FRA_BIT_WIDTH) :
                                        Re_mult >>> (TW_BIT_WIDTH - 1 - (FRA_BIT_WIDTH - Last_FRA_BIT_WIDTH));
                assign Im_out = calc_en? bf_Im_d <<< (FRA_BIT_WIDTH - Last_FRA_BIT_WIDTH) :
                                        Im_mult >>> (TW_BIT_WIDTH - 1 - (FRA_BIT_WIDTH - Last_FRA_BIT_WIDTH));
            end else begin
                assign Re_out = calc_en? bf_Re_d >>> (Last_FRA_BIT_WIDTH - FRA_BIT_WIDTH) :
                                        Re_mult >>> (TW_BIT_WIDTH - 1 - (FRA_BIT_WIDTH - Last_FRA_BIT_WIDTH));
                assign Im_out = calc_en? bf_Im_d >>> (Last_FRA_BIT_WIDTH - FRA_BIT_WIDTH) :
                                        Im_mult >>> (TW_BIT_WIDTH - 1 - (FRA_BIT_WIDTH - Last_FRA_BIT_WIDTH));
            end
            
        end else begin
            
            if (FRA_BIT_WIDTH >= Last_FRA_BIT_WIDTH) begin
                assign Re_out = bf_Re_d <<< (FRA_BIT_WIDTH - Last_FRA_BIT_WIDTH);
                assign Im_out = bf_Im_d <<< (FRA_BIT_WIDTH - Last_FRA_BIT_WIDTH);
            end else begin
                assign Re_out = bf_Re_d >>> (Last_FRA_BIT_WIDTH - FRA_BIT_WIDTH);
                assign Im_out = bf_Im_d >>> (Last_FRA_BIT_WIDTH - FRA_BIT_WIDTH);
            end
            
        end
    endgenerate
    
    // twiddle factor address -------------------------------------------------
    generate
        if (NO_STAGE != $clog2(N_FFT) - 1) begin
        
            always_ff @(posedge clk, negedge rst_n) begin
                if (!rst_n)
                    tw_addr <= 0;
                else if (tw_ren)
                    tw_addr <= tw_addr + 1'b1;
                else
                    tw_addr <= 0;
            end
            
        end else begin
        
            // do not need tw_addr
            
        end
    endgenerate
    
    //-------------------------------------------------------------------------
    // Data buffer
    //-------------------------------------------------------------------------
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            r_ptr <= 0;
        else if (!spi_en_inf_system_sync)
            r_ptr <= 0;
        else if (buffer_ren)
            r_ptr <= r_ptr + 1'b1;
    end
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            w_ptr <= 0;
        else if (!spi_en_inf_system_sync)
            w_ptr <= 0;
        else if (buffer_wen)
            w_ptr <= w_ptr + 1'b1;
    end

    
    generate 
        if (NO_STAGE != $clog2(N_FFT) - 1) begin
            if (USE_RAM == 1) begin
                
                always_ff @(posedge clk) begin
                    if (buffer_wen | buffer_ren) begin
                        if (buffer_wen) begin
                            buffer[w_ptr[BUFFER_ADDR_WIDTH - 1:0]] <= buffer_wdata;
                        end
                        if (buffer_ren) begin
                            buffer_rdata <= buffer[r_ptr[BUFFER_ADDR_WIDTH - 1:0]];
                        end
                    end
                end
                
            end else begin
                
                always_ff @(posedge clk) begin
                    if (buffer_wen) begin
                        buffer[w_ptr[BUFFER_ADDR_WIDTH - 1:0]] <= buffer_wdata;
                    end
                end
                
                assign buffer_rdata = buffer_ren? buffer[r_ptr[BUFFER_ADDR_WIDTH - 1:0]] : 'x;
                
            end
        end else begin
        
            always_ff @(posedge clk) begin
                if (buffer_wen) begin
                    buffer_rdata <= buffer_wdata;
                end
            end
            
        end
    endgenerate
    
    
endmodule
