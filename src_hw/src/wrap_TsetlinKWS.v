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
// Module: "wrap_TsetlinKWS.v"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: wrap TsetlinKWS module.
//
//==============================================================================

module wrap_TsetlinKWS #(
   
    parameter N_FFT                 = 256,
    parameter N_FRAME               = 64,
    parameter N_MEL                 = 32,
    parameter N_PE_COL              = 5,
    parameter N_ELEMENT             = 4,
    parameter NUMBER_OF_PATCH       = 58,
    parameter DEPTH_BLOCK_BANK      = 2048,
    parameter DEPTH_ROW_BANK        = 2048,
    parameter DEPTH_CCL_BANK        = 4096,
    parameter DEPTH_WEIGHT_BANK     = 2048,
    
    parameter I2S_DATA_WIDTH        = 24,
    parameter Input_INT_BIT_WIDTH   = 12,
    parameter Input_FRA_BIT_WIDTH   = 0,
    parameter STAGE1_INT_BIT_WIDTH  = 12,
    parameter STAGE1_FRA_BIT_WIDTH  = 1,
    parameter STAGE2_INT_BIT_WIDTH  = 13,
    parameter STAGE2_FRA_BIT_WIDTH  = 1,
    parameter STAGE3_INT_BIT_WIDTH  = 13,
    parameter STAGE3_FRA_BIT_WIDTH  = 1,
    parameter STAGE4_INT_BIT_WIDTH  = 13,
    parameter STAGE4_FRA_BIT_WIDTH  = 1,
    parameter STAGE5_INT_BIT_WIDTH  = 14,
    parameter STAGE5_FRA_BIT_WIDTH  = 1,
    parameter STAGE6_INT_BIT_WIDTH  = 14,
    parameter STAGE6_FRA_BIT_WIDTH  = 0,
    parameter STAGE7_INT_BIT_WIDTH  = 14,
    parameter STAGE7_FRA_BIT_WIDTH  = 0,
    parameter STAGE8_INT_BIT_WIDTH  = 15,
    parameter STAGE8_FRA_BIT_WIDTH  = 0,
    parameter TW_BIT_WIDTH          = 8,
    parameter DATAOUT_WIDTH         = 16
)(

    // system clock signals ---------------------------------------------------
    input wire                         sys_clk,
    input wire                         rst_n,
    
    // i2s signals ------------------------------------------------------------
    input wire                         MCLK,
    input wire                         ADC_SDATA,
    output wire                        BCLK,
    output wire                        LRCLK,
    
    // spi slave signals ------------------------------------------------------
    input wire                         SCK,
    input wire                         CS,
    input wire                         MOSI,
    
    output wire [3:0]                  Result,
    output wire                        Inf_Done
);
    
    wire    SYNC_sys_rst_n,     SYNC_MID_sys_rst_n;
    wire    SYNC_MCLK_rst_n,    SYNC_MID_MCLK_rst_n;
    wire    SYNC_BCLK_rst_n,    SYNC_MID_BCLK_rst_n;
    wire    SYNC_LRCLK_rst_n,   SYNC_MID_LRCLK_rst_n;
    
    DFCND1 RESET_SYNC_FF1_sys(
        .CP(sys_clk),    .CDN(rst_n),   .D(1'b1),                   .Q(SYNC_MID_sys_rst_n)
    );
    
    DFCND1 RESET_SYNC_FF2_sys(
        .CP(sys_clk),    .CDN(rst_n),   .D(SYNC_MID_sys_rst_n),     .Q(SYNC_sys_rst_n)
    );
    
    DFCND1 RESET_SYNC_FF1_MCLK(
        .CP(MCLK),       .CDN(rst_n),   .D(1'b1),                   .Q(SYNC_MID_MCLK_rst_n)
    );
    
    DFCND1 RESET_SYNC_FF2_MCLK(
        .CP(MCLK),       .CDN(rst_n),   .D(SYNC_MID_MCLK_rst_n),    .Q(SYNC_MCLK_rst_n)
    );
    
    TsetlinKWS #(
        .N_FFT                      (N_FFT                      ),
        .N_FRAME                    (N_FRAME                    ),
        .N_MEL                      (N_MEL                      ),
        .N_PE_COL                   (N_PE_COL                   ),
        .N_ELEMENT                  (N_ELEMENT                  ),
        .NUMBER_OF_PATCH            (NUMBER_OF_PATCH            ),
        .DEPTH_BLOCK_BANK           (DEPTH_BLOCK_BANK           ),
        .DEPTH_ROW_BANK             (DEPTH_ROW_BANK             ),
        .DEPTH_CCL_BANK             (DEPTH_CCL_BANK             ),
        .DEPTH_WEIGHT_BANK          (DEPTH_WEIGHT_BANK          ),
        
        .I2S_DATA_WIDTH             (I2S_DATA_WIDTH             ),
        .Input_INT_BIT_WIDTH        (Input_INT_BIT_WIDTH        ),
        .Input_FRA_BIT_WIDTH        (Input_FRA_BIT_WIDTH        ),
        .STAGE1_INT_BIT_WIDTH       (STAGE1_INT_BIT_WIDTH       ),
        .STAGE1_FRA_BIT_WIDTH       (STAGE1_FRA_BIT_WIDTH       ),
        .STAGE2_INT_BIT_WIDTH       (STAGE2_INT_BIT_WIDTH       ),
        .STAGE2_FRA_BIT_WIDTH       (STAGE2_FRA_BIT_WIDTH       ),
        .STAGE3_INT_BIT_WIDTH       (STAGE3_INT_BIT_WIDTH       ),
        .STAGE3_FRA_BIT_WIDTH       (STAGE3_FRA_BIT_WIDTH       ),
        .STAGE4_INT_BIT_WIDTH       (STAGE4_INT_BIT_WIDTH       ),
        .STAGE4_FRA_BIT_WIDTH       (STAGE4_FRA_BIT_WIDTH       ),
        .STAGE5_INT_BIT_WIDTH       (STAGE5_INT_BIT_WIDTH       ),
        .STAGE5_FRA_BIT_WIDTH       (STAGE5_FRA_BIT_WIDTH       ),
        .STAGE6_INT_BIT_WIDTH       (STAGE6_INT_BIT_WIDTH       ),
        .STAGE6_FRA_BIT_WIDTH       (STAGE6_FRA_BIT_WIDTH       ),
        .STAGE7_INT_BIT_WIDTH       (STAGE7_INT_BIT_WIDTH       ),
        .STAGE7_FRA_BIT_WIDTH       (STAGE7_FRA_BIT_WIDTH       ),
        .STAGE8_INT_BIT_WIDTH       (STAGE8_INT_BIT_WIDTH       ),
        .STAGE8_FRA_BIT_WIDTH       (STAGE8_FRA_BIT_WIDTH       ),
        .TW_BIT_WIDTH               (TW_BIT_WIDTH               ),
        .DATAOUT_WIDTH              (DATAOUT_WIDTH              )

    ) TsetlinKWS_inst(
        // system clock signals -----------------------------------------------
        .sys_clk                    (sys_clk                    ),
        .sys_rst_n                  (SYNC_sys_rst_n             ),

        // i2s signals --------------------------------------------------------
        .MCLK                       (MCLK                       ),
        .MCLK_rst_n                 (SYNC_MCLK_rst_n            ),
        .ADC_SDATA                  (ADC_SDATA                  ),
        .BCLK                       (BCLK                       ),
        .LRCLK                      (LRCLK                      ),

        // spi slave signals --------------------------------------------------
        .SCK                        (SCK                        ),
        .CS                         (CS                         ),
        .MOSI                       (MOSI                       ),

        .Result                     (Result                     ),
        .Inf_Done                   (Inf_Done                   )
    );

endmodule
    