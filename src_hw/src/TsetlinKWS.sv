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
// Module: "TsetlinKWS.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: TsetlinKWS top module.
//
//==============================================================================

module TsetlinKWS #(
    
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
    parameter DATAOUT_WIDTH         = 16,
    
    localparam N_PE_CLUSTER         = N_ELEMENT * N_PE_COL
)(

    // system clock signals ---------------------------------------------------
    input logic                         sys_clk,
    input logic                         sys_rst_n,
    
    // i2s signals ------------------------------------------------------------
    input logic                         MCLK,
    input logic                         MCLK_rst_n,
    input logic                         ADC_SDATA,
    output logic                        BCLK,
    output logic                        LRCLK,
    
    // spi slave signals ------------------------------------------------------
    input logic                         SCK,
    input logic                         CS,
    input logic                         MOSI,
    
    output logic [3:0]                  Result,
    output logic                        Inf_Done
);
    
    // feature extractor signals
    logic                                   feature_bank_ren;
    logic [$clog2(2*N_MEL)-1:0]             feature_rptr;
    logic [N_FRAME-1:0]                     feature_bank_rdata;
    logic                                   fe_complete;
    
    // spi_slave signals to tsetlin machine model bank 
    logic                                   SPI_WEN_BLOCK_BANK;
    logic                                   SPI_WEN_ROW_BANK        [N_PE_COL];
    logic                                   SPI_WEN_CCL_BANK        [N_PE_COL];
    logic                                   SPI_WEN_WEIGHT_BANK;
    logic                                   SPI_WEN_FE_BANK;         
    logic [11:0]                            SPI_ADDR;
    logic [31:0]                            SPI_DATA;
    
    // spi_slave signals to tsetlin_machine_accelerator and feature_extractor
    logic                                   SPI_EN_CONF;
    logic                                   SPI_EN_INF;
    logic                                   SPI_EN_FE;
    logic [3:0]                             SPI_NUM_CLASS;
    logic [7:0]                             SPI_NUM_CLAUSE;
    logic [5:0]                             SPI_NUM_SUM_TIME;
    logic [15:0]                            SPI_FLUX_TH;
    logic [$clog2(DEPTH_BLOCK_BANK)-1:0]    SPI_LEN_BLOCK_BANK;
    logic [$clog2(DEPTH_ROW_BANK)-1:0]      SPI_LEN_ROW_BANK        [N_PE_COL];
    logic [$clog2(DEPTH_CCL_BANK)-1:0]      SPI_LEN_CCL_BANK        [N_PE_COL];
    logic [$clog2(DEPTH_WEIGHT_BANK)-1:0]   SPI_LEN_WEIGHT_BANK;
    
    
    feature_extractor #(
        .N_FFT                          (N_FFT                  ),
        .N_FRAME                        (N_FRAME                ),
        .N_MEL                          (N_MEL                  ),
        
        .I2S_DATA_WIDTH                 (I2S_DATA_WIDTH         ),
        .Input_INT_BIT_WIDTH            (Input_INT_BIT_WIDTH    ),
        .Input_FRA_BIT_WIDTH            (Input_FRA_BIT_WIDTH    ),
        .STAGE1_INT_BIT_WIDTH           (STAGE1_INT_BIT_WIDTH   ),
        .STAGE1_FRA_BIT_WIDTH           (STAGE1_FRA_BIT_WIDTH   ),
        .STAGE2_INT_BIT_WIDTH           (STAGE2_INT_BIT_WIDTH   ),
        .STAGE2_FRA_BIT_WIDTH           (STAGE2_FRA_BIT_WIDTH   ),
        .STAGE3_INT_BIT_WIDTH           (STAGE3_INT_BIT_WIDTH   ),
        .STAGE3_FRA_BIT_WIDTH           (STAGE3_FRA_BIT_WIDTH   ),
        .STAGE4_INT_BIT_WIDTH           (STAGE4_INT_BIT_WIDTH   ),
        .STAGE4_FRA_BIT_WIDTH           (STAGE4_FRA_BIT_WIDTH   ),
        .STAGE5_INT_BIT_WIDTH           (STAGE5_INT_BIT_WIDTH   ),
        .STAGE5_FRA_BIT_WIDTH           (STAGE5_FRA_BIT_WIDTH   ),
        .STAGE6_INT_BIT_WIDTH           (STAGE6_INT_BIT_WIDTH   ),
        .STAGE6_FRA_BIT_WIDTH           (STAGE6_FRA_BIT_WIDTH   ),
        .STAGE7_INT_BIT_WIDTH           (STAGE7_INT_BIT_WIDTH   ),
        .STAGE7_FRA_BIT_WIDTH           (STAGE7_FRA_BIT_WIDTH   ),
        .STAGE8_INT_BIT_WIDTH           (STAGE8_INT_BIT_WIDTH   ),
        .STAGE8_FRA_BIT_WIDTH           (STAGE8_FRA_BIT_WIDTH   ),
        .TW_BIT_WIDTH                   (TW_BIT_WIDTH           ),
        .DATAOUT_WIDTH                  (DATAOUT_WIDTH          )
        
    ) feature_extractor_inst(
        // system clock signals ---------------------------------------------------
        .sys_clk                        (sys_clk                ),
        .sys_rst_n                      (sys_rst_n              ),
        
        // i2s signals ------------------------------------------------------------
        .MCLK                           (MCLK                   ),
        .MCLK_rst_n                     (MCLK_rst_n             ),
        .ADC_SDATA                      (ADC_SDATA              ),
        .BCLK                           (BCLK                   ),
        .LRCLK                          (LRCLK                  ),
        
        // spi_slave signals --------------------------------------------------
        .SPI_WEN_FE_BANK                (SPI_WEN_FE_BANK        ),
        .SPI_ADDR                       (SPI_ADDR               ),
        .SPI_DATA                       (SPI_DATA               ),
        
        // spi_slave Configuration registers ----------------------------------
        .SPI_EN_INF                     (SPI_EN_INF             ),
        .SPI_EN_FE                      (SPI_EN_FE              ),
        .SPI_FLUX_TH                    (SPI_FLUX_TH            ),
        
        // accelerator signals ------------------------------------------------
        .feature_bank_ren               (feature_bank_ren       ),
        .feature_rptr                   (feature_rptr           ),
        .feature_bank_rdata             (feature_bank_rdata     ),
        .fe_complete                    (fe_complete            )
    );
    
    
    tsetlin_machine_accelerator #(
        .N_PE_COL                       (N_PE_COL               ),
        .N_ELEMENT                      (N_ELEMENT              ),
        .N_MEL                          (N_MEL                  ),
        .N_FRAME                        (N_FRAME                ),
        .NUMBER_OF_PATCH                (NUMBER_OF_PATCH        ),
        .DEPTH_BLOCK_BANK               (DEPTH_BLOCK_BANK       ),
        .DEPTH_ROW_BANK                 (DEPTH_ROW_BANK         ),
        .DEPTH_CCL_BANK                 (DEPTH_CCL_BANK         ),
        .DEPTH_WEIGHT_BANK              (DEPTH_WEIGHT_BANK      )

    ) tsetlin_machine_accelerator_inst(
        .clk                            (sys_clk                ),
        .rst_n                          (sys_rst_n              ),
        
        // feature bank signals -----------------------------------------------
        .fe_complete                    (fe_complete            ),
        .feature_bank_rdata             (feature_bank_rdata     ),
        .feature_bank_ren               (feature_bank_ren       ),
        .feature_rptr                   (feature_rptr           ),
        
        // spi_slave signals --------------------------------------------------
        .SPI_WEN_BLOCK_BANK             (SPI_WEN_BLOCK_BANK     ),
        .SPI_WEN_ROW_BANK               (SPI_WEN_ROW_BANK       ),
        .SPI_WEN_CCL_BANK               (SPI_WEN_CCL_BANK       ),
        .SPI_WEN_WEIGHT_BANK            (SPI_WEN_WEIGHT_BANK    ),
        .SPI_ADDR                       (SPI_ADDR               ),
        .SPI_DATA                       (SPI_DATA               ),
        
        // spi_slave Configuration registers ----------------------------------
        .SPI_NUM_CLASS                  (SPI_NUM_CLASS          ),
        .SPI_NUM_CLAUSE                 (SPI_NUM_CLAUSE         ),
        .SPI_NUM_SUM_TIME               (SPI_NUM_SUM_TIME       ),
        .SPI_LEN_BLOCK_BANK             (SPI_LEN_BLOCK_BANK     ),
        .SPI_LEN_ROW_BANK               (SPI_LEN_ROW_BANK       ),
        .SPI_LEN_CCL_BANK               (SPI_LEN_CCL_BANK       ),
        .SPI_LEN_WEIGHT_BANK            (SPI_LEN_WEIGHT_BANK    ),
        
        // result signals -----------------------------------------------------
        .Result                         (Result                 ),
        .Inf_Done                       (Inf_Done               )
    );


    spi_slave #(
        .N_PE_COL                       (N_PE_COL               ),
        .DEPTH_BLOCK_BANK               (DEPTH_BLOCK_BANK       ),
        .DEPTH_ROW_BANK                 (DEPTH_ROW_BANK         ),
        .DEPTH_CCL_BANK                 (DEPTH_CCL_BANK         ),
        .DEPTH_WEIGHT_BANK              (DEPTH_WEIGHT_BANK      )
        
    ) spi_slave_inst(
        // SPI interface ------------------------------------------------------
        .SCK                            (SCK                    ),
        .CS                             (CS                     ),
        .MOSI                           (MOSI                   ),
        
        // System inputs ------------------------------------------------------
        .rst_n                          (sys_rst_n              ),
        
        // outputs to system --------------------------------------------------
        .SPI_WEN_BLOCK_BANK             (SPI_WEN_BLOCK_BANK     ),
        .SPI_WEN_ROW_BANK               (SPI_WEN_ROW_BANK       ),
        .SPI_WEN_CCL_BANK               (SPI_WEN_CCL_BANK       ),
        .SPI_WEN_WEIGHT_BANK            (SPI_WEN_WEIGHT_BANK    ),
        .SPI_WEN_FE_BANK                (SPI_WEN_FE_BANK        ),
        .SPI_ADDR                       (SPI_ADDR               ),
        .SPI_DATA                       (SPI_DATA               ),
        
        // Configuration registers --------------------------------------------
        .SPI_EN_CONF                    (SPI_EN_CONF            ),
        .SPI_EN_INF                     (SPI_EN_INF             ),
        .SPI_EN_FE                      (SPI_EN_FE              ),
        .SPI_NUM_CLASS                  (SPI_NUM_CLASS          ),
        .SPI_NUM_CLAUSE                 (SPI_NUM_CLAUSE         ),
        .SPI_NUM_SUM_TIME               (SPI_NUM_SUM_TIME       ),
        .SPI_FLUX_TH                    (SPI_FLUX_TH            ),
        .SPI_LEN_BLOCK_BANK             (SPI_LEN_BLOCK_BANK     ),
        .SPI_LEN_ROW_BANK               (SPI_LEN_ROW_BANK       ),
        .SPI_LEN_CCL_BANK               (SPI_LEN_CCL_BANK       ),
        .SPI_LEN_WEIGHT_BANK            (SPI_LEN_WEIGHT_BANK    )
    );
    
    
endmodule
