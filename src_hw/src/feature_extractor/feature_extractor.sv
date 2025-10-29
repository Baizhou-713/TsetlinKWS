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
// Module: "feature_extractor.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: Feature extractor top module.
//
//==============================================================================

module feature_extractor #(
    parameter N_FFT                 = 256,
    parameter N_FRAME               = 64,
    parameter N_MEL                 = 32,
    
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
    
    localparam DATAIN_WIDTH         = Input_INT_BIT_WIDTH + Input_FRA_BIT_WIDTH
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
    
    // spi_slave signals ------------------------------------------------------
    input logic                         SPI_WEN_FE_BANK,
    input logic [11:0]                  SPI_ADDR,
    input logic [31:0]                  SPI_DATA,
    
    // spi_slave Configuration registers --------------------------------------
    input logic                         SPI_EN_INF,
    input logic                         SPI_EN_FE,
    input logic [15:0]                  SPI_FLUX_TH,
    
    // accelerator signals ----------------------------------------------------
    input logic                         feature_bank_ren,
    input logic [$clog2(2*N_MEL)-1:0]   feature_rptr,
    output logic [N_FRAME-1:0]          feature_bank_rdata,
    output logic                        fe_complete
);
    
    // spi_slave sync signals
    logic                               spi_wen_fe_bank_d1, spi_wen_fe_bank_d2, spi_wen_fe_bank_d3;
    logic                               spi_wen_fe_bank_sync;
    logic                               spi_en_inf_sample_d1, spi_en_inf_sample_sync;
    logic                               spi_en_fe_sample_d1, spi_en_fe_sample_sync;
    logic                               spi_en_inf_system_d1, spi_en_inf_system_sync;
    logic                               spi_en_fe_system_d1, spi_en_fe_system_sync;
    
    // i2s audio signals
    logic [DATAIN_WIDTH-1:0]            audio_data;
    
    // pre emphassis signals
    logic [DATAIN_WIDTH-1:0]            emp_data;
    
    // Asynchronous FIFO signals
    logic                               databuf_wreq, databuf_rreq;
    logic                               buf_wfull, buf_almost_rfull, buf_rempty;
    logic signed [DATAIN_WIDTH-1:0]     buf_data;

    // FFT signals
    logic                                                               fft_data_valid;
    logic signed [STAGE8_INT_BIT_WIDTH + STAGE8_FRA_BIT_WIDTH - 1:0]    Re_result;
    logic signed [STAGE8_INT_BIT_WIDTH + STAGE8_FRA_BIT_WIDTH - 1:0]    Im_result;
    logic                                                               fft_abs_valid;
    logic [STAGE8_INT_BIT_WIDTH + STAGE8_FRA_BIT_WIDTH-1:0]             fft_abs_data;
    
    // MEL filter signals
    logic                               even_mel_valid;
    logic                               odd_mel_valid;
    logic [DATAOUT_WIDTH-1:0]           mel_value;
    logic                               mfcc_flux_valid_out;
    logic [DATAOUT_WIDTH-1:0]           mfcc_data;
    logic [DATAOUT_WIDTH-1:0]           flux_data;
    
    // Binarizer signals
    logic                               ready_bin2buffer;
    
    // MFCC cricular buffer signals
    logic [DATAOUT_WIDTH-1:0]           mfcc_cirbuf_rdata0;
    logic [DATAOUT_WIDTH-1:0]           mfcc_cirbuf_rdata1;
    logic [DATAOUT_WIDTH-1:0]           mfcc_cirbuf_rdata2;
    logic [DATAOUT_WIDTH-1:0]           mfcc_cirbuf_rdata3;
    logic [DATAOUT_WIDTH-1:0]           mfcc_cirbuf_rdata4;
    logic [DATAOUT_WIDTH-1:0]           mfcc_cirbuf_rdata5;
    logic [DATAOUT_WIDTH-1:0]           mfcc_cirbuf_rdata6;
    logic [DATAOUT_WIDTH-1:0]           mfcc_cirbuf_rdata7;
    logic                               MEM_MFCC_CIRBUF_BANK_CEB;
    logic                               MEM_MFCC_CIRBUF_BANK0_WEB;
    logic                               MEM_MFCC_CIRBUF_BANK1_WEB;
    logic                               MEM_MFCC_CIRBUF_BANK2_WEB;
    logic                               MEM_MFCC_CIRBUF_BANK3_WEB;
    logic                               MEM_MFCC_CIRBUF_BANK4_WEB;
    logic                               MEM_MFCC_CIRBUF_BANK5_WEB;
    logic                               MEM_MFCC_CIRBUF_BANK6_WEB;
    logic                               MEM_MFCC_CIRBUF_BANK7_WEB;
    logic [$clog2(8*N_MEL)-1:0]         MEM_MFCC_CIRBUF_BANK_A;
    logic [DATAOUT_WIDTH-1:0]           MEM_MFCC_CIRBUF_WDATA;
    
    // FLUX cricular buffer signals
    logic [N_FRAME-1:0]                 flux_cirbuf_rdata;
    logic                               MEM_FLUX_CIRBUF_BANK_CEB;
    logic                               MEM_FLUX_CIRBUF_BANK_WEB;
    logic [N_FRAME-1:0]                 MEM_FLUX_CIRBUF_BANK_BWEB;
    logic [$clog2(N_MEL)-1:0]           MEM_FLUX_CIRBUF_BANK_A;
    logic [N_FRAME-1:0]                 MEM_FLUX_CIRBUF_BANK_D;
    
    // Feature bank signals
    logic                               MEM_FEBANK_CEB_binarizer;
    logic                               MEM_FEBANK_WEB_binarizer;
    logic [$clog2(2*N_MEL)-1:0]         MEM_FEBANK_A_binarizer;
    logic [N_FRAME-1:0]                 MEM_FEBANK_D_binarizer;
    logic [N_FRAME-1:0]                 MEM_FEBANK_Q;
    
    // sync process
    assign spi_wen_fe_bank_sync  = ~spi_wen_fe_bank_d3 & spi_wen_fe_bank_d2;
    
    always_ff @(posedge LRCLK, negedge MCLK_rst_n) begin
        if (!MCLK_rst_n) begin
            spi_en_inf_sample_d1    <= 0;
            spi_en_inf_sample_sync  <= 0;
            spi_en_fe_sample_d1     <= 0;
            spi_en_fe_sample_sync   <= 0;
        end else begin
            spi_en_inf_sample_d1    <= SPI_EN_INF;
            spi_en_inf_sample_sync  <= spi_en_inf_sample_d1;
            spi_en_fe_sample_d1     <= SPI_EN_FE;
            spi_en_fe_sample_sync   <= spi_en_fe_sample_d1;
        end
    end
    
    always_ff @(posedge sys_clk, negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            spi_wen_fe_bank_d1      <= 0;
            spi_wen_fe_bank_d2      <= 0;
            spi_wen_fe_bank_d3      <= 0;
            spi_en_inf_system_d1    <= 0;
            spi_en_inf_system_sync  <= 0;
            spi_en_fe_system_d1     <= 0;
            spi_en_fe_system_sync   <= 0;
        end else begin
            spi_wen_fe_bank_d1      <= SPI_WEN_FE_BANK;
            spi_wen_fe_bank_d2      <= spi_wen_fe_bank_d1;
            spi_wen_fe_bank_d3      <= spi_wen_fe_bank_d2;
            spi_en_inf_system_d1    <= SPI_EN_INF;
            spi_en_inf_system_sync  <= spi_en_inf_system_d1;
            spi_en_fe_system_d1     <= SPI_EN_FE;
            spi_en_fe_system_sync   <= spi_en_fe_system_d1;
        end
    end
    
    i2s_master #(
        .I2S_DATA_WIDTH             (I2S_DATA_WIDTH             ),
        .Input_INT_BIT_WIDTH        (Input_INT_BIT_WIDTH        ),
        .Input_FRA_BIT_WIDTH        (Input_FRA_BIT_WIDTH        )
        
    ) i2s_master_inst(
        .MCLK                       (MCLK                       ),
        .MCLK_rst_n                 (MCLK_rst_n                 ),
        .ADC_SDATA                  (ADC_SDATA                  ),
        .BCLK                       (BCLK                       ),
        .LRCLK                      (LRCLK                      ),
        
        .audio_data                 (audio_data                 )
    );
    
    pre_emp #(
        .DATA_WIDTH                 (DATAIN_WIDTH               )

    ) pre_emp_inst(
        .clk                        (LRCLK                      ),
        .rst_n                      (MCLK_rst_n                 ),
        .audio_data                 (audio_data                 ),
        .full                       (buf_wfull                  ),
        .spi_en_inf_sample_sync     (spi_en_inf_sample_sync     ),
        .spi_en_fe_sample_sync      (spi_en_fe_sample_sync      ),
        
        .emp_data                   (emp_data                   ),
        .w_req                      (databuf_wreq               )
    );

    data_buf #(
        .DATA_WIDTH                 (DATAIN_WIDTH               ),
        .N_FFT                      (N_FFT                      )
        
    ) data_buf_inst(
        .wclk                       (LRCLK                      ),
        .wrst_n                     (MCLK_rst_n                 ),
        .rclk                       (sys_clk                    ),
        .rrst_n                     (sys_rst_n                  ),
        .w_req                      (databuf_wreq               ),
        .r_req                      (databuf_rreq               ),
        .w_data                     (emp_data                   ),
        .spi_en_inf_sample_sync     (spi_en_inf_sample_sync     ),
        .spi_en_inf_system_sync     (spi_en_inf_system_sync     ),
        
        .r_data                     (buf_data                   ),
        .wfull                      (buf_wfull                  ),
        .almost_rfull               (buf_almost_rfull           ),
        .rempty                     (buf_rempty                 )
    );
    
    fft #(
        .N_FFT                      (N_FFT                      ),
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
        .TW_BIT_WIDTH               (TW_BIT_WIDTH               )
        
    ) fft_inst(
        .clk                        (sys_clk                    ),
        .rst_n                      (sys_rst_n                  ),
        .buf_almost_rfull           (buf_almost_rfull           ),
        .buf_rempty                 (buf_rempty                 ),
        .Re_in                      (buf_data                   ),
        .spi_en_inf_system_sync     (spi_en_inf_system_sync     ),
        
        .r_req                      (databuf_rreq               ),
        .valid_out                  (fft_data_valid             ),
        .Re_out                     (Re_result                  ),
        .Im_out                     (Im_result                  )
    );
    
    fft_swap #(
        .N_FFT                      (N_FFT                      ),
        .STAGE8_INT_BIT_WIDTH       (STAGE8_INT_BIT_WIDTH       ),
        .STAGE8_FRA_BIT_WIDTH       (STAGE8_FRA_BIT_WIDTH       )
        
    ) fft_swap_inst(
        .clk                        (sys_clk                    ),
        .rst_n                      (sys_rst_n                  ),
        .valid_in                   (fft_data_valid             ),
        .Re_result                  (Re_result                  ),
        .Im_result                  (Im_result                  ),
        .spi_en_inf_system_sync     (spi_en_inf_system_sync     ),
        
        .data_out_valid             (fft_abs_valid              ),
        .data_out                   (fft_abs_data               )
    );
    
    mel_filter #(
        .N_FFT                      (N_FFT                      ),
        .N_MEL                      (N_MEL                      ),
        .DATAOUT_WIDTH              (DATAOUT_WIDTH              ),
        .STAGE8_INT_BIT_WIDTH       (STAGE8_INT_BIT_WIDTH       ),
        .STAGE8_FRA_BIT_WIDTH       (STAGE8_FRA_BIT_WIDTH       )
        
    ) mel_filter_inst(
        .clk                        (sys_clk                    ),
        .rst_n                      (sys_rst_n                  ),
        .data_in_valid              (fft_abs_valid              ),
        .data_in                    (fft_abs_data               ),
        .spi_en_inf_system_sync     (spi_en_inf_system_sync     ),
                                     
        .even_mel_valid             (even_mel_valid             ),
        .odd_mel_valid              (odd_mel_valid              ),
        .mel_value                  (mel_value                  )
    );
    
    ping_pong_buffer #(
        .N_MEL                      (N_MEL                      ),
        .BIT_WIDTH                  (DATAOUT_WIDTH              )
        
    ) ping_pong_buffer_inst(
        .clk                        (sys_clk                    ),
        .rst_n                      (sys_rst_n                  ),
        .ready_in                   (ready_bin2buffer           ),
        .even_mel_valid             (even_mel_valid             ),
        .odd_mel_valid              (odd_mel_valid              ),
        .mel_value                  (mel_value                  ),
        .spi_en_inf_system_sync     (spi_en_inf_system_sync     ),
        
        .valid_out                  (mfcc_flux_valid_out        ),
        .mfcc_data_out              (mfcc_data                  ),
        .flux_data_out              (flux_data                  )
    );
    
    binarizer #(
        .N_MEL                      (N_MEL                      ),
        .N_FRAME                    (N_FRAME                    ),
        .BIT_WIDTH                  (DATAOUT_WIDTH              )
        
    ) binarizer_inst(
        .clk                        (sys_clk                    ),
        .rst_n                      (sys_rst_n                  ),
        .valid_in                   (mfcc_flux_valid_out        ),   
        .mfcc_data_in               (mfcc_data                  ),
        .flux_data_in               (flux_data                  ),
        .ready_out                  (ready_bin2buffer           ),
        
        .spi_en_fe_system_sync      (spi_en_fe_system_sync      ),
        .spi_en_inf_system_sync     (spi_en_inf_system_sync     ),
        .SPI_FLUX_TH                (SPI_FLUX_TH                ),
        
        .mfcc_cirbuf_rdata0         (mfcc_cirbuf_rdata0         ),
        .mfcc_cirbuf_rdata1         (mfcc_cirbuf_rdata1         ),
        .mfcc_cirbuf_rdata2         (mfcc_cirbuf_rdata2         ),
        .mfcc_cirbuf_rdata3         (mfcc_cirbuf_rdata3         ),
        .mfcc_cirbuf_rdata4         (mfcc_cirbuf_rdata4         ),
        .mfcc_cirbuf_rdata5         (mfcc_cirbuf_rdata5         ),
        .mfcc_cirbuf_rdata6         (mfcc_cirbuf_rdata6         ),
        .mfcc_cirbuf_rdata7         (mfcc_cirbuf_rdata7         ),
        .MEM_MFCC_CIRBUF_BANK_CEB   (MEM_MFCC_CIRBUF_BANK_CEB   ),
        .MEM_MFCC_CIRBUF_BANK0_WEB  (MEM_MFCC_CIRBUF_BANK0_WEB  ),
        .MEM_MFCC_CIRBUF_BANK1_WEB  (MEM_MFCC_CIRBUF_BANK1_WEB  ),
        .MEM_MFCC_CIRBUF_BANK2_WEB  (MEM_MFCC_CIRBUF_BANK2_WEB  ),
        .MEM_MFCC_CIRBUF_BANK3_WEB  (MEM_MFCC_CIRBUF_BANK3_WEB  ),
        .MEM_MFCC_CIRBUF_BANK4_WEB  (MEM_MFCC_CIRBUF_BANK4_WEB  ),
        .MEM_MFCC_CIRBUF_BANK5_WEB  (MEM_MFCC_CIRBUF_BANK5_WEB  ),
        .MEM_MFCC_CIRBUF_BANK6_WEB  (MEM_MFCC_CIRBUF_BANK6_WEB  ),
        .MEM_MFCC_CIRBUF_BANK7_WEB  (MEM_MFCC_CIRBUF_BANK7_WEB  ),
        .MEM_MFCC_CIRBUF_BANK_A     (MEM_MFCC_CIRBUF_BANK_A     ),
        .MEM_MFCC_CIRBUF_WDATA      (MEM_MFCC_CIRBUF_WDATA      ),
        
        .flux_cirbuf_rdata          (flux_cirbuf_rdata          ),
        .MEM_FLUX_CIRBUF_BANK_CEB   (MEM_FLUX_CIRBUF_BANK_CEB   ),
        .MEM_FLUX_CIRBUF_BANK_WEB   (MEM_FLUX_CIRBUF_BANK_WEB   ),
        .MEM_FLUX_CIRBUF_BANK_BWEB  (MEM_FLUX_CIRBUF_BANK_BWEB  ),
        .MEM_FLUX_CIRBUF_BANK_A     (MEM_FLUX_CIRBUF_BANK_A     ),
        .MEM_FLUX_CIRBUF_BANK_D     (MEM_FLUX_CIRBUF_BANK_D     ),
        
        .MEM_FEBANK_CEB_binarizer   (MEM_FEBANK_CEB_binarizer   ),
        .MEM_FEBANK_WEB_binarizer   (MEM_FEBANK_WEB_binarizer   ),
        .MEM_FEBANK_A_binarizer     (MEM_FEBANK_A_binarizer     ),
        .MEM_FEBANK_D_binarizer     (MEM_FEBANK_D_binarizer     ),
        
        .fe_complete                (fe_complete                )
    );
    
    feature_module #(
        .N_MEL                      (N_MEL                      ),
        .N_FRAME                    (N_FRAME                    ),
        .BIT_WIDTH                  (DATAOUT_WIDTH              )
        
    ) feature_module_inst(
        .clk                        (sys_clk                    ),
        .rst_n                      (sys_rst_n                  ),
        
        .spi_wen_fe_bank_sync       (spi_wen_fe_bank_sync       ),
        .SPI_ADDR                   (SPI_ADDR                   ),
        .SPI_DATA                   (SPI_DATA                   ),
        
        .feature_bank_ren           (feature_bank_ren           ),
        .feature_rptr               (feature_rptr               ),
        .feature_bank_rdata         (feature_bank_rdata         ),
        
        .MEM_MFCC_CIRBUF_BANK_CEB   (MEM_MFCC_CIRBUF_BANK_CEB   ),
        .MEM_MFCC_CIRBUF_BANK0_WEB  (MEM_MFCC_CIRBUF_BANK0_WEB  ),
        .MEM_MFCC_CIRBUF_BANK1_WEB  (MEM_MFCC_CIRBUF_BANK1_WEB  ),
        .MEM_MFCC_CIRBUF_BANK2_WEB  (MEM_MFCC_CIRBUF_BANK2_WEB  ),
        .MEM_MFCC_CIRBUF_BANK3_WEB  (MEM_MFCC_CIRBUF_BANK3_WEB  ),
        .MEM_MFCC_CIRBUF_BANK4_WEB  (MEM_MFCC_CIRBUF_BANK4_WEB  ),
        .MEM_MFCC_CIRBUF_BANK5_WEB  (MEM_MFCC_CIRBUF_BANK5_WEB  ),
        .MEM_MFCC_CIRBUF_BANK6_WEB  (MEM_MFCC_CIRBUF_BANK6_WEB  ),
        .MEM_MFCC_CIRBUF_BANK7_WEB  (MEM_MFCC_CIRBUF_BANK7_WEB  ),
        .MEM_MFCC_CIRBUF_BANK_A     (MEM_MFCC_CIRBUF_BANK_A     ),
        .MEM_MFCC_CIRBUF_WDATA      (MEM_MFCC_CIRBUF_WDATA      ),
        .mfcc_cirbuf_rdata0         (mfcc_cirbuf_rdata0         ),
        .mfcc_cirbuf_rdata1         (mfcc_cirbuf_rdata1         ),
        .mfcc_cirbuf_rdata2         (mfcc_cirbuf_rdata2         ),
        .mfcc_cirbuf_rdata3         (mfcc_cirbuf_rdata3         ),
        .mfcc_cirbuf_rdata4         (mfcc_cirbuf_rdata4         ),
        .mfcc_cirbuf_rdata5         (mfcc_cirbuf_rdata5         ),
        .mfcc_cirbuf_rdata6         (mfcc_cirbuf_rdata6         ),
        .mfcc_cirbuf_rdata7         (mfcc_cirbuf_rdata7         ),
        
        .MEM_FLUX_CIRBUF_BANK_CEB   (MEM_FLUX_CIRBUF_BANK_CEB   ),
        .MEM_FLUX_CIRBUF_BANK_WEB   (MEM_FLUX_CIRBUF_BANK_WEB   ),
        .MEM_FLUX_CIRBUF_BANK_BWEB  (MEM_FLUX_CIRBUF_BANK_BWEB  ),
        .MEM_FLUX_CIRBUF_BANK_A     (MEM_FLUX_CIRBUF_BANK_A     ),
        .MEM_FLUX_CIRBUF_BANK_D     (MEM_FLUX_CIRBUF_BANK_D     ),
        .flux_cirbuf_rdata          (flux_cirbuf_rdata          ),
        
        .MEM_FEBANK_CEB_binarizer   (MEM_FEBANK_CEB_binarizer   ),
        .MEM_FEBANK_WEB_binarizer   (MEM_FEBANK_WEB_binarizer   ),
        .MEM_FEBANK_A_binarizer     (MEM_FEBANK_A_binarizer     ),
        .MEM_FEBANK_D_binarizer     (MEM_FEBANK_D_binarizer     )
    );
    
endmodule
