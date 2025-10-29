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
// Module: "feature_module.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: Feature extractor feature bank module.
//
//==============================================================================

module feature_module #(
    parameter N_MEL                     = 32,
    parameter N_FRAME                   = 64,
    parameter BIT_WIDTH                 = 16
)(
    input logic                         clk,
    input logic                         rst_n,
    
    // spi slave signals ------------------------------------------------------
    input logic                         spi_wen_fe_bank_sync,
    input logic [11:0]                  SPI_ADDR,
    input logic [31:0]                  SPI_DATA,
    
    // Accelerator signals ----------------------------------------------------
    input logic                         feature_bank_ren,
    input logic [$clog2(2*N_MEL)-1:0]   feature_rptr,
    output logic [N_FRAME-1:0]          feature_bank_rdata,
    
    // MFCC cricular buffer signals -------------------------------------------
    input logic                         MEM_MFCC_CIRBUF_BANK_CEB,
    input logic                         MEM_MFCC_CIRBUF_BANK0_WEB,
    input logic                         MEM_MFCC_CIRBUF_BANK1_WEB,
    input logic                         MEM_MFCC_CIRBUF_BANK2_WEB,
    input logic                         MEM_MFCC_CIRBUF_BANK3_WEB,
    input logic                         MEM_MFCC_CIRBUF_BANK4_WEB,
    input logic                         MEM_MFCC_CIRBUF_BANK5_WEB,
    input logic                         MEM_MFCC_CIRBUF_BANK6_WEB,
    input logic                         MEM_MFCC_CIRBUF_BANK7_WEB,
    input logic [$clog2(8*N_MEL)-1:0]   MEM_MFCC_CIRBUF_BANK_A,
    input logic [BIT_WIDTH-1:0]         MEM_MFCC_CIRBUF_WDATA,
    output logic [BIT_WIDTH-1:0]        mfcc_cirbuf_rdata0,
    output logic [BIT_WIDTH-1:0]        mfcc_cirbuf_rdata1,
    output logic [BIT_WIDTH-1:0]        mfcc_cirbuf_rdata2,
    output logic [BIT_WIDTH-1:0]        mfcc_cirbuf_rdata3,
    output logic [BIT_WIDTH-1:0]        mfcc_cirbuf_rdata4,
    output logic [BIT_WIDTH-1:0]        mfcc_cirbuf_rdata5,
    output logic [BIT_WIDTH-1:0]        mfcc_cirbuf_rdata6,
    output logic [BIT_WIDTH-1:0]        mfcc_cirbuf_rdata7,
    
    // FLUX cricular buffer signals -------------------------------------------
    input logic                         MEM_FLUX_CIRBUF_BANK_CEB,
    input logic                         MEM_FLUX_CIRBUF_BANK_WEB,
    input logic [N_FRAME-1:0]           MEM_FLUX_CIRBUF_BANK_BWEB,
    input logic [$clog2(N_MEL)-1:0]     MEM_FLUX_CIRBUF_BANK_A,
    input logic [N_FRAME-1:0]           MEM_FLUX_CIRBUF_BANK_D,
    output logic [N_FRAME-1:0]          flux_cirbuf_rdata,
    
    // Feature bank signals ---------------------------------------------------
    input logic                         MEM_FEBANK_CEB_binarizer,
    input logic                         MEM_FEBANK_WEB_binarizer,
    input logic [$clog2(2*N_MEL)-1:0]   MEM_FEBANK_A_binarizer,
    input logic [N_FRAME-1:0]           MEM_FEBANK_D_binarizer
    
);

    logic [BIT_WIDTH-1:0]       mfcc_circular_buffer_bank0    [0:8*N_MEL-1];    // 8 ram (w16d256)
    logic [BIT_WIDTH-1:0]       mfcc_circular_buffer_bank1    [0:8*N_MEL-1];
    logic [BIT_WIDTH-1:0]       mfcc_circular_buffer_bank2    [0:8*N_MEL-1];
    logic [BIT_WIDTH-1:0]       mfcc_circular_buffer_bank3    [0:8*N_MEL-1];
    logic [BIT_WIDTH-1:0]       mfcc_circular_buffer_bank4    [0:8*N_MEL-1];
    logic [BIT_WIDTH-1:0]       mfcc_circular_buffer_bank5    [0:8*N_MEL-1];
    logic [BIT_WIDTH-1:0]       mfcc_circular_buffer_bank6    [0:8*N_MEL-1];
    logic [BIT_WIDTH-1:0]       mfcc_circular_buffer_bank7    [0:8*N_MEL-1];
    
    logic [N_FRAME-1:0]         flux_circular_buffer    [0:N_MEL-1];            // ram (w64d32)
    logic [N_FRAME-1:0]         feature_bank            [0:2*N_MEL-1];          // ram (w64d64)
    
    logic                       MEM_FEBANK_CEB;
    logic [7:0]                 MEM_FEBANK_WEB;
    logic [$clog2(2*N_MEL)-1:0] MEM_FEBANK_A;
    logic [N_FRAME-1:0]         MEM_FEBANK_D;
    logic [N_FRAME-1:0]         MEM_FEBANK_Q;
    
    assign feature_bank_rdata = MEM_FEBANK_Q;
    
    assign MEM_FEBANK_CEB   = MEM_FEBANK_CEB_binarizer && !spi_wen_fe_bank_sync && !feature_bank_ren;
    assign MEM_FEBANK_A     = (spi_wen_fe_bank_sync)? SPI_ADDR[$clog2(2*N_MEL):1] : 
                              (feature_bank_ren)? feature_rptr : MEM_FEBANK_A_binarizer;
    
    always_comb begin
        MEM_FEBANK_D = 0;
        if (spi_wen_fe_bank_sync) begin
            case(SPI_ADDR[0])
                0: MEM_FEBANK_D = SPI_DATA;
                1: MEM_FEBANK_D = SPI_DATA << 32;
            endcase
        end else if (!MEM_FEBANK_WEB_binarizer) begin
            MEM_FEBANK_D = MEM_FEBANK_D_binarizer;
        end
    end
    
    always_comb begin
        MEM_FEBANK_WEB = '1;
        if (spi_wen_fe_bank_sync) begin
            case(SPI_ADDR[0])
                0: MEM_FEBANK_WEB = 8'b1111_0000;
                1: MEM_FEBANK_WEB = 8'b0000_1111;
            endcase
        end else if (!MEM_FEBANK_WEB_binarizer) begin
            MEM_FEBANK_WEB = 8'b0000_0000;
        end
    end
    
    // MFCC circular buffer
    always_ff @(posedge clk) begin
        if (!MEM_MFCC_CIRBUF_BANK_CEB) begin
            if (!MEM_MFCC_CIRBUF_BANK0_WEB)
                mfcc_circular_buffer_bank0[MEM_MFCC_CIRBUF_BANK_A] <= MEM_MFCC_CIRBUF_WDATA;
            else
                mfcc_cirbuf_rdata0 <= mfcc_circular_buffer_bank0[MEM_MFCC_CIRBUF_BANK_A];
        end
    end
    
    always_ff @(posedge clk) begin
        if (!MEM_MFCC_CIRBUF_BANK_CEB) begin
            if (!MEM_MFCC_CIRBUF_BANK1_WEB)
                mfcc_circular_buffer_bank1[MEM_MFCC_CIRBUF_BANK_A] <= MEM_MFCC_CIRBUF_WDATA;
            else
                mfcc_cirbuf_rdata1 <= mfcc_circular_buffer_bank1[MEM_MFCC_CIRBUF_BANK_A];
        end
    end
    
    always_ff @(posedge clk) begin
        if (!MEM_MFCC_CIRBUF_BANK_CEB) begin
            if (!MEM_MFCC_CIRBUF_BANK2_WEB)
                mfcc_circular_buffer_bank2[MEM_MFCC_CIRBUF_BANK_A] <= MEM_MFCC_CIRBUF_WDATA;
            else
                mfcc_cirbuf_rdata2 <= mfcc_circular_buffer_bank2[MEM_MFCC_CIRBUF_BANK_A];
        end
    end
    
    always_ff @(posedge clk) begin
        if (!MEM_MFCC_CIRBUF_BANK_CEB) begin
            if (!MEM_MFCC_CIRBUF_BANK3_WEB)
                mfcc_circular_buffer_bank3[MEM_MFCC_CIRBUF_BANK_A] <= MEM_MFCC_CIRBUF_WDATA;
            else
                mfcc_cirbuf_rdata3 <= mfcc_circular_buffer_bank3[MEM_MFCC_CIRBUF_BANK_A];
        end
    end
    
    always_ff @(posedge clk) begin
        if (!MEM_MFCC_CIRBUF_BANK_CEB) begin
            if (!MEM_MFCC_CIRBUF_BANK4_WEB)
                mfcc_circular_buffer_bank4[MEM_MFCC_CIRBUF_BANK_A] <= MEM_MFCC_CIRBUF_WDATA;
            else
                mfcc_cirbuf_rdata4 <= mfcc_circular_buffer_bank4[MEM_MFCC_CIRBUF_BANK_A];
        end
    end
    
    always_ff @(posedge clk) begin
        if (!MEM_MFCC_CIRBUF_BANK_CEB) begin
            if (!MEM_MFCC_CIRBUF_BANK5_WEB)
                mfcc_circular_buffer_bank5[MEM_MFCC_CIRBUF_BANK_A] <= MEM_MFCC_CIRBUF_WDATA;
            else
                mfcc_cirbuf_rdata5 <= mfcc_circular_buffer_bank5[MEM_MFCC_CIRBUF_BANK_A];
        end
    end
    
    always_ff @(posedge clk) begin
        if (!MEM_MFCC_CIRBUF_BANK_CEB) begin
            if (!MEM_MFCC_CIRBUF_BANK6_WEB)
                mfcc_circular_buffer_bank6[MEM_MFCC_CIRBUF_BANK_A] <= MEM_MFCC_CIRBUF_WDATA;
            else
                mfcc_cirbuf_rdata6 <= mfcc_circular_buffer_bank6[MEM_MFCC_CIRBUF_BANK_A];
        end
    end
    
    always_ff @(posedge clk) begin
        if (!MEM_MFCC_CIRBUF_BANK_CEB) begin
            if (!MEM_MFCC_CIRBUF_BANK7_WEB)
                mfcc_circular_buffer_bank7[MEM_MFCC_CIRBUF_BANK_A] <= MEM_MFCC_CIRBUF_WDATA;
            else
                mfcc_cirbuf_rdata7 <= mfcc_circular_buffer_bank7[MEM_MFCC_CIRBUF_BANK_A];
        end
    end
    
    // FLUX circular buffer
    always_ff @(posedge clk) begin
        if (!MEM_FLUX_CIRBUF_BANK_CEB) begin
            if (!MEM_FLUX_CIRBUF_BANK_WEB)
                flux_circular_buffer[MEM_FLUX_CIRBUF_BANK_A] <= MEM_FLUX_CIRBUF_BANK_D;
            else
                flux_cirbuf_rdata <= flux_circular_buffer[MEM_FLUX_CIRBUF_BANK_A];
        end
    end
    
    
    //-------------------------------------------------------------------------
    // Feature bank
    //-------------------------------------------------------------------------
    for (genvar i = 0; i < 8; i++) begin
        always_ff @(posedge clk) begin
            if (!MEM_FEBANK_CEB) begin
                if (!MEM_FEBANK_WEB[i])
                    feature_bank[MEM_FEBANK_A][i * 8 +: 8] <= MEM_FEBANK_D[i * 8 +: 8];
            end
        end
    end
    
    always_ff @(posedge clk) begin
        if (!MEM_FEBANK_CEB) begin
            if (MEM_FEBANK_WEB)
                MEM_FEBANK_Q <= feature_bank[MEM_FEBANK_A];
        end
    end
    
endmodule

