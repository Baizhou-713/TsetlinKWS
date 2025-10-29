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
// Module: "fft.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: FFT top module.
//
//==============================================================================

module fft #(
    parameter N_FFT                 = 256,
    
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
    parameter TW_BIT_WIDTH          = 8
    
)(
    input logic                             clk,
    input logic                             rst_n,
    
    // data buffer signals ----------------------------------------------------
    input logic                             buf_almost_rfull,
    input logic                             buf_rempty,
    input logic signed  [   Input_INT_BIT_WIDTH + Input_FRA_BIT_WIDTH - 1:0]    Re_in,
    output logic                            r_req,
    
    // spi_slave Configuration registers --------------------------------------
    input logic                             spi_en_inf_system_sync,
    
    // FFT swap signals -------------------------------------------------------
    output logic                            valid_out,
    output logic signed [   STAGE8_INT_BIT_WIDTH + STAGE8_FRA_BIT_WIDTH - 1:0]  Re_out,
    output logic signed [   STAGE8_INT_BIT_WIDTH + STAGE8_FRA_BIT_WIDTH - 1:0]  Im_out
);
    
    logic valid_in;
    logic valid_1_to_2;
    logic valid_2_to_3;
    logic valid_3_to_4;
    logic valid_4_to_5;
    logic valid_5_to_6;
    logic valid_6_to_7;
    logic valid_7_to_8;
    logic FSM_pre_store_en      [$clog2(N_FFT)];
    logic FSM_calc_en           [$clog2(N_FFT)];
    logic FSM_data_in_buf_ren   [$clog2(N_FFT)];
    
    logic signed [STAGE1_INT_BIT_WIDTH + STAGE1_FRA_BIT_WIDTH - 1:0]    Re_result_1_to_2, Im_result_1_to_2;
    logic signed [STAGE2_INT_BIT_WIDTH + STAGE2_FRA_BIT_WIDTH - 1:0]    Re_result_2_to_3, Im_result_2_to_3;
    logic signed [STAGE3_INT_BIT_WIDTH + STAGE3_FRA_BIT_WIDTH - 1:0]    Re_result_3_to_4, Im_result_3_to_4;
    logic signed [STAGE4_INT_BIT_WIDTH + STAGE4_FRA_BIT_WIDTH - 1:0]    Re_result_4_to_5, Im_result_4_to_5;
    logic signed [STAGE5_INT_BIT_WIDTH + STAGE5_FRA_BIT_WIDTH - 1:0]    Re_result_5_to_6, Im_result_5_to_6;
    logic signed [STAGE6_INT_BIT_WIDTH + STAGE6_FRA_BIT_WIDTH - 1:0]    Re_result_6_to_7, Im_result_6_to_7;
    logic signed [STAGE7_INT_BIT_WIDTH + STAGE7_FRA_BIT_WIDTH - 1:0]    Re_result_7_to_8, Im_result_7_to_8;
    
    fft_controller #(
        .N_FFT                  (N_FFT                  )
        
    ) fft_controller_inst(  
        .clk                    (clk                    ),
        .rst_n                  (rst_n                  ),
        
        .buf_almost_rfull       (buf_almost_rfull       ),
        .buf_rempty             (buf_rempty             ),
        .r_req                  (r_req                  ),
        
        .spi_en_inf_system_sync (spi_en_inf_system_sync ),
        
        .valid                  (valid_in               ),
        .FSM_pre_store_en       (FSM_pre_store_en       ),
        .FSM_calc_en            (FSM_calc_en            ),
        .FSM_data_in_buf_ren    (FSM_data_in_buf_ren    )
    );

    fft_stage #(
        .Last_INT_BIT_WIDTH     (Input_INT_BIT_WIDTH    ),
        .Last_FRA_BIT_WIDTH     (Input_FRA_BIT_WIDTH    ),
        .INT_BIT_WIDTH          (STAGE1_INT_BIT_WIDTH   ),
        .FRA_BIT_WIDTH          (STAGE1_FRA_BIT_WIDTH   ),
        .TW_BIT_WIDTH           (TW_BIT_WIDTH           ),
        .N_FFT                  (N_FFT                  ),
        .NO_STAGE               (0                      ),
        .USE_RAM                (1                      ),
        .USE_ROM                (1                      )
        
    ) fft_stage_1(
        .clk                    (clk                    ),
        .rst_n                  (rst_n                  ),
        .valid_in               (valid_in               ),
        .FSM_pre_store_en       (FSM_pre_store_en[0]    ),
        .FSM_calc_en            (FSM_calc_en[0]         ),
        .FSM_data_in_buf_ren    (FSM_data_in_buf_ren[0] ),
        .spi_en_inf_system_sync (spi_en_inf_system_sync ),
        
        .Re_in                  (Re_in                  ),
        .Im_in                  ('0                     ),
        .valid_out              (valid_1_to_2           ),
        .Re_out                 (Re_result_1_to_2       ),
        .Im_out                 (Im_result_1_to_2       )
    );

    
    fft_stage #(
        .Last_INT_BIT_WIDTH     (STAGE1_INT_BIT_WIDTH   ),
        .Last_FRA_BIT_WIDTH     (STAGE1_FRA_BIT_WIDTH   ),
        .INT_BIT_WIDTH          (STAGE2_INT_BIT_WIDTH   ),
        .FRA_BIT_WIDTH          (STAGE2_FRA_BIT_WIDTH   ),
        .TW_BIT_WIDTH           (TW_BIT_WIDTH           ),
        .N_FFT                  (N_FFT                  ),
        .NO_STAGE               (1                      ),
        .USE_RAM                (1                      ),
        .USE_ROM                (1                      )
        
    ) fft_stage_2(
        .clk                    (clk                    ),
        .rst_n                  (rst_n                  ),
        .valid_in               (valid_1_to_2           ),
        .FSM_pre_store_en       (FSM_pre_store_en[1]    ),
        .FSM_calc_en            (FSM_calc_en[1]         ),
        .FSM_data_in_buf_ren    (FSM_data_in_buf_ren[1] ),
        .spi_en_inf_system_sync (spi_en_inf_system_sync ),
        
        .Re_in                  (Re_result_1_to_2       ),
        .Im_in                  (Im_result_1_to_2       ),
        .valid_out              (valid_2_to_3           ),
        .Re_out                 (Re_result_2_to_3       ),
        .Im_out                 (Im_result_2_to_3       )
    );  
    

    fft_stage #(
        .Last_INT_BIT_WIDTH     (STAGE2_INT_BIT_WIDTH   ),
        .Last_FRA_BIT_WIDTH     (STAGE2_FRA_BIT_WIDTH   ),
        .INT_BIT_WIDTH          (STAGE3_INT_BIT_WIDTH   ),
        .FRA_BIT_WIDTH          (STAGE3_FRA_BIT_WIDTH   ),
        .TW_BIT_WIDTH           (TW_BIT_WIDTH           ),
        .N_FFT                  (N_FFT                  ),
        .NO_STAGE               (2                      ),
        .USE_RAM                (1                      ),
        .USE_ROM                (1                      )
        
    ) fft_stage_3(
        .clk                    (clk                    ),
        .rst_n                  (rst_n                  ),
        .valid_in               (valid_2_to_3           ),
        .FSM_pre_store_en       (FSM_pre_store_en[2]    ),
        .FSM_calc_en            (FSM_calc_en[2]         ),
        .FSM_data_in_buf_ren    (FSM_data_in_buf_ren[2] ),
        .spi_en_inf_system_sync (spi_en_inf_system_sync ),
        
        .Re_in                  (Re_result_2_to_3       ),
        .Im_in                  (Im_result_2_to_3       ),
        .valid_out              (valid_3_to_4           ),
        .Re_out                 (Re_result_3_to_4       ),
        .Im_out                 (Im_result_3_to_4       )
    );  
    

    fft_stage #(
        .Last_INT_BIT_WIDTH     (STAGE3_INT_BIT_WIDTH   ),
        .Last_FRA_BIT_WIDTH     (STAGE3_FRA_BIT_WIDTH   ),
        .INT_BIT_WIDTH          (STAGE4_INT_BIT_WIDTH   ),
        .FRA_BIT_WIDTH          (STAGE4_FRA_BIT_WIDTH   ),
        .TW_BIT_WIDTH           (TW_BIT_WIDTH           ),
        .N_FFT                  (N_FFT                  ),
        .NO_STAGE               (3                      ),
        .USE_RAM                (0                      ),
        .USE_ROM                (0                      )
        
    ) fft_stage_4(
        .clk                    (clk                    ),
        .rst_n                  (rst_n                  ),
        .valid_in               (valid_3_to_4           ),
        .FSM_pre_store_en       (FSM_pre_store_en[3]    ),
        .FSM_calc_en            (FSM_calc_en[3]         ),
        .FSM_data_in_buf_ren    (FSM_data_in_buf_ren[3] ),
        .spi_en_inf_system_sync (spi_en_inf_system_sync ),
        
        .Re_in                  (Re_result_3_to_4       ),
        .Im_in                  (Im_result_3_to_4       ),
        .valid_out              (valid_4_to_5           ),
        .Re_out                 (Re_result_4_to_5       ),
        .Im_out                 (Im_result_4_to_5       )
    );  
    

    fft_stage #(
        .Last_INT_BIT_WIDTH     (STAGE4_INT_BIT_WIDTH   ),
        .Last_FRA_BIT_WIDTH     (STAGE4_FRA_BIT_WIDTH   ),
        .INT_BIT_WIDTH          (STAGE5_INT_BIT_WIDTH   ),
        .FRA_BIT_WIDTH          (STAGE5_FRA_BIT_WIDTH   ),
        .TW_BIT_WIDTH           (TW_BIT_WIDTH           ),
        .N_FFT                  (N_FFT                  ),
        .NO_STAGE               (4                      ),
        .USE_RAM                (0                      ),
        .USE_ROM                (0                      )
        
    ) fft_stage_5(
        .clk                    (clk                    ),
        .rst_n                  (rst_n                  ),
        .valid_in               (valid_4_to_5           ),
        .FSM_pre_store_en       (FSM_pre_store_en[4]    ),
        .FSM_calc_en            (FSM_calc_en[4]         ),
        .FSM_data_in_buf_ren    (FSM_data_in_buf_ren[4] ),
        .spi_en_inf_system_sync (spi_en_inf_system_sync ),
        
        .Re_in                  (Re_result_4_to_5       ),
        .Im_in                  (Im_result_4_to_5       ),
        .valid_out              (valid_5_to_6           ),
        .Re_out                 (Re_result_5_to_6       ),
        .Im_out                 (Im_result_5_to_6       )
    );  
    

    fft_stage #(
        .Last_INT_BIT_WIDTH     (STAGE5_INT_BIT_WIDTH   ),
        .Last_FRA_BIT_WIDTH     (STAGE5_FRA_BIT_WIDTH   ),
        .INT_BIT_WIDTH          (STAGE6_INT_BIT_WIDTH   ),
        .FRA_BIT_WIDTH          (STAGE6_FRA_BIT_WIDTH   ),
        .TW_BIT_WIDTH           (TW_BIT_WIDTH           ),
        .N_FFT                  (N_FFT                  ),
        .NO_STAGE               (5                      ),
        .USE_RAM                (0                      ),
        .USE_ROM                (0                      )
        
    ) fft_stage_6(
        .clk                    (clk                    ),
        .rst_n                  (rst_n                  ),
        .valid_in               (valid_5_to_6           ),
        .FSM_pre_store_en       (FSM_pre_store_en[5]    ),
        .FSM_calc_en            (FSM_calc_en[5]         ),
        .FSM_data_in_buf_ren    (FSM_data_in_buf_ren[5] ),
        .spi_en_inf_system_sync (spi_en_inf_system_sync ),
        
        .Re_in                  (Re_result_5_to_6       ),
        .Im_in                  (Im_result_5_to_6       ),
        .valid_out              (valid_6_to_7           ),
        .Re_out                 (Re_result_6_to_7       ),
        .Im_out                 (Im_result_6_to_7       )
    );  
    

    fft_stage #(
        .Last_INT_BIT_WIDTH     (STAGE6_INT_BIT_WIDTH   ),
        .Last_FRA_BIT_WIDTH     (STAGE6_FRA_BIT_WIDTH   ),
        .INT_BIT_WIDTH          (STAGE7_INT_BIT_WIDTH   ),
        .FRA_BIT_WIDTH          (STAGE7_FRA_BIT_WIDTH   ),
        .TW_BIT_WIDTH           (TW_BIT_WIDTH           ),
        .N_FFT                  (N_FFT                  ),
        .NO_STAGE               (6                      ),
        .USE_RAM                (0                      ),
        .USE_ROM                (0                      )
        
    ) fft_stage_7(
        .clk                    (clk                    ),
        .rst_n                  (rst_n                  ),
        .valid_in               (valid_6_to_7           ),
        .FSM_pre_store_en       (FSM_pre_store_en[6]    ),
        .FSM_calc_en            (FSM_calc_en[6]         ),
        .FSM_data_in_buf_ren    (FSM_data_in_buf_ren[6] ),
        .spi_en_inf_system_sync (spi_en_inf_system_sync ),
        
        .Re_in                  (Re_result_6_to_7       ),
        .Im_in                  (Im_result_6_to_7       ),
        .valid_out              (valid_7_to_8           ),
        .Re_out                 (Re_result_7_to_8       ),
        .Im_out                 (Im_result_7_to_8       )
    );  
    

    fft_stage #(
        .Last_INT_BIT_WIDTH     (STAGE7_INT_BIT_WIDTH   ),
        .Last_FRA_BIT_WIDTH     (STAGE7_FRA_BIT_WIDTH   ),
        .INT_BIT_WIDTH          (STAGE8_INT_BIT_WIDTH   ),
        .FRA_BIT_WIDTH          (STAGE8_FRA_BIT_WIDTH   ),
        .TW_BIT_WIDTH           (TW_BIT_WIDTH           ),
        .N_FFT                  (N_FFT                  ),
        .NO_STAGE               (7                      ),
        .USE_RAM                (0                      ),
        .USE_ROM                (0                      )
        
    ) fft_stage_8(
        .clk                    (clk                    ),
        .rst_n                  (rst_n                  ),
        .valid_in               (valid_7_to_8           ),
        .FSM_pre_store_en       (FSM_pre_store_en[7]    ),
        .FSM_calc_en            (FSM_calc_en[7]         ),
        .FSM_data_in_buf_ren    (FSM_data_in_buf_ren[7] ),
        .spi_en_inf_system_sync (spi_en_inf_system_sync ),
        
        .Re_in                  (Re_result_7_to_8       ),
        .Im_in                  (Im_result_7_to_8       ),
        .valid_out              (valid_out              ),
        .Re_out                 (Re_out                 ),
        .Im_out                 (Im_out                 )
    );
    
endmodule
