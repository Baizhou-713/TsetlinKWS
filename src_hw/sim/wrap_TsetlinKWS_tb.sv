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
// Module: "wrap_TsetlinKWS_tb.sv"
//
// Author: Baizhou Lin, University of Southampton
// 
// Desc: wrap_TsetlinKWS module testbench.
//
//==============================================================================

module wrap_TsetlinKWS_tb();

timeunit 1ns;
timeprecision 1ps;

    parameter N_FFT                 = 256   ;
    parameter N_FRAME               = 64    ;
    parameter N_MEL                 = 32    ;
    parameter N_PE_COL              = 5     ;
    parameter N_ELEMENT             = 4     ;
    parameter NUMBER_OF_PATCH       = 58    ;
    parameter DEPTH_BLOCK_BANK      = 2048  ;
    parameter DEPTH_ROW_BANK        = 2048  ;
    parameter DEPTH_CCL_BANK        = 4096  ;
    parameter DEPTH_WEIGHT_BANK     = 2048  ;

    parameter I2S_DATA_WIDTH        = 24    ;
    parameter Input_INT_BIT_WIDTH   = 12    ;
    parameter Input_FRA_BIT_WIDTH   = 0     ;
    parameter STAGE1_INT_BIT_WIDTH  = 12    ;
    parameter STAGE1_FRA_BIT_WIDTH  = 1     ;
    parameter STAGE2_INT_BIT_WIDTH  = 13    ;
    parameter STAGE2_FRA_BIT_WIDTH  = 1     ;
    parameter STAGE3_INT_BIT_WIDTH  = 13    ;
    parameter STAGE3_FRA_BIT_WIDTH  = 1     ;
    parameter STAGE4_INT_BIT_WIDTH  = 13    ;
    parameter STAGE4_FRA_BIT_WIDTH  = 1     ;
    parameter STAGE5_INT_BIT_WIDTH  = 14    ;
    parameter STAGE5_FRA_BIT_WIDTH  = 1     ;
    parameter STAGE6_INT_BIT_WIDTH  = 14    ;
    parameter STAGE6_FRA_BIT_WIDTH  = 0     ;
    parameter STAGE7_INT_BIT_WIDTH  = 14    ;
    parameter STAGE7_FRA_BIT_WIDTH  = 0     ;
    parameter STAGE8_INT_BIT_WIDTH  = 15    ;
    parameter STAGE8_FRA_BIT_WIDTH  = 0     ;
    parameter TW_BIT_WIDTH          = 8     ;
    parameter DATAOUT_WIDTH         = 16    ;
    
    localparam N_PE_CLUSTER         = N_ELEMENT * N_PE_COL;
    localparam DATAIN_WIDTH         = Input_INT_BIT_WIDTH + Input_FRA_BIT_WIDTH;
    
    // system clock signals ---------------------------------------------------
    logic                       sys_clk;
    logic                       rst_n;
    
    // i2s signals ------------------------------------------------------------
    logic                       MCLK;
    logic                       ADC_SDATA;
    logic                       BCLK;
    logic                       LRCLK;
    
    logic                       SCK;
    logic                       CS;
    logic                       MOSI;
    
    logic [3:0]                 Result;
    logic                       Inf_Done;
    
    
    logic [N_FRAME-1:0]         feature_gold_value      [0:2*N_MEL-1];
    logic [DATAIN_WIDTH-1:0]    data_in                 [0:16640-1];
    logic signed [13:0]         sum_result              [12];                
    logic signed [13:0]         sum_result_temp;
    
    int file, r, i, j, k, m, round, index;
    string line;
    integer status;
    int conf_count;
    int check, feature_check, result_check;
    
    logic [31:0]    SPI_CONF_ARRAY [28];
    bit             is_valid;
    string          binary_str;
    logic [31:0]    tmp;
    
    logic [31:0]    BLOCK_IDX_BANK_CONFIG_ADDR;
    logic [31:0]    BLOCK_IDX_BANK_ARRAY        [DEPTH_BLOCK_BANK];
    int             LEN_BLOCK_BANK;
    
    logic [31:0]    ROW_CNT_BANK_CONFIG_ADDR    [N_PE_COL];
    logic [31:0]    ROW_CNT_BANK_ARRAY          [N_PE_COL] [DEPTH_ROW_BANK];
    int             LEN_ROW_BANK                [N_PE_COL];
    string          row_cnt_file                [N_PE_COL];
    
    logic [31:0]    CCL_IDX_BANK_CONFIG_ADDR    [N_PE_COL];
    logic [31:0]    CCL_IDX_BANK_ARRAY          [N_PE_COL] [DEPTH_CCL_BANK];
    int             LEN_CCL_BANK                [N_PE_COL];
    string          ccl_idx_file                [N_PE_COL];
    
    logic [31:0]    WEIGHT_BANK_CONFIG_ADDR;
    logic [31:0]    WEIGHT_BANK_ARRAY           [DEPTH_WEIGHT_BANK];
    int             LEN_WEIGHT_BANK;
    
    logic [31:0] CONF_SPI_EN_INF_ADDR;
    logic [31:0] CONF_SPI_EN_INF_DATA;
        
    wrap_TsetlinKWS wrap_TsetlinKWS_inst(
        .*
    );
    
    initial begin
        MCLK = 0;
        forever #61.035 MCLK = ~MCLK;           // 8.192mhz
    end
    
    initial begin
        sys_clk = 0;
        forever #1250 sys_clk = ~sys_clk;       // 400khz
    end
    
    assign BLOCK_IDX_BANK_CONFIG_ADDR      = 32'b1001_000_0010001111111_000000000000;   // 1152
    
    assign ROW_CNT_BANK_CONFIG_ADDR[0 ]    = 32'b1010_000_0011000110111_000000000000;   // 1592
    assign ROW_CNT_BANK_CONFIG_ADDR[1 ]    = 32'b1010_001_0011001010001_000000000000;   // 1618
    assign ROW_CNT_BANK_CONFIG_ADDR[2 ]    = 32'b1010_010_0011001011010_000000000000;   // 1627
    assign ROW_CNT_BANK_CONFIG_ADDR[3 ]    = 32'b1010_011_0011001001101_000000000000;   // 1614
    assign ROW_CNT_BANK_CONFIG_ADDR[4 ]    = 32'b1010_100_0011010000011_000000000000;   // 1668
    
    
    assign CCL_IDX_BANK_CONFIG_ADDR[0 ]    = 32'b1011_000_0110000110110_000000000000;   // 3127
    assign CCL_IDX_BANK_CONFIG_ADDR[1 ]    = 32'b1011_001_0110000101001_000000000000;   // 3114
    assign CCL_IDX_BANK_CONFIG_ADDR[2 ]    = 32'b1011_010_0110000011010_000000000000;   // 3099
    assign CCL_IDX_BANK_CONFIG_ADDR[3 ]    = 32'b1011_011_0110000100011_000000000000;   // 3108
    assign CCL_IDX_BANK_CONFIG_ADDR[4 ]    = 32'b1011_100_0110001110001_000000000000;   // 3186
    
    
    assign WEIGHT_BANK_CONFIG_ADDR         = 32'b1100_000_0010110011111_000000000000;   // 1440
    
    assign CONF_SPI_EN_INF_ADDR = 32'b1000_000_0000000000000_000000000001;
    assign CONF_SPI_EN_INF_DATA = 32'd1;
    
    
    initial begin
        LEN_BLOCK_BANK    = 1152;
        
        LEN_ROW_BANK[0 ]  = 1592;
        LEN_ROW_BANK[1 ]  = 1618;
        LEN_ROW_BANK[2 ]  = 1627;
        LEN_ROW_BANK[3 ]  = 1614;
        LEN_ROW_BANK[4 ]  = 1668;
        
        row_cnt_file[0 ]  = "row_cnt_bank0.dat";
        row_cnt_file[1 ]  = "row_cnt_bank1.dat";
        row_cnt_file[2 ]  = "row_cnt_bank2.dat";
        row_cnt_file[3 ]  = "row_cnt_bank3.dat";
        row_cnt_file[4 ]  = "row_cnt_bank4.dat";
        
        LEN_CCL_BANK[0 ]  = 3127;
        LEN_CCL_BANK[1 ]  = 3114;
        LEN_CCL_BANK[2 ]  = 3099;
        LEN_CCL_BANK[3 ]  = 3108;
        LEN_CCL_BANK[4 ]  = 3186;
      
        ccl_idx_file[0 ]  = "col_cla_idx_bank0.dat";
        ccl_idx_file[1 ]  = "col_cla_idx_bank1.dat";
        ccl_idx_file[2 ]  = "col_cla_idx_bank2.dat";
        ccl_idx_file[3 ]  = "col_cla_idx_bank3.dat";
        ccl_idx_file[4 ]  = "col_cla_idx_bank4.dat";
        
        LEN_WEIGHT_BANK = 1440;
    end
    
    
    initial begin
        feature_check = 0;
        result_check = 0;
        CS = 1;
        SCK = 0;
        MOSI = 0;
        ADC_SDATA = 0;
        rst_n = 1;
        #10 rst_n = 0;
        #30 rst_n = 1;
        // delay two clock cycle to wait the ARSR reset circuit
        #450;
        #976.56;
        
        // ------------------------------------------------------------------------
        // Configure register
        // ------------------------------------------------------------------------
        CS = 0;
        #3000;
        i = 0;
        repeat (28) begin
            j = 0;
            repeat(32) begin
                MOSI = SPI_CONF_ARRAY[i][31-j];
                #5120;
                SCK = 1;
                #5120;
                SCK = 0;
                j = j + 1;
            end
            i = i + 1;
        end
        #3000;
        CS = 1;
        #10000;
        
        
        // ------------------------------------------------------------------------
        // Configure model block index bank
        // ------------------------------------------------------------------------
        CS = 0;
        #3000;
        j = 0;
        repeat(32) begin
            MOSI = BLOCK_IDX_BANK_CONFIG_ADDR[31-j];
            #5120;
            SCK = 1;
            #5120;
            SCK = 0;
            j = j + 1;
        end
        
        i = 0;
        repeat (LEN_BLOCK_BANK) begin
            j = 0;
            repeat(32) begin
                MOSI = BLOCK_IDX_BANK_ARRAY[i][31-j];
                #5120;
                SCK = 1;
                #5120;
                SCK = 0;
                j = j + 1;
            end
            i = i + 1;
        end
        #3000;
        CS = 1;
        #10000;
        
        // ------------------------------------------------------------------------
        // Configure model row count bank
        // ------------------------------------------------------------------------
        for (int k = 0; k < N_PE_COL; k++) begin
            CS = 0;
            #3000;
            j = 0;
            repeat(32) begin
                MOSI = ROW_CNT_BANK_CONFIG_ADDR[k][31-j];
                #5120;
                SCK = 1;
                #5120;
                SCK = 0;
                j = j + 1;
            end
            i = 0;
            repeat (LEN_ROW_BANK[k]) begin
                j = 0;
                repeat(32) begin
                    MOSI = ROW_CNT_BANK_ARRAY[k][i][31-j];
                    #5120;
                    SCK = 1;
                    #5120;
                    SCK = 0;
                    j = j + 1;
                end
                i = i + 1;
            end
            #3000;
            CS = 1;
            #10000;
        end
        
        // ------------------------------------------------------------------------
        // Configure model ccl index bank
        // ------------------------------------------------------------------------
        for (int k = 0; k < N_PE_COL; k++) begin
            CS = 0;
            #3000;
            j = 0;
            repeat(32) begin
                MOSI = CCL_IDX_BANK_CONFIG_ADDR[k][31-j];
                #5120;
                SCK = 1;
                #5120;
                SCK = 0;
                j = j + 1;
            end
            i = 0;
            repeat (LEN_CCL_BANK[k]) begin
                j = 0;
                repeat(32) begin
                    MOSI = CCL_IDX_BANK_ARRAY[k][i][31-j];
                    #5120;
                    SCK = 1;
                    #5120;
                    SCK = 0;
                    j = j + 1;
                end
                i = i + 1;
            end
            #3000;
            CS = 1;
            #10000;
        end
        
        // ------------------------------------------------------------------------
        // Configure model weight bank
        // ------------------------------------------------------------------------
        
        CS = 0;
        #3000;
        j = 0;
        repeat(32) begin
            MOSI = WEIGHT_BANK_CONFIG_ADDR[31-j];
            #5120;
            SCK = 1;
            #5120;
            SCK = 0;
            j = j + 1;
        end
        i = 0;
        repeat (LEN_WEIGHT_BANK) begin
            j = 0;
            repeat(32) begin
                MOSI = WEIGHT_BANK_ARRAY[i][31-j];
                #5120;
                SCK = 1;
                #5120;
                SCK = 0;
                j = j + 1;
            end
            i = i + 1;
        end
        #3000;
        CS = 1;
        #10000;
        
        // ------------------------------------------------------------------------
        // Enable SPI_EN_INF
        // ------------------------------------------------------------------------
        CS = 0;
        #3000;
        j = 0;
        repeat(32) begin
            MOSI = CONF_SPI_EN_INF_ADDR[31-j];
            #5120;
            SCK = 1;
            #5120;
            SCK = 0;
            j = j + 1;
        end
        
        j = 0;
        repeat(32) begin
            MOSI = CONF_SPI_EN_INF_DATA[31-j];
            #5120;
            SCK = 1;
            #5120;
            SCK = 0;
            j = j + 1;
        end
        #3000;
        CS = 1;
        
        // delay to wait the sync spi_en signal
        #976.56;
        #976.56;
        #800;
        #22460;
        // ------------------------------------------------------------------------
        // Imitate i2s audio input
        // ------------------------------------------------------------------------
        round = 0;
        repeat (1) begin // stimulate 1s audio data
            j = 0;
            repeat (16640) begin
                k = 0;
                repeat (64) begin
                    if ( k >= 1 && k <= 12) begin
                        ADC_SDATA = data_in[j][12-k];   // MSB first
                    end else begin
                        ADC_SDATA = 1'bx;
                    end
                    k = k + 1;
                    #976.56;
                    
                end
                j = j + 1;
            end
            round = round + 1;
        end
        
        j = 0;
        k = 0;
        repeat (64) begin
            if ( k >= 1 && k <= 12) begin
                ADC_SDATA = data_in[j][12-k];   // MSB first
            end else begin
                ADC_SDATA = 1'bx;
            end
            k = k + 1;
            #976.56;
        end
        
        // record the activity of 4 frames
        j = 1000;
        repeat (1024) begin
            k = 0;
            repeat (64) begin
                if ( k >= 1 && k <= 12) begin
                    ADC_SDATA = data_in[j][12-k];   // MSB first
                end else begin
                    ADC_SDATA = 1'bx;
                end
                k = k + 1;
                #976.56;
                
            end
            j = j + 1;
        end
        #1000;
        
        $finish;
    end
    
    // check the feature bank
    always @(posedge wrap_TsetlinKWS_inst.TsetlinKWS_inst.feature_extractor_inst.fe_complete) begin
        if (feature_check == 0) begin
            #2500;
            #0.1;
            m = 0;
            repeat (64) begin
                if (wrap_TsetlinKWS_inst.TsetlinKWS_inst.feature_extractor_inst.feature_module_inst.feature_bank[m] != feature_gold_value[m])
                    $display("Error happen in Row %0d. My: %h. GOLD: %h.", m, 
                        wrap_TsetlinKWS_inst.TsetlinKWS_inst.feature_extractor_inst.feature_module_inst.feature_bank[m], feature_gold_value[m]);
                else
                    $display("Right happen in Row %0d. My: %h. GOLD: %h.", m, 
                        wrap_TsetlinKWS_inst.TsetlinKWS_inst.feature_extractor_inst.feature_module_inst.feature_bank[m], feature_gold_value[m]);
                m = m + 1;
            end
            feature_check = 1;
        end
    end
    
    // check results
    always @(posedge wrap_TsetlinKWS_inst.TsetlinKWS_inst.tsetlin_machine_accelerator_inst.argmax_inst.argmax_ena) begin
        if (result_check < 12) begin
            #2;
            sum_result_temp = wrap_TsetlinKWS_inst.TsetlinKWS_inst.tsetlin_machine_accelerator_inst.argmax_inst.class_summation;
            if (sum_result_temp != sum_result[result_check])
                $display("Error happen in Class %0d. My: %d. GOLD: %d.", result_check, 
                    sum_result_temp, sum_result[result_check]);
            else
                $display("Right happen in Class %0d. My: %d. GOLD: %d.", result_check, 
                    sum_result_temp, sum_result[result_check]);
            result_check = result_check + 1;
        end
    end
    
    // ------------------------------------------------------------------------
    // Read "sum_result.csv"
    // ------------------------------------------------------------------------
    initial begin
        sum_result[0] = 2204;
        sum_result[1] = -379;
        sum_result[2] = -1414;
        sum_result[3] = -1233;
        sum_result[4] = -473;
        sum_result[5] = -601;
        sum_result[6] = -1677;
        sum_result[7] = -1425;
        sum_result[8] = -1493;
        sum_result[9] = -350;
        sum_result[10] = -2066;
        sum_result[11] = 54;
    end

    // ------------------------------------------------------------------------
    // Read "audio_data.csv"
    // ------------------------------------------------------------------------
    initial begin
        file = $fopen("audio_data.csv", "r");
        if (file == 0) begin
            $display("Error: Unable to open audio_data file.");
            $finish;
        end

        i = 0; // row index
        while (!$feof(file)) begin
            line = "";
            r = $fgets(line, file); // read one row
            if (line != "") begin
                $sscanf(line, "%d", 
                        data_in[i]);
                i++;
            end
        end
        $fclose(file);

        for (i = 0; i < 2; i++) begin
            $display("data_in[%0d] = %0d", i, data_in[i]);
        end
    end
    
    // ------------------------------------------------------------------------
    // Read "mfcc_binary.csv"
    // ------------------------------------------------------------------------
    initial begin
        file = $fopen("mfcc_binary.csv", "r");
        if (file == 0) begin
            $display("Error: Cannot open mfcc_binary file.");
            $finish;
        end
        
        index = 0;
        while (!$feof(file) && index < 64) begin
            status = $fscanf(file, 
            "%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b\n",
            feature_gold_value[index][ 0], feature_gold_value[index][ 1], feature_gold_value[index][ 2], feature_gold_value[index][ 3],
            feature_gold_value[index][ 4], feature_gold_value[index][ 5], feature_gold_value[index][ 6], feature_gold_value[index][ 7],
            feature_gold_value[index][ 8], feature_gold_value[index][ 9], feature_gold_value[index][10], feature_gold_value[index][11],
            feature_gold_value[index][12], feature_gold_value[index][13], feature_gold_value[index][14], feature_gold_value[index][15],
            feature_gold_value[index][16], feature_gold_value[index][17], feature_gold_value[index][18], feature_gold_value[index][19],
            feature_gold_value[index][20], feature_gold_value[index][21], feature_gold_value[index][22], feature_gold_value[index][23],
            feature_gold_value[index][24], feature_gold_value[index][25], feature_gold_value[index][26], feature_gold_value[index][27],
            feature_gold_value[index][28], feature_gold_value[index][29], feature_gold_value[index][30], feature_gold_value[index][31],
            feature_gold_value[index][32], feature_gold_value[index][33], feature_gold_value[index][34], feature_gold_value[index][35],
            feature_gold_value[index][36], feature_gold_value[index][37], feature_gold_value[index][38], feature_gold_value[index][39],
            feature_gold_value[index][40], feature_gold_value[index][41], feature_gold_value[index][42], feature_gold_value[index][43],
            feature_gold_value[index][44], feature_gold_value[index][45], feature_gold_value[index][46], feature_gold_value[index][47],
            feature_gold_value[index][48], feature_gold_value[index][49], feature_gold_value[index][50], feature_gold_value[index][51],
            feature_gold_value[index][52], feature_gold_value[index][53], feature_gold_value[index][54], feature_gold_value[index][55],
            feature_gold_value[index][56], feature_gold_value[index][57], feature_gold_value[index][58], feature_gold_value[index][59],
            feature_gold_value[index][60], feature_gold_value[index][61], feature_gold_value[index][62], feature_gold_value[index][63]);
        
            if (status == 64) begin
                $display("Row %0d: %h", index, feature_gold_value[index]);
                index++;
            end else begin
                $display("Error: Incorrect file format at row %0d", index);
                $finish;
            end
        end
        $fclose(file);
        $display("File read complete.");
    end
    
    
    // ------------------------------------------------------------------------
    // Read "spi_config_reg.txt"
    // ------------------------------------------------------------------------
    initial begin
        conf_count = 0;
        file = $fopen("spi_config_reg.txt", "r");
        if (!file) begin
            $display("Error: Cannot open spi_config_reg.txt file");
            $finish;
        end

        while (!$feof(file)) begin
            line = "";
            r = $fgets(line, file); // read one row
            
            // Check the first 32 char
            is_valid = 1;
            if (line.len() >= 32) begin
                for (int i = 0; i < 32; i++) begin
                    if (line[i] != "0" && line[i] != "1") begin
                        is_valid = 0;
                        break;
                    end
                end
            end else begin
                is_valid = 0;  // if len less than 32
            end

            // char to logic[31:0]
            if (is_valid) begin
                tmp = 0;
                for (int i = 0; i < 32; i++) begin
                    tmp[31-i] = (line[i] == "1");
                end
                SPI_CONF_ARRAY[conf_count] = tmp;
                conf_count++;
            end
        end

        $fclose(file);
        $display("Read %0d valid configurations:", conf_count);
        for (int i = 0; i < conf_count; i++) begin
            $display("SPI_CONF_ARRAY[%0d] = 32'b%032b", i, SPI_CONF_ARRAY[i]);
        end
    end
    
    // ------------------------------------------------------------------------
    // Read "block_idx_bank.dat"
    // ------------------------------------------------------------------------
    initial begin
        conf_count = 0;
        file = $fopen("block_idx_bank.dat", "r");
        if (!file) begin
            $display("Error: Cannot open block_idx_bank.dat file");
            $finish;
        end

        while (!$feof(file)) begin
            line = "";
            r = $fgets(line, file); // read one row
            
            // Check the first 20 char
            is_valid = 1;
            if (line.len() >= 20) begin
                for (int i = 0; i < 20; i++) begin
                    if (line[i] != "0" && line[i] != "1") begin
                        is_valid = 0;
                        break;
                    end
                end
            end else begin
                is_valid = 0;  // if len less than 20
            end

            // char to logic[31:0]
            if (is_valid) begin
                tmp = 0;
                for (int i = 0; i < 20; i++) begin
                    tmp[19-i] = (line[i] == "1");
                end
                BLOCK_IDX_BANK_ARRAY[conf_count] = tmp;
                conf_count++;
            end
        end

        $fclose(file);
        $display("Read %0d valid configurations:", conf_count);
        for (int i = 0; i < 10; i++) begin
            $display("BLOCK_IDX_BANK_ARRAY[%0d] = 32'b%032b", i, BLOCK_IDX_BANK_ARRAY[i]);
        end
    end
    
    // ------------------------------------------------------------------------
    // Read "row_cnt_bank0.dat"
    // ------------------------------------------------------------------------
    initial begin
        for (int k = 0; k < N_PE_COL; k++) begin
            conf_count = 0;
            file = $fopen(row_cnt_file[k], "r");
            if (!file) begin
                $display("Error: Cannot open row_cnt_file.dat file");
                $finish;
            end

            while (!$feof(file)) begin
                line = "";
                r = $fgets(line, file); // read one row
                
                // Check the first 6 char
                is_valid = 1;
                if (line.len() >= 6) begin
                    for (int i = 0; i < 6; i++) begin
                        if (line[i] != "0" && line[i] != "1") begin
                            is_valid = 0;
                            break;
                        end
                    end
                end else begin
                    is_valid = 0;  // if len less than 6
                end

                // char to logic[31:0]
                if (is_valid) begin
                    tmp = 0;
                    for (int i = 0; i < 6; i++) begin
                        tmp[5-i] = (line[i] == "1");
                    end
                    ROW_CNT_BANK_ARRAY[k][conf_count] = tmp;
                    conf_count++;
                end
            end

            $fclose(file);
            $display("Read %0d valid configurations:", conf_count);
            for (int i = 0; i < 5; i++) begin
                $display("ROW_CNT_BANK_ARRAY[%0d][%0d] = 32'b%032b", k, i, ROW_CNT_BANK_ARRAY[k][i]);
            end
        end
    end
    
    // ------------------------------------------------------------------------
    // Read "col_cla_idx_bank0.dat"
    // ------------------------------------------------------------------------
    initial begin
        for (int k = 0; k < N_PE_COL; k++) begin
            conf_count = 0;
            file = $fopen(ccl_idx_file[k], "r");
            if (!file) begin
                $display("Error: Cannot open col_cla_idx_file.dat file");
                $finish;
            end

            while (!$feof(file)) begin
                line = "";
                r = $fgets(line, file); // read one row
                
                // Check the first 5 char
                is_valid = 1;
                if (line.len() >= 5) begin
                    for (int i = 0; i < 5; i++) begin
                        if (line[i] != "0" && line[i] != "1") begin
                            is_valid = 0;
                            break;
                        end
                    end
                end else begin
                    is_valid = 0;  // if len less than 5
                end

                // char to logic[31:0]
                if (is_valid) begin
                    tmp = 0;
                    for (int i = 0; i < 5; i++) begin
                        tmp[4-i] = (line[i] == "1");
                    end
                    CCL_IDX_BANK_ARRAY[k][conf_count] = tmp;
                    conf_count++;
                end
            end

            $fclose(file);
            $display("Read %0d valid configurations:", conf_count);
            for (int i = 0; i < 5; i++) begin
                $display("CCL_IDX_BANK_ARRAY[%0d][%0d] = 32'b%032b", k, i, CCL_IDX_BANK_ARRAY[k][i]);
            end
        end
    end
    
    
    // ------------------------------------------------------------------------
    // Read "weight_bank.dat"
    // ------------------------------------------------------------------------
    initial begin
        conf_count = 0;
        file = $fopen("weight_bank.dat", "r");
        if (file == 0) begin
            $display("Error: Unable to open weight_bank.dat file.");
            $finish;
        end

        i = 0; // row index
        while (!$feof(file)) begin
            line = "";
            r = $fgets(line, file); // read one row
            if (line != "") begin
                $sscanf(line, "%h", 
                        WEIGHT_BANK_ARRAY[i]);
                i++;
            end
        end
        $fclose(file);
        $display("Read %0d valid configurations:", i);
        for (i = 0; i < 10; i++) begin
            $display("WEIGHT_BANK_ARRAY[%0d] = %0h", i, WEIGHT_BANK_ARRAY[i]);
        end
    end
    
    
endmodule
