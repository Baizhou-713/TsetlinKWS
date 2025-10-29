# for adau1761
set_property -dict {PACKAGE_PIN T17 IOSTANDARD LVCMOS33} [get_ports LRCLK]
set_property -dict {PACKAGE_PIN F17 IOSTANDARD LVCMOS33} [get_ports ADC_SDATA]
set_property -dict {PACKAGE_PIN R18 IOSTANDARD LVCMOS33} [get_ports BCLK]
set_property -dict {PACKAGE_PIN U5 IOSTANDARD LVCMOS33} [get_ports MCLK]
set_property -dict {PACKAGE_PIN U9 IOSTANDARD LVCMOS33} [get_ports IIC_scl_io]
set_property -dict {PACKAGE_PIN T9 IOSTANDARD LVCMOS33} [get_ports IIC_sda_io]

create_generated_clock -name sys_clk \
    -source [get_pins design_1_i/processing_system7_0/inst/PS7_i/FCLKCLK[0]] \
    -divide_by 100 \
    [get_pins design_1_i/clock_div_40m_2_400k_0/inst/clk_out_reg/Q]
    
create_clock -period 10240 -name SCK [get_pins design_1_i/processing_system7_0/inst/PS7_i/EMIOSPI0SCLKO]

create_generated_clock -name BCLK \
    -source [get_pins design_1_i/wrap_TsetlinKWS_0/inst/TsetlinKWS_inst/feature_extractor_inst/i2s_master_inst/BCLK_reg_reg/C] \
    -divide_by 8 \
    [get_pins design_1_i/wrap_TsetlinKWS_0/inst/TsetlinKWS_inst/feature_extractor_inst/i2s_master_inst/BCLK_reg_reg/Q]

create_generated_clock -name LRCLK \
    -source [get_pins design_1_i/wrap_TsetlinKWS_0/inst/TsetlinKWS_inst/feature_extractor_inst/i2s_master_inst/LRCLK_reg_reg/C] \
    -divide_by 512 \
    [get_pins design_1_i/wrap_TsetlinKWS_0/inst/TsetlinKWS_inst/feature_extractor_inst/i2s_master_inst/LRCLK_reg_reg/Q]

set_propagated_clock [all_clocks]

set_property DONT_TOUCH true [get_cells design_1_i/wrap_TsetlinKWS_0/inst/RESET_SYNC_FF1_sys]
set_property DONT_TOUCH true [get_cells design_1_i/wrap_TsetlinKWS_0/inst/RESET_SYNC_FF2_sys]
set_property DONT_TOUCH true [get_cells design_1_i/wrap_TsetlinKWS_0/inst/RESET_SYNC_FF1_MCLK]
set_property DONT_TOUCH true [get_cells design_1_i/wrap_TsetlinKWS_0/inst/RESET_SYNC_FF2_MCLK]

set_property RAM_STYLE REGISTER [get_cells -hierarchical -filter {NAME =~ *Pand0_SPad*}]
set_property RAM_STYLE REGISTER [get_cells -hierarchical -filter {NAME =~ *Pand1_SPad*}]

# Process asynchronous clock
set_clock_groups -asynchronous \
    -group [get_clocks {clk_fpga_0 sys_clk}] \
    -group [get_clocks SCK] \
    -group [get_clocks {clk_out1_design_1_clk_wiz_0_0 BCLK LRCLK}]

set_input_delay -clock BCLK -max 2.5 [get_ports ADC_SDATA]
set_input_delay -clock BCLK -min 0.5 [get_ports ADC_SDATA]
