# Block design for nb_radio on ZCU111 (XCZU28DR-2FFVG1517E).
#
#   PS (A53) --HPM0--> SmartConnect --> RFDC AXI-Lite (100 MHz)
#                                   --> AXI DMA lite   (100 MHz)
#                                   --> nb_radio regs  (256 MHz, SmartConnect does the CDC)
#   AXI DMA MM2S --> async AXIS FIFO --> nb_radio TX --> RF-DAC tile 228 ch0 (4.096 GSPS, IQ, x8,
#                                                        2 samples/clk, NCO, mix-mode = 2nd Nyquist)
#   RF-ADC tile 224 ch0 (2.048 GSPS, IQ, /8, NCO, 3rd-Nyquist undersampling at 2.4 GHz)
#                                       --> nb_radio RX --> async AXIS FIFO --> AXI DMA S2MM
#
# 2.4 GHz channel plan (2402 + n MHz): ADC alias = wrap(f, 2048) = +354+n MHz (zone 3, odd),
# DAC alias = wrap(f, 4096) = -1694+n MHz (zone 2, mix-mode). Runtime retuning: sw/nb_radio.py.
#   Radio clock: RFDC clk_adc0 (256 MHz) -> clk_wiz (BUFG + locked) -> m0/s0_axis_aclk + nb_radio
#
# RFDC configuration: ADC 2.048 GSPS, DAC 4.096 GSPS, tile PLLs from a 409.6 MHz reference
# (the ZCU111 LMX2594 must be programmed for 409.6 MHz -- e.g. xrfclk in PYNQ, or the
# Xilinx rfclk example). RFDC CONFIG property names below follow the Gen1 IP (v2.x);
# if a property is rejected by your Vivado version, set the same values in the RFDC GUI.

set bd_name nb_radio_bd

# ---- user knobs ---------------------------------------------------------------------------
set ADC_FS_GSPS  2.048      ;# RF-ADC sample rate  (AXIS = 2048/8 = 256 MSPS, 1 sample/clk)
set DAC_FS_GSPS  4.096      ;# RF-DAC sample rate  (AXIS = 4096/8 = 512 MSPS, 2 samples/clk)
set REFCLK_MHZ   409.600    ;# RFDC tile PLL reference
set AXIS_MHZ     256.000    ;# fabric / AXI4-Stream clock
set CHANNEL_MHZ  2402.0     ;# power-up channel (2402 + n MHz); retune at run time from software

# NCO = RF frequency folded into (-Fs/2, Fs/2]; Nyquist zone index (1-based) -> odd/even
proc nco_wrap {f_mhz fs_gsps} {
  set fs [expr {$fs_gsps * 1000.0}]
  set a  [expr {fmod($f_mhz + $fs/2.0, $fs)}]
  if {$a < 0} { set a [expr {$a + $fs}] }
  return [format %.6f [expr {($a - $fs/2.0) / 1000.0}]]
}
proc nyq_even {f_mhz fs_gsps} {
  return [expr {(int(floor($f_mhz / ($fs_gsps * 500.0))) % 2) == 1}]
}
set ADC_NCO_GHZ [nco_wrap $CHANNEL_MHZ $ADC_FS_GSPS]
set DAC_NCO_GHZ [nco_wrap $CHANNEL_MHZ $DAC_FS_GSPS]
set ADC_NYQ     [nyq_even $CHANNEL_MHZ $ADC_FS_GSPS]   ;# 0: odd zone (2402 MHz @ 2.048 GSPS -> zone 3)
set DAC_NYQ     [nyq_even $CHANNEL_MHZ $DAC_FS_GSPS]   ;# 1: even zone -> mix-mode (zone 2)
puts "INFO: channel $CHANNEL_MHZ MHz: ADC NCO $ADC_NCO_GHZ GHz (nyq_even=$ADC_NYQ), DAC NCO $DAC_NCO_GHZ GHz (nyq_even=$DAC_NYQ)"
# --------------------------------------------------------------------------------------------

create_bd_design $bd_name

# ---------------- Zynq UltraScale+ PS ----------------
set ps [create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e ps]
if {[get_property board_part [current_project]] ne ""} {
  apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e -config {apply_board_preset "1"} $ps
}
set_property -dict [list \
  CONFIG.PSU__USE__M_AXI_GP0 {1} \
  CONFIG.PSU__USE__M_AXI_GP1 {0} \
  CONFIG.PSU__USE__M_AXI_GP2 {0} \
  CONFIG.PSU__USE__S_AXI_GP2 {1} \
  CONFIG.PSU__USE__IRQ0 {1} \
  CONFIG.PSU__FPGA_PL0_ENABLE {1} \
  CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ {100} \
] $ps

# ---------------- RF Data Converter ----------------
set rfdc [create_bd_cell -type ip -vlnv xilinx.com:ip:usp_rf_data_converter rfdc]
set_property -dict [list \
  CONFIG.ADC0_Enable {1} \
  CONFIG.ADC0_PLL_Enable {true} \
  CONFIG.ADC0_Refclk_Freq $REFCLK_MHZ \
  CONFIG.ADC0_Sampling_Rate $ADC_FS_GSPS \
  CONFIG.ADC0_Fabric_Freq $AXIS_MHZ \
  CONFIG.ADC0_Outclk_Freq $AXIS_MHZ \
  CONFIG.ADC_Slice00_Enable {true} \
  CONFIG.ADC_Data_Type00 {1} \
  CONFIG.ADC_Decimation_Mode00 {8} \
  CONFIG.ADC_Mixer_Type00 {2} \
  CONFIG.ADC_Mixer_Mode00 {0} \
  CONFIG.ADC_NCO_Freq00 $ADC_NCO_GHZ \
  CONFIG.ADC_Nyquist00 $ADC_NYQ \
  CONFIG.ADC_Data_Width00 {1} \
  CONFIG.ADC_Slice01_Enable {false} \
  CONFIG.ADC1_Enable {0} \
  CONFIG.ADC2_Enable {0} \
  CONFIG.ADC3_Enable {0} \
  CONFIG.DAC0_Enable {1} \
  CONFIG.DAC0_PLL_Enable {true} \
  CONFIG.DAC0_Refclk_Freq $REFCLK_MHZ \
  CONFIG.DAC0_Sampling_Rate $DAC_FS_GSPS \
  CONFIG.DAC0_Fabric_Freq $AXIS_MHZ \
  CONFIG.DAC0_Outclk_Freq $AXIS_MHZ \
  CONFIG.DAC_Slice00_Enable {true} \
  CONFIG.DAC_Data_Type00 {1} \
  CONFIG.DAC_Interpolation_Mode00 {8} \
  CONFIG.DAC_Mixer_Type00 {2} \
  CONFIG.DAC_Mixer_Mode00 {0} \
  CONFIG.DAC_NCO_Freq00 $DAC_NCO_GHZ \
  CONFIG.DAC_Nyquist00 $DAC_NYQ \
  CONFIG.DAC_Data_Width00 {2} \
  CONFIG.DAC1_Enable {0} \
] $rfdc

# analog + clock pins are dedicated tile pins: just make them external
foreach intf {adc0_clk dac0_clk sysref_in vin0_01 vout00} {
  make_bd_intf_pins_external [get_bd_intf_pins rfdc/$intf]
}

# ---------------- radio clock ----------------
set cw [create_bd_cell -type ip -vlnv xilinx.com:ip:clk_wiz clk_radio]
set_property -dict [list \
  CONFIG.PRIM_IN_FREQ $AXIS_MHZ \
  CONFIG.CLKOUT1_REQUESTED_OUT_FREQ $AXIS_MHZ \
  CONFIG.USE_RESET {false} \
  CONFIG.USE_LOCKED {true} \
] $cw
connect_bd_net [get_bd_pins rfdc/clk_adc0] [get_bd_pins clk_radio/clk_in1]

set rst_ps [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset rst_ps]
set rst_rf [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset rst_radio]
connect_bd_net [get_bd_pins ps/pl_clk0]     [get_bd_pins rst_ps/slowest_sync_clk]
connect_bd_net [get_bd_pins ps/pl_resetn0]  [get_bd_pins rst_ps/ext_reset_in] [get_bd_pins rst_radio/ext_reset_in]
connect_bd_net [get_bd_pins clk_radio/clk_out1] [get_bd_pins rst_radio/slowest_sync_clk]
connect_bd_net [get_bd_pins clk_radio/locked]   [get_bd_pins rst_radio/dcm_locked]

set clk100  [get_bd_pins ps/pl_clk0]
set rst100  [get_bd_pins rst_ps/peripheral_aresetn]
set clkrf   [get_bd_pins clk_radio/clk_out1]
set rstrf   [get_bd_pins rst_radio/peripheral_aresetn]

# ---------------- modem ----------------
set nb [create_bd_cell -type module -reference nb_radio_wrap nb_radio]
set_property CONFIG.DAC_SPC {2} $nb      ;# 2 samples/clk to the 4.096 GSPS RF-DAC
connect_bd_net $clkrf [get_bd_pins nb_radio/clk] [get_bd_pins rfdc/m0_axis_aclk] [get_bd_pins rfdc/s0_axis_aclk]
connect_bd_net $rstrf [get_bd_pins nb_radio/aresetn] [get_bd_pins rfdc/m0_axis_aresetn] [get_bd_pins rfdc/s0_axis_aresetn]
connect_bd_intf_net [get_bd_intf_pins rfdc/m00_axis] [get_bd_intf_pins nb_radio/s_axis_adci]
connect_bd_intf_net [get_bd_intf_pins rfdc/m01_axis] [get_bd_intf_pins nb_radio/s_axis_adcq]
connect_bd_intf_net [get_bd_intf_pins nb_radio/m_axis_dac] [get_bd_intf_pins rfdc/s00_axis]

# ---------------- DMA + clock-crossing FIFOs ----------------
set dma [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_dma axi_dma]
set_property -dict [list \
  CONFIG.c_include_sg {0} \
  CONFIG.c_sg_length_width {16} \
  CONFIG.c_m_axis_mm2s_tdata_width {8} \
  CONFIG.c_s_axis_s2mm_tdata_width {8} \
  CONFIG.c_m_axi_mm2s_data_width {32} \
  CONFIG.c_m_axi_s2mm_data_width {32} \
  CONFIG.c_mm2s_burst_size {16} \
  CONFIG.c_s2mm_burst_size {16} \
] $dma

set ftx [create_bd_cell -type ip -vlnv xilinx.com:ip:axis_data_fifo fifo_tx]
set frx [create_bd_cell -type ip -vlnv xilinx.com:ip:axis_data_fifo fifo_rx]
foreach f [list $ftx $frx] {
  set_property -dict [list CONFIG.FIFO_DEPTH {4096} CONFIG.IS_ACLK_ASYNC {1} \
    CONFIG.TDATA_NUM_BYTES {1} CONFIG.HAS_TLAST {1}] $f
}
connect_bd_intf_net [get_bd_intf_pins axi_dma/M_AXIS_MM2S] [get_bd_intf_pins fifo_tx/S_AXIS]
connect_bd_intf_net [get_bd_intf_pins fifo_tx/M_AXIS]      [get_bd_intf_pins nb_radio/s_axis_tx]
connect_bd_intf_net [get_bd_intf_pins nb_radio/m_axis_rx]  [get_bd_intf_pins fifo_rx/S_AXIS]
connect_bd_intf_net [get_bd_intf_pins fifo_rx/M_AXIS]      [get_bd_intf_pins axi_dma/S_AXIS_S2MM]
connect_bd_net $clk100 [get_bd_pins fifo_tx/s_axis_aclk] [get_bd_pins fifo_rx/m_axis_aclk]
connect_bd_net $clkrf  [get_bd_pins fifo_tx/m_axis_aclk] [get_bd_pins fifo_rx/s_axis_aclk]
connect_bd_net $rst100 [get_bd_pins fifo_tx/s_axis_aresetn]
connect_bd_net $rstrf  [get_bd_pins fifo_rx/s_axis_aresetn]

# ---------------- control interconnect ----------------
set sc_ctl [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect sc_ctl]
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {3} CONFIG.NUM_CLKS {2}] $sc_ctl
connect_bd_intf_net [get_bd_intf_pins ps/M_AXI_HPM0_FPD] [get_bd_intf_pins sc_ctl/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins sc_ctl/M00_AXI] [get_bd_intf_pins rfdc/s_axi]
connect_bd_intf_net [get_bd_intf_pins sc_ctl/M01_AXI] [get_bd_intf_pins axi_dma/S_AXI_LITE]
connect_bd_intf_net [get_bd_intf_pins sc_ctl/M02_AXI] [get_bd_intf_pins nb_radio/s_axi]
connect_bd_net $clk100 [get_bd_pins sc_ctl/aclk] [get_bd_pins ps/maxihpm0_fpd_aclk] \
  [get_bd_pins rfdc/s_axi_aclk] [get_bd_pins axi_dma/s_axi_lite_aclk] [get_bd_pins axi_dma/m_axi_mm2s_aclk] \
  [get_bd_pins axi_dma/m_axi_s2mm_aclk] [get_bd_pins ps/saxihp0_fpd_aclk]
connect_bd_net $clkrf  [get_bd_pins sc_ctl/aclk1]
connect_bd_net $rst100 [get_bd_pins sc_ctl/aresetn] [get_bd_pins rfdc/s_axi_aresetn] [get_bd_pins axi_dma/axi_resetn]

# ---------------- DMA memory path ----------------
set sc_mem [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect sc_mem]
set_property -dict [list CONFIG.NUM_SI {2} CONFIG.NUM_MI {1}] $sc_mem
connect_bd_intf_net [get_bd_intf_pins axi_dma/M_AXI_MM2S] [get_bd_intf_pins sc_mem/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins axi_dma/M_AXI_S2MM] [get_bd_intf_pins sc_mem/S01_AXI]
connect_bd_intf_net [get_bd_intf_pins sc_mem/M00_AXI]     [get_bd_intf_pins ps/S_AXI_HP0_FPD]
connect_bd_net $clk100 [get_bd_pins sc_mem/aclk]
connect_bd_net $rst100 [get_bd_pins sc_mem/aresetn]

# ---------------- interrupts ----------------
set irq [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat irq_cat]
set_property CONFIG.NUM_PORTS {3} $irq
connect_bd_net [get_bd_pins axi_dma/mm2s_introut] [get_bd_pins irq_cat/In0]
connect_bd_net [get_bd_pins axi_dma/s2mm_introut] [get_bd_pins irq_cat/In1]
connect_bd_net [get_bd_pins rfdc/irq]             [get_bd_pins irq_cat/In2]
connect_bd_net [get_bd_pins irq_cat/dout]         [get_bd_pins ps/pl_ps_irq0]

# ---------------- addresses ----------------
assign_bd_address
set_property offset 0xA0040000 [get_bd_addr_segs {ps/Data/SEG_nb_radio_reg0}]
validate_bd_design
save_bd_design
