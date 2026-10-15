# Build the ZCU111 nb_radio design end to end.
#   vivado -mode batch -source vivado/build.tcl [-tclargs <jobs>]
# Produces build/vivado/nb_radio.bit, nb_radio.hwh (PYNQ) and nb_radio.xsa (Vitis/PetaLinux).
set origin [file dirname [file normalize [info script]]]
set root   [file normalize $origin/..]
set jobs   [expr {[llength $argv] > 0 ? [lindex $argv 0] : 8}]
set part   xczu28dr-ffvg1517-2-e

create_project nb_radio $root/build/vivado -part $part -force
set bp [lindex [lsort -decreasing [get_board_parts -quiet xilinx.com:zcu111:*]] 0]
if {$bp ne ""} {
  set_property board_part $bp [current_project]
  puts "INFO: using board part $bp"
} else {
  puts "WARNING: ZCU111 board files not installed - PS preset not applied (DDR/MIO must be set manually)"
}

set srcs [list nb_pkg.sv nb_coefs_pkg.sv cordic.sv fir_cplx.sv cic_dec.sv cic_int.sv agc.sv \
              timing_rec.sv preamble_det.sv sym_proc.sv deframer.sv tx_framer.sv sync_fifo.sv \
              axil_regs.sv nco_mix.sv nb_radio_top.sv nb_radio_wrap.v]
foreach f $srcs { add_files -norecurse $root/rtl/$f }
set_property file_type SystemVerilog [get_files *.sv]
update_compile_order -fileset sources_1

source $origin/bd.tcl

make_wrapper -files [get_files $bd_name.bd] -top
add_files -norecurse [glob $root/build/vivado/nb_radio.gen/sources_1/bd/$bd_name/hdl/${bd_name}_wrapper.v]
set_property top ${bd_name}_wrapper [current_fileset]
update_compile_order -fileset sources_1

add_files -fileset constrs_1 -norecurse $origin/nb_radio.xdc

set_property strategy Performance_ExplorePostRoutePhysOpt [get_runs impl_1]
launch_runs synth_1 -jobs $jobs
wait_on_run synth_1
launch_runs impl_1 -to_step write_bitstream -jobs $jobs
wait_on_run impl_1

open_run impl_1
report_timing_summary -file $root/build/vivado/timing.rpt
report_utilization    -file $root/build/vivado/util.rpt
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
puts "INFO: WNS = $wns ns"

set out $root/build/vivado
file copy -force [glob $out/nb_radio.runs/impl_1/*.bit] $out/nb_radio.bit
file copy -force [glob $out/nb_radio.gen/sources_1/bd/$bd_name/hw_handoff/$bd_name.hwh] $out/nb_radio.hwh
write_hw_platform -fixed -include_bit -force $out/nb_radio.xsa
puts "INFO: done -> $out/nb_radio.{bit,hwh,xsa}"
