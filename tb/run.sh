#!/usr/bin/env bash
# Build (once) and run the end-to-end testbench with Verilator.
#   tb/run.sh [plusargs...]      e.g. tb/run.sh +R=50 +MOD=0 +ESN0=12 +CFO=3000 +SRO=20 +MP=1
set -euo pipefail
cd "$(dirname "$0")/.."
OBJ=build/tb
mkdir -p build
if [[ ! -x $OBJ/Vtb_top || -n "${REBUILD:-}" || -n "$(find rtl tb -name "*.sv" -newer $OBJ/Vtb_top)" ]]; then
  verilator --binary --timing -j 0 -O3 -Wno-fatal -Wno-WIDTH -Wno-MODDUP -Wno-PINCONNECTEMPTY \
    --top-module tb_top -Mdir $OBJ -Irtl \
    rtl/nb_pkg.sv rtl/nb_coefs_pkg.sv rtl/cordic.sv rtl/fir_cplx.sv rtl/cic_dec.sv rtl/cic_int.sv \
    rtl/agc.sv rtl/timing_rec.sv rtl/preamble_det.sv rtl/sym_proc.sv rtl/deframer.sv \
    rtl/tx_framer.sv rtl/sync_fifo.sv rtl/axil_regs.sv rtl/nb_radio_top.sv tb/tb_top.sv \
    > build/verilator.log 2>&1 || { tail -40 build/verilator.log; exit 1; }
fi
mkdir -p build/run && cd build/run
exec ../tb/Vtb_top "$@"
