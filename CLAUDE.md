# z-nbrf — narrowband QPSK/16-QAM radio for the ZCU111 RFSoC

Start with `README.md` (architecture, register map, channel plan) and `docs/DESIGN_LOG.md`
(decisions, past bugs, open items).

## Layout
- `rtl/` SystemVerilog modem; `nb_radio_top.sv` is the top, `nb_radio_wrap.v` the Verilog
  wrapper for Vivado IP Integrator. `nb_coefs_pkg.sv` is generated — don't edit it by hand.
- `tb/` Verilator end-to-end testbench (TX → channel model → RX), `run.sh`, `regress.sh`.
- `scripts/` `gen_coefs.py` (filters, preamble, CORDIC table), `ber_sweep.py`, `plots.py`.
- `vivado/` `bd.tcl` (block design), `build.tcl` (project → bit/hwh/xsa).
- `sw/` PYNQ driver `nb_radio.py` (channel API `set_channel(n)` = 2402 + n MHz), bring-up script.

## Commands
- Tools: Verilator ≥ 5 and Yosys with the slang plugin (OSS CAD Suite); Python with numpy,
  scipy and matplotlib.
- Single sim: `tb/run.sh +R=50 +MOD=1 +ESN0=20 +CFO=10000 +SRO=20 +MP=1 +NPKT=10`
  (other plusargs: `+ACI= +ACI_OFF= +GAIN= +TEST= +LPBK= +BP= +INV= +IQSWAP= +DBG=1..3`). The first plusarg wins.
- Regression (must stay 100% pass): `tb/regress.sh`
- BER sweep: `python3 scripts/ber_sweep.py && python3 scripts/plots.py`
- Strict elaboration check (catches things Verilator accepts and Vivado rejects):
  `yosys -m slang -p "read_slang rtl/nb_pkg.sv rtl/nb_coefs_pkg.sv rtl/*.sv rtl/nb_radio_wrap.v --top nb_radio_wrap; hierarchy -check"`
- Bitstream: `vivado -mode batch -source vivado/build.tcl -tclargs 8`

## Rules of thumb for this codebase
- Fabric clock is 320 MHz (R = 10 → 5 MHz, R = 50 → 1 MHz): register every multiplier output before it feeds an adder, clamp or
  shift.
- Symbol-rate FSMs must finish within 64 clocks (R = 8). Simulation assertions flag overruns;
  run the regression at R = 8 after touching `sym_proc`, `timing_rec` or `preamble_det`.
- Phase arithmetic is mod 2³² (2³² = 2π). Never halve or divide a value that may have wrapped.
- Constellation scale: QPSK ±8192, 16-QAM ±3664 / ±10992. The AGC target 11650 is calibrated
  to that.
