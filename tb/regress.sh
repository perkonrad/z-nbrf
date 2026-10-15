#!/usr/bin/env bash
# Regression: each line is one end-to-end RTL simulation; all must print "RESULT: PASS".
set -uo pipefail
cd "$(dirname "$0")/.."
tb/run.sh +NPKT=1 +LMAX=4 +DUMP=0 > /dev/null   # build
TESTS=(
  "clean_5MHz_qpsk        +R=8  +MOD=0 +ESN0=60 +NPKT=10"
  "clean_5MHz_16qam       +R=8  +MOD=1 +ESN0=60 +NPKT=10"
  "clean_1MHz_16qam       +R=40 +MOD=1 +ESN0=60 +NPKT=6"
  "digital_loopback       +R=8  +MOD=1 +LPBK=1 +NPKT=6"
  "test_generator         +R=8  +MOD=0 +TEST=1 +ESN0=30 +NPKT=6"
  "rx_backpressure        +R=8  +MOD=1 +BP=1 +LMAX=300 +NPKT=10"
  "max_len_4095           +R=8  +MOD=1 +ESN0=30 +LMIN=4000 +LMAX=4095 +NPKT=3 +CFO=20000 +SRO=20"
  "tiny_frames            +R=8  +MOD=0 +LMIN=1 +LMAX=3 +NPKT=10"
  "bw_2.5MHz_impaired     +R=16 +MOD=1 +ESN0=30 +MP=1 +CFO=-15000 +SRO=-15 +NPKT=8"
  "bw_3.08MHz_impaired    +R=13 +MOD=1 +ESN0=30 +MP=1 +CFO=12000 +SRO=15 +NPKT=8"
  "bw_1MHz_impaired       +R=40 +MOD=1 +ESN0=30 +MP=1 +CFO=8000 +SRO=20 +NPKT=6"
  "5MHz_qpsk_10dB_all     +R=8  +MOD=0 +ESN0=10 +MP=1 +CFO=50000 +SRO=20 +NPKT=10"
  "5MHz_16qam_22dB_all    +R=8  +MOD=1 +ESN0=22 +MP=1 +CFO=20000 +SRO=20 +NPKT=10"
  "1MHz_qpsk_12dB_all     +R=40 +MOD=0 +ESN0=12 +MP=1 +CFO=10000 +SRO=-20 +NPKT=8"
  "cfo_edge_5MHz_+100k    +R=8  +MOD=0 +ESN0=15 +CFO=100000 +NPKT=6"
  "cfo_edge_5MHz_-110k    +R=8  +MOD=0 +ESN0=15 +CFO=-110000 +NPKT=6"
  "cfo_edge_1MHz_-22k     +R=40 +MOD=1 +ESN0=25 +CFO=-22000 +NPKT=6"
  "aci_1MHz_grid_+30dB    +R=40 +MOD=0 +ESN0=14 +ACI=30 +ACI_OFF=1000000 +GAIN=0.01 +NPKT=8"
  "aci_1MHz_grid_-1MHz    +R=40 +MOD=1 +ESN0=22 +ACI=20 +ACI_OFF=-1000000 +GAIN=0.01 +NPKT=8"
  "aci_0.83MHz_+35dB      +R=48 +MOD=0 +ESN0=14 +ACI=35 +ACI_OFF=1000000 +GAIN=0.01 +NPKT=8"
  "weak_input_-45dB       +R=40 +MOD=1 +ESN0=25 +GAIN=0.003 +WARM=4000 +NPKT=6"
  "bw_0.5MHz_clean        +R=80 +MOD=1 +ESN0=60 +NPKT=6"
  "bw_0.5MHz_impaired     +R=80 +MOD=1 +ESN0=22 +MP=1 +SRO=20 +CFO=6000 +NPKT=6"
  "bw_2MHz_impaired       +R=20 +MOD=1 +ESN0=22 +MP=1 +SRO=-20 +CFO=-25000 +NPKT=6"
  "bw_8MHz_clean          +R=5  +MOD=1 +ESN0=60 +NPKT=8"
  "bw_8MHz_impaired       +R=5  +MOD=1 +ESN0=22 +MP=1 +SRO=20 +CFO=100000 +NPKT=8"
  "bw_20MHz_clean         +R=2  +MOD=1 +ESN0=60 +NPKT=10"
  "bw_20MHz_impaired      +R=2  +MOD=1 +ESN0=22 +MP=1 +SRO=20 +CFO=250000 +NPKT=10"
  "bw_20MHz_qpsk_10dB     +R=2  +MOD=0 +ESN0=10 +MP=1 +SRO=-20 +CFO=-400000 +NPKT=10"
  "bw_20MHz_max_len       +R=2  +MOD=1 +ESN0=30 +LMIN=4000 +LMAX=4095 +NPKT=3"
  "timing_phase_at_wrap   +R=20 +MOD=1 +ESN0=60 +DLY0=2000 +NPKT=6"
)
pass=0; fail=0
for t in "${TESTS[@]}"; do
  name=${t%% *}; args=${t#* }
  out=$(tb/run.sh $args +DUMP=0 +LMAX=200 2>&1)   # first plusarg wins
  res=$(echo "$out" | grep -E "^RESULT" || echo "RESULT: FAIL (no result)")
  mer=$(echo "$out" | grep -oE "MER [-0-9.na]+ dB" | head -1)
  printf "%-24s %-34s %s\n" "$name" "$res" "$mer"
  if [[ $res == "RESULT: PASS"* ]]; then pass=$((pass+1)); else fail=$((fail+1)); fi
done
echo "passed $pass / $((pass+fail))"
[[ $fail -eq 0 ]]
