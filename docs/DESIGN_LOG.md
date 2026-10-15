# Design log

Why the design is the way it is: requirements, decisions, and the bugs verification caught.
Read this before changing anything structural.

## Requirements

| # | Requirement | Source |
|---|---|---|
| R1 | Narrowband radio on the ZCU111 (XCZU28DR RFSoC) | initial request |
| R2 | Channel *bandwidth* selectable 1–5 MHz | clarified: "1–5 MHz" means bandwidth, not carrier |
| R3 | RX + TX | clarified |
| R4 | QPSK / 16-QAM modem fully in PL | clarified |
| R5 | Vivado RTL + scripted block design (not PYNQ-only, not HLS) | clarified |
| R6 | RF channels at **2402 + n MHz** | follow-up |

## Architecture decisions

| Decision | Choice | Why |
|---|---|---|
| Fabric rate | Single 256 MHz domain, 1 complex sample/clk from the RFDC | 2.048 GSPS / x8 DDC = 256 MSPS. One clock domain for all DSP; PS crossings only at SmartConnect and async AXIS FIFOs |
| Reference clock | 409.6 MHz to the RFDC tile PLLs | Stock ZCU111/PYNQ LMX2594 setting; 2048 = 5 × 409.6 and 4096 = 10 × 409.6 |
| BW programming | One integer R: Rs = 32/R MBd, BW = 40/R MHz, R = 8…63 | R = 8 gives exactly 5 MHz, R = 40 exactly 1 MHz; a single CIC rate covers everything |
| Decimation/interpolation | CIC (N=4, ×R) → 23-tap CFIR (×2, droop comp) → 41-tap RRC at 4 sps | CFIR coefficients are R-independent (CIC droop in fs1 units barely changes for R ≥ 8) |
| Pulse | RRC α = 0.25, span 10 symbols | ISI floor −38.7 dB; the measured clean MER of 37 dB is at that floor |
| Timing recovery | Cubic Farrow + Gardner + PI + mod-1 NCO (Rice structure) | Standard, symbol-rate agnostic, works at 4 sps input |
| Acquisition | 64-symbol preamble in 4 non-coherent quarters; data-aided phase/frequency init | Non-coherent quarters tolerate CFO up to ~Rs/32; DD loops alone can't pull in that far |
| Carrier/EQ | CORDIC derotator + 5-tap T-spaced DD-LMS EQ + 2nd-order DD PLL | Flat-ish lab channels need only a short EQ; it also fixes AGC residual scale |
| Header | 32 bits, BPSK on the I=Q diagonal, CRC-8 | QPSK header bits failed 15–20% at 9 dB Es/N0; BPSK-diagonal gives +3 dB per bit |
| Payload | PRBS15 scrambling, CRC-16, uncoded | FEC is the main open item |
| Idle | Continuous PRBS-23 QPSK between frames | Keeps the RX loops locked; RX acquires per frame anyway |
| Symbol back end | Two concurrent stages: A (derotator, owns φ) and B (EQ/slicer/PLL/LMS) | One FSM no longer fit the 64-clock minimum symbol period once multipliers were pipelined for 256 MHz |
| Timing closure style | Every multiplier output registered before adders, clamps or shifts; round/shift/saturate split into stages | 256 MHz on a -2 part with long arithmetic chains |
| BD integration | Verilog-2001 wrapper `nb_radio_wrap.v` around the SV top | Vivado module references can't have a SystemVerilog top file |

## 2.4 GHz channel plan (R6)

| Decision | Choice | Why |
|---|---|---|
| Tuning | RFDC fine-mixer NCOs only; the modem stays at baseband | 48-bit NCO with mHz resolution; per-frame preamble means hopping needs no modem reset |
| ADC | Stays at 2.048 GSPS; 2402–2480 MHz is 3rd Nyquist (alias +354…+432 MHz) | No RTL change; needs a 2.4 GHz BPF (required anyway) |
| DAC | 4.096 GSPS, mix-mode (2nd Nyquist), x8 → 512 MSPS, 2 samples/clk | At 2.048 GSPS, 2.4 GHz is DAC zone 3: −17 dB and stronger images |
| ×2 to 512 MSPS | Linear interpolation in fabric | Images measured < −95 dBc (CIC plus cos² response); a few LUTs |
| NCO value | RF frequency folded into (−Fs/2, Fs/2] | Never inverts the spectrum, in any zone |
| 1 MHz grid BW | R = 40 (1.00 MHz) or R = 48 (0.83 MHz) | R = 48 trades 17% throughput for ~5 dB more ACR |
| AGC | Multiplicative update, gain range 1/256…1024 | Backing the ADC off for neighbour headroom needs gain ~50–600; the linear-step AGC took >9k symbols to get there |

## Bugs found by verification (all fixed, all covered by regression)

| Symptom | Root cause | Fix |
|---|---|---|
| Nothing locks | Gardner TED sign | Negated the error term |
| Frames lost at CFO ≈ 4.5°/sym | `(61ω)>>>1` on a value that had wrapped mod 2³² gives a π error | Compute 30ω + ω/2 |
| False detections at reset | E = 0 passes `m·4096 ≥ thr·E` | Minimum-energy guard |
| Headers rotated by k·90° | DD-LMS EQ inside the DD-PLL parked a rotation in its taps that the preamble re-init can't see | Phase anchoring: centre-tap angle moved into φ, taps counter-rotated, every symbol |
| Headers 180° off at ≥ ~5°/sym | The 4 quarter correlations span > 180°, so their vector sum flips | Align quarters with u = e^(−j16ω) before summing |
| Symbol overrun at R = 8 | Pipelined single FSM > 64 clocks/symbol | Split into stages A and B (assertions guard this) |
| Weak-signal / ACI failures | Linear AGC too slow at high gain, and capped at 64 | Multiplicative AGC, gain up to 1024 |
| `v4_d` used before declaration | Verilator accepted it, slang rejected it (Vivado may too) | Reordered; slang elaboration added as a check |
| (TB) SRO runs failed at R = 40 | Channel delay went negative → stale buffer reads | Centred delay with ±1800-sample headroom, `$fatal` on overrun |

## Verification status

* `tb/regress.sh`: 21 end-to-end cases pass (`docs/results/regress.log`).
* BER within ~0.5–1 dB of theory for QPSK and 16-QAM at 5 MHz and 1 MHz (`docs/ber.png`, `docs/results/ber.json`).
* Yosys `synth_xilinx -family xcup`: ~19k LUT, ~31k FF, 299 DSP48E2, 2 RAMB36 (before the ×2 DAC interpolator, which adds a few hundred FFs).
* **Not done:** Vivado run of `bd.tcl` and `build.tcl`, 256 MHz timing closure, hardware bring-up.

## Open items

1. Run `vivado/build.tcl`; fix any RFDC `CONFIG.*` names or enums the installed IP version rejects (README §8).
2. Confirm the RFDC stream mapping (ADC m00 = I, m01 = Q; DAC s00 = {Q1, I1, Q0, I0}) and NCO sign conventions (`sw/loopback_test.py` detects the signs).
3. Close timing at 256 MHz; watch the parallel FIRs, CIC integrators and CORDIC barrel shifter.
4. Two boards at 2.4 GHz: ±Rs/36 = ±22 kHz (±9 ppm) at 1 MHz. Use a shared reference or software AFC, or widen the estimator to 8 × 8 segments (≈ ±Rs/16, ~1 dB detection cost).
5. FEC on the payload (convolutional + Viterbi, or LDPC).
6. Optional: fold symmetric FIR taps (halves the DSPs), fractionally-spaced EQ, multi-channel.
