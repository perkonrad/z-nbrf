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
| Fabric rate | Single 320 MHz domain, 1 complex sample/clk from the RFDC | 2.56 GSPS / x8 DDC = 320 MSPS. Raised from 256 MHz on request (2026-10-15). One clock domain for all DSP; PS crossings only at SmartConnect and async AXIS FIFOs |
| Reference clock | 409.6 MHz to the RFDC tile PLLs | Stock ZCU111/PYNQ LMX2594 setting; VCO 10.24 GHz = 25 × 409.6 → ADC 2.56 (÷4), DAC 5.12 (÷2) |
| BW programming | One integer R: Rs = 40/R MBd, BW = 50/R MHz, R = 8…63 | R = 10 gives exactly 5 MHz, R = 50 exactly 1 MHz; a single CIC rate covers everything |
| Decimation/interpolation | CIC (N=4, ×R) → 23-tap CFIR (×2, droop comp) → 41-tap RRC at 4 sps | CFIR coefficients are R-independent (CIC droop in fs1 units barely changes for R ≥ 8) |
| Pulse | RRC α = 0.25, span 10 symbols | ISI floor −38.7 dB; the measured clean MER of 37 dB is at that floor |
| Timing recovery | Cubic Farrow + Gardner + PI + mod-1 NCO (Rice structure) | Standard, symbol-rate agnostic, works at 4 sps input |
| Acquisition | 64-symbol preamble in 4 non-coherent quarters; data-aided phase/frequency init | Non-coherent quarters tolerate CFO up to ~Rs/32; DD loops alone can't pull in that far |
| Carrier/EQ | CORDIC derotator + 5-tap T-spaced DD-LMS EQ + 2nd-order DD PLL | Flat-ish lab channels need only a short EQ; it also fixes AGC residual scale |
| Header | 32 bits, BPSK on the I=Q diagonal, CRC-8 | QPSK header bits failed 15–20% at 9 dB Es/N0; BPSK-diagonal gives +3 dB per bit |
| Payload | PRBS15 scrambling, CRC-16, uncoded | FEC is the main open item |
| Idle | Continuous PRBS-23 QPSK between frames | Keeps the RX loops locked; RX acquires per frame anyway |
| Symbol back end | Two concurrent stages: A (derotator, owns φ) and B (EQ/slicer/PLL/LMS) | One FSM no longer fit the 64-clock minimum symbol period once multipliers were pipelined for 256 MHz |
| Timing closure style | Every multiplier output registered before adders, clamps or shifts; round/shift/saturate split into stages | 320 MHz (pipelined for 256 MHz originally) on a -2 part with long arithmetic chains |
| BD integration | Verilog-2001 wrapper `nb_radio_wrap.v` around the SV top | Vivado module references can't have a SystemVerilog top file |

## 2.4 GHz channel plan (R6)

| Decision | Choice | Why |
|---|---|---|
| Tuning | RFDC fine-mixer NCOs only; the modem stays at baseband | 48-bit NCO with mHz resolution; per-frame preamble means hopping needs no modem reset |
| ADC | 2.56 GSPS; 2402–2480 MHz is 2nd Nyquist (alias −158…−80 MHz) | Needs a 2.4 GHz BPF (required anyway). Was 2.048 GSPS / zone 3 at the 256 MHz clock |
| DAC | 5.12 GSPS, 1st Nyquist (normal mode), x8 → 640 MSPS, 2 samples/clk | The whole band is in zone 1, so no mix-mode; first image at 5120 − f ≈ 2.64–2.72 GHz. Was 4.096 GSPS mix-mode at 256 MHz |
| ×2 to 512 MSPS | Linear interpolation in fabric | Images measured < −95 dBc (CIC plus cos² response); a few LUTs |
| NCO value | RF frequency folded into (−Fs/2, Fs/2] | Never inverts the spectrum, in any zone |
| 1 MHz grid BW | R = 50 (1.00 MHz) or R = 60 (0.83 MHz) | R = 60 trades 17% throughput for ~5 dB more ACR |
| AGC | Multiplicative update, gain range 1/256…1024 | Backing the ADC off for neighbour headroom needs gain ~50–600; the linear-step AGC took >9k symbols to get there |
| Channel select (2026-10-15) | RFDC NCOs fixed at 2441 MHz; fabric NCO (`nco_mix`) per DAC lane and on the ADC; channel from the `chan_i`/`chan_stb_i` port or the CHAN register | Asked for direct channel control, not through AXI. The RFDC Gen1 mixer can only be retuned over AXI-Lite plus an update event; a fabric NCO hops in ~8 clocks from a plain port |
| TX NCO placement | After the ×2 linear interpolator, one mixer per DAC lane | Mixing before the interpolator would leave its image at −29 dBc for a 39 MHz offset; after it, −86 dBc measured |
| NCO quality | 1024 × 18-bit quarter-wave ROM with a half-sample offset, 12-bit phase | Exact quadrant mirroring; phase-truncation spurs ≈ −72 dBc, well below the 12-bit ADC and the modem's MER |
| Hop handshake | `tx_busy_o` (frame loading/queued/on air + 16-symbol filter tail), `rx_busy_o` (deframer active) | A hopper must not retune mid-frame; the modem can't know when the far end is done, so it only reports its own state |

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
| (TB) Every case hung in the first register write on Verilator 5.048 | The AXI-Lite BFM drove and sampled handshakes right after `@(posedge clk)`, which races the DUT's flops under `--timing` | BFM and packet source drive and sample on the falling edge |
| (docs) Bring-up guide said a wrong RX NCO sign gives no detections | On a real-input ADC a wrong NCO sign conjugates the signal; detection and the BPSK-diagonal header survive, only the payload CRC fails (`+INV=1`, `+IQSWAP=1`) | README §7 and `loopback_test.py` corrected |
| First frame after a hop lost with multipath (16-QAM, 30 dB) | The EQ kept taps converged to the old channel's echo phase | EQ held at identity for 64 clocks on every channel change |
| (TB) Off-centre channels degraded by the channel model | Linear fractional-delay interpolation is −23 dB accurate for a signal 39 MHz off DC | Cubic Lagrange interpolation |

## Verification status

* `tb/regress.sh`: 27 end-to-end cases pass at 320 MHz (Verilator 5.048) (`docs/results/regress.log`).
* BER within ~0.5–1 dB of theory for QPSK and 16-QAM at 5 MHz and 1 MHz (`docs/ber.png`, `docs/results/ber.json`; measured at the earlier 256 MHz clock, R = 8/40, the same 5/1 MHz channels).
* Yosys `synth_xilinx -family xcup`: ~19k LUT, ~31k FF, 299 DSP48E2, 2 RAMB36 (before the ×2 DAC interpolator, which adds a few hundred FFs).
* Channel port and fabric NCOs: TB drives the port like an external controller; hopping by 29/37 channels between frames, channel 78, and the RX trim sign (`+CFO=5e6 +TRIM=5e6` passes, `-5e6` fails) are in the regression. DAC dump at channel 78: interpolator image −86 dBc, mirror −101 dBc.
* **Not done:** Vivado run of `bd.tcl` and `build.tcl`, 320 MHz timing closure, hardware bring-up, slang re-check after the NCO change.

## Open items

1. Run `vivado/build.tcl`; fix any RFDC `CONFIG.*` names or enums the installed IP version rejects (README §8).
2. Confirm the RFDC stream mapping (ADC m00 = I, m01 = Q; DAC s00 = {Q1, I1, Q0, I0}) and NCO sign conventions (`sw/loopback_test.py` detects the signs).
3. Close timing at 320 MHz; watch the parallel FIRs, CIC integrators and CORDIC barrel shifter.
4. Two boards at 2.4 GHz: ±Rs/36 = ±22 kHz (±9 ppm) at 1 MHz. Use a shared reference or software AFC, or widen the estimator to 8 × 8 segments (≈ ±Rs/16, ~1 dB detection cost).
5. FEC on the payload (convolutional + Viterbi, or LDPC).
6. Optional: fold symmetric FIR taps (halves the DSPs), fractionally-spaced EQ, multi-channel.
