#!/usr/bin/env python3
"""ZCU111 bring-up / loopback test for nb_radio at 2.4 GHz (run on the board's PYNQ image).

RF path: DAC228 CH0 -> 2.4 GHz BPF -> 30-40 dB attenuation -> 2.4 GHz BPF -> ADC224 CH0.
The BPFs matter: the DAC also emits its image at 5120-f (~2.64..2.72 GHz) and the ADC
undersamples (2nd Nyquist), so anything at 80..158 MHz or 2.64..2.72 GHz folds onto the channel.

Stages:
  1. digital loopback (no RF)                    -- PL modem + DMA
  2. NCO sign detection with the test generator  -- RFdc driver conventions
  3. channel scan 2402 + n MHz, DMA packets       -- end to end
"""
import os, sys, time, itertools
sys.path.insert(0, os.path.dirname(__file__))
from nb_radio import NbRadioOverlay, channel_mhz


def run_dma(radio, n=30, size=256):
    ok = bad = lost = 0
    for _ in range(n):
        p = os.urandom(size)
        radio.send(p)
        r = radio.recv(timeout=0.5)
        if r is None:
            lost += 1
        elif r[2] and r[0] == p:
            ok += 1
        else:
            bad += 1
    return ok, bad, lost


def detect_signs(ol):
    """A wrong NCO sign on either side (or an I/Q lane swap) mirrors the spectrum: frames are
    still detected and headers still decode, but every payload fails its CRC. Two mirrors
    cancel, so two of the four combinations work; keep the one with the most good frames.
    Loopback can't tell which side was mirrored, only the pair; for two-board links that's
    enough as long as both boards use the same pair."""
    radio = ol.radio
    radio.configure(bw_mhz=1.0, mod="qpsk", test_mode=True)
    best = None
    for rs, ts in itertools.product((+1, -1), (+1, -1)):
        ol.rx_sign, ol.tx_sign = rs, ts
        ol.set_channel(0)
        radio.configure(bw_mhz=1.0, mod="qpsk", test_mode=True)   # soft reset clears counters
        time.sleep(0.3)
        st = radio.status()
        print(f"    rx_sign {rs:+d} tx_sign {ts:+d}: det {st['detections']:4d} ok {st['rx_ok']:4d} "
              f"crc_err {st['rx_crc_err']:4d} hdr_err {st['rx_hdr_err']:4d}  CFO {st['cfo_hz']:8.0f} Hz")
        if best is None or st["rx_ok"] > best[0]:
            best = (st["rx_ok"], rs, ts)
    ol.rx_sign, ol.tx_sign = best[1], best[2]
    print(f"    -> using rx_sign {ol.rx_sign:+d}, tx_sign {ol.tx_sign:+d}")


def main():
    ol = NbRadioOverlay("nb_radio.bit", channel=0)
    radio = ol.radio

    print("== 1. digital loopback")
    radio.configure(bw_mhz=1.0, mod="16qam", digital_loopback=True)
    time.sleep(0.05)
    print("   ", run_dma(radio, 20))

    print("== 2. NCO sign detection (test generator, channel 0 = 2402 MHz)")
    detect_signs(ol)

    print("== 3. channel scan, 1 MHz channels (R=50), QPSK and 16-QAM")
    for n in range(0, 79, 13):
        f = ol.set_channel(n)
        for mod in ("qpsk", "16qam"):
            radio.configure(bw_mhz=1.0, mod=mod)
            time.sleep(0.05)
            ok, bad, lost = run_dma(radio, 20, 256)
            st = radio.status()
            print(f"    ch {n:2d} ({f:.0f} MHz) {mod:5s}: ok {ok} bad {bad} lost {lost}  "
                  f"MER {st['mer_db']:.1f} dB  CFO {st['cfo_hz']:.0f} Hz  AGC {st['agc_gain']:.1f}")


if __name__ == "__main__":
    main()
