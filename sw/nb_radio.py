"""Python driver for the nb_radio modem (PYNQ / any Linux with /dev/mem via pynq.MMIO).

Typical use on the ZCU111 PYNQ image:

    from nb_radio import NbRadioOverlay
    ol = NbRadioOverlay("nb_radio.bit")        # programs PL, LMX/LMK clocks, RFDC NCOs
    ol.radio.configure(bw_mhz=1.0, mod="qpsk") # 1 MHz channel grid -> R = 40 (or 0.83 MHz, R = 48)
    ol.set_channel(17)                         # 2402 + 17 = 2419 MHz
    ol.radio.send(b"hello world")
    print(ol.radio.recv(timeout=1.0))
    print(ol.radio.status())
"""
import math
import time
import struct

FS0 = 256e6          # complex sample rate at the modem's DSP input/output
ADC_FS_MHZ = 2048.0  # RF-ADC sample rate (2.4 GHz = 3rd Nyquist zone, undersampled)
DAC_FS_MHZ = 4096.0  # RF-DAC sample rate (2.4 GHz = 2nd Nyquist zone, mix-mode)
CH_BASE_MHZ = 2402.0
CH_SPACING_MHZ = 1.0
ALPHA = 0.25

REG = dict(
    ID=0x00, VERSION=0x04, CTRL=0x08, RATE=0x0C, CIC_SHIFT=0x10, TX_GAIN=0x14, TEST=0x18,
    AGC=0x1C, TED=0x20, PLL=0x24, EQ=0x28, DET=0x2C,
    RX_OK=0x40, RX_CRC=0x44, RX_HDR=0x48, TX_FRAMES=0x4C, RX_OVF=0x50, PLL_FREQ=0x54,
    EVM_ACC=0x58, EVM_N=0x5C, AGC_GAIN=0x60, PEAK_M=0x64, PEAK_E=0x68, LAST_HDR=0x6C,
    TED_V=0x70, DET_CNT=0x74,
)
CTRL_TX_EN, CTRL_RX_EN, CTRL_TEST, CTRL_LPBK, CTRL_SRST, CTRL_EQ, CTRL_PLL, CTRL_16QAM = (1 << i for i in range(8))
S_QPSK = 8192


def _s32(v):
    return v - (1 << 32) if v & 0x80000000 else v


class NbRadio:
    """Register-level driver. `mmio` needs read(offset) / write(offset, value)."""

    def __init__(self, mmio, dma=None, buf_alloc=None):
        self.mmio = mmio
        self.dma = dma
        self._alloc = buf_alloc
        self.R = 8
        idv = self.mmio.read(REG["ID"])
        if idv != 0x4E425231:
            raise RuntimeError(f"nb_radio not found (ID=0x{idv:08x})")
        if dma is not None and buf_alloc is not None:
            self._txbuf = buf_alloc(shape=(4096,), dtype="u1")
            self._rxbuf = buf_alloc(shape=(4096 + 8,), dtype="u1")

    # ---------------------------------------------------------------- configuration
    @staticmethod
    def rate_for_bw(bw_mhz):
        """Occupied bandwidth = Rs*(1+alpha) = 40/R MHz, R in 8..63 (5.0 .. 0.635 MHz).
        On a 1 MHz channel grid use R=40 (exactly 1 MHz) or R=48 (0.83 MHz, guard band, +5 dB ACR)."""
        R = int(round(40.0 / bw_mhz))
        return max(8, min(63, R))

    def configure(self, bw_mhz=5.0, mod="qpsk", tx_backoff_db=1.0, eq=True, pll=True,
                  test_mode=False, digital_loopback=False, ifs_symbols=32, det_thr=0.4):
        R = self.rate_for_bw(bw_mhz)
        self.R = R
        rx_shift = math.ceil(4 * math.log2(R) - 1e-9)
        tx_shift = math.ceil(3 * math.log2(R) - 1e-9)
        # normalise CIC interpolator gain R^3/2^tx_shift to unity, then back off
        tx_gain = int(16384 * (2 ** tx_shift) / R ** 3 * 10 ** (-tx_backoff_db / 20))
        self.mmio.write(REG["RATE"], R)
        self.mmio.write(REG["CIC_SHIFT"], (tx_shift << 8) | rx_shift)
        self.mmio.write(REG["TX_GAIN"], min(tx_gain, 0xFFFF))
        self.mmio.write(REG["TEST"], (ifs_symbols << 16) | 64)
        self.mmio.write(REG["DET"], int(det_thr * 256) & 0xFF)
        ctrl = CTRL_TX_EN | CTRL_RX_EN | CTRL_SRST
        ctrl |= CTRL_EQ if eq else 0
        ctrl |= CTRL_PLL if pll else 0
        ctrl |= CTRL_16QAM if mod.lower() in ("16qam", "qam16") else 0
        ctrl |= CTRL_TEST if test_mode else 0
        ctrl |= CTRL_LPBK if digital_loopback else 0
        self.mmio.write(REG["CTRL"], ctrl)
        return dict(R=R, Rs=FS0 / (8 * R), bw=FS0 / (8 * R) * (1 + ALPHA))

    @property
    def symbol_rate(self):
        return FS0 / (8 * self.R)

    def set_loops(self, ted=(3, 10), pll=(3, 10), eq_mu_sh=6, agc_target=11650, agc_mu_sh=9):
        """Loop gains as right-shifts (larger = slower)."""
        self.mmio.write(REG["TED"], (ted[1] << 8) | ted[0])
        self.mmio.write(REG["PLL"], (pll[1] << 8) | pll[0])
        self.mmio.write(REG["EQ"], eq_mu_sh)
        self.mmio.write(REG["AGC"], (agc_mu_sh << 16) | agc_target)

    # ---------------------------------------------------------------- data path
    def send(self, payload: bytes):
        if not 1 <= len(payload) <= 4095:
            raise ValueError("payload must be 1..4095 bytes")
        n = len(payload)
        self._txbuf[:n] = list(payload)
        self._txbuf.flush()
        self.dma.sendchannel.transfer(self._txbuf, nbytes=n)
        self.dma.sendchannel.wait()

    def recv(self, timeout=1.0):
        """Returns (payload, seq, crc_ok, mod16) or None on timeout.
        DMA packet = payload | seq | status{bit0 crc_ok, bit1 16qam}."""
        ch = self.dma.recvchannel
        ch.transfer(self._rxbuf)
        t0 = time.time()
        while not ch.idle:
            if time.time() - t0 > timeout:
                return None
            time.sleep(1e-4)
        n = self.dma.read(0x58)       # S2MM_LENGTH: bytes actually received
        self._rxbuf.invalidate()
        pkt = bytes(self._rxbuf[:n])
        return pkt[:-2], pkt[-2], bool(pkt[-1] & 1), bool(pkt[-1] & 2)

    # ---------------------------------------------------------------- status
    def status(self):
        r = lambda k: self.mmio.read(REG[k])
        evm_n = r("EVM_N")
        evm = 10 * math.log10(r("EVM_ACC") * 256.0 / evm_n / (2 * S_QPSK ** 2)) if evm_n else float("nan")
        pe = r("PEAK_E")
        return dict(
            rx_ok=r("RX_OK"), rx_crc_err=r("RX_CRC"), rx_hdr_err=r("RX_HDR"),
            tx_frames=r("TX_FRAMES"), rx_overflow=r("RX_OVF"), detections=r("DET_CNT"),
            evm_db=evm, mer_db=-evm,
            cfo_hz=_s32(r("PLL_FREQ")) / 2 ** 32 * self.symbol_rate,
            agc_gain=r("AGC_GAIN") / 4096.0,   # Q10.12
            rho=r("PEAK_M") / pe if pe else 0.0,
            ted_v=_s32(r("TED_V")) / 2 ** 24,
        )


def nco_wrap(f_mhz, fs_mhz):
    """RF frequency folded into (-Fs/2, Fs/2]: the NCO value that moves +f_rf to DC (ADC) or
    synthesises +f_rf from DC (DAC) without spectral inversion, for any Nyquist zone."""
    return (f_mhz + fs_mhz / 2) % fs_mhz - fs_mhz / 2


def nyquist_zone(f_mhz, fs_mhz):
    return int(f_mhz // (fs_mhz / 2)) + 1


def channel_mhz(n, base=CH_BASE_MHZ, spacing=CH_SPACING_MHZ):
    return base + n * spacing


class NbRadioOverlay:
    """PYNQ overlay wrapper: clocks, RFDC mixers, modem.

    Channel selection is pure RFDC NCO tuning; the modem always works at baseband and
    re-acquires on every frame preamble, so hopping needs no modem reset."""

    def __init__(self, bitfile="nb_radio.bit", channel=0, program_clocks=True,
                 rx_sign=+1, tx_sign=+1):
        from pynq import Overlay, MMIO, allocate
        if program_clocks:
            import xrfclk
            try:
                xrfclk.set_ref_clks(lmk_freq=122.88, lmx_freq=409.6)
            except AttributeError:          # older PYNQ (v2.6)
                xrfclk.set_all_ref_clks(409.6)
        self.ol = Overlay(bitfile)
        self.rfdc = self.ol.rfdc
        self.adc = self.rfdc.adc_tiles[0].blocks[0]
        self.dac = self.rfdc.dac_tiles[0].blocks[0]
        # NCO sign conventions differ between RFdc driver versions; loopback_test.py detects them
        self.rx_sign, self.tx_sign = rx_sign, tx_sign
        ip = self.ol.ip_dict["nb_radio"]
        self.radio = NbRadio(MMIO(ip["phys_addr"], ip["addr_range"]), self.ol.axi_dma, allocate)
        self.set_channel(channel)

    def set_channel(self, n, base=CH_BASE_MHZ, spacing=CH_SPACING_MHZ):
        """Tune RX and TX to base + n*spacing MHz (default grid 2402 + n MHz)."""
        f = channel_mhz(n, base, spacing)
        self.set_carrier(f)
        return f

    def set_carrier(self, rx_mhz, tx_mhz=None, rx_offset_hz=0.0):
        """Program the RFDC fine-mixer NCOs (48-bit, ~mHz steps) and Nyquist zones.
        rx_offset_hz: software AFC trim (add the measured status()['cfo_hz'] to move the NCO onto the signal)."""
        import xrfdc
        tx_mhz = rx_mhz if tx_mhz is None else tx_mhz
        rx_eff = rx_mhz + rx_offset_hz * 1e-6
        plan = ((self.adc, rx_eff, ADC_FS_MHZ, self.rx_sign), (self.dac, tx_mhz, DAC_FS_MHZ, self.tx_sign))
        for blk, f, fs, sign in plan:
            zone = nyquist_zone(f, fs)
            blk.NyquistZone = 1 if zone % 2 else 2     # DAC: zone 2 = mix-mode
            ms = blk.MixerSettings
            ms["Freq"] = sign * nco_wrap(f, fs)
            blk.MixerSettings = ms
            blk.UpdateEvent(xrfdc.EVENT_MIXER)
        self.rx_mhz, self.tx_mhz = rx_mhz, tx_mhz
