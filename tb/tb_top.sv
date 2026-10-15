// End-to-end testbench: DMA-style packets -> nb_radio TX -> channel -> nb_radio RX -> check.
// Channel (320 MSPS complex baseband, i.e. what the RFDC DUC/DDC pair presents):
//   sample-rate offset + fractional delay (linear interp, ~50x oversampled), 2-path echo,
//   carrier frequency offset, AWGN at a given Es/N0, gain, 12-bit ADC quantisation.
// Plusargs: +R=<8..63> +MOD=<0|1> +ESN0=<dB> +CFO=<Hz> +SRO=<ppm> +MP=<0|1> +NPKT=<n>
//           +LMIN/+LMAX=<bytes> +LPBK=<0|1> (internal digital loopback) +GAIN=<lin> +DUMP=<0|1>
//           +INV=1 (conjugate RX input: spectral inversion) +IQSWAP=1 (swap ADC I/Q lanes)
//           +CHAN=<0..78> (via the channel port, default 39 = centre) +HOP=<k> (hop by k channels between frames)
//           +TRIM=<Hz> (RX NCO trim register; pair with +CFO to check the RX mixer's sign)
`timescale 1ns/1ps
module tb_top;
  logic clk = 0, aresetn = 0;
  localparam real FS = 320.0e6;   // fabric clock = complex sample rate at the modem ports
  always #1.5625 clk = ~clk;      // 320 MHz

  // ---------------- plusargs ----------------
  int    TEST = 0, BP = 0;
  int    R = 8, MOD = 0, NPKT = 20, LMIN = 1, LMAX = 200, MP = 0, LPBK = 0, DUMP = 1, SEED = 1;
  int    INV = 0, IQSWAP = 0, CHAN = 39, HOP = 0;   // CHAN 39 = 2441 MHz = RFDC centre (NCO at DC)
  real   TRIM = 0.0;
  logic [6:0] chan = 0;
  logic       chan_stb = 0, tx_busy, rx_busy;
  real   ESN0 = 40.0, CFO = 0.0, SRO = 0.0, GAIN = 0.5;

  // ---------------- DUT ----------------
  logic [7:0]  awaddr, araddr;
  logic        awvalid, awready, wvalid, wready, bvalid, arvalid, arready, rvalid;
  logic [31:0] wdata, rdata;
  logic [1:0]  bresp, rresp;
  logic [7:0]  tx_tdata;  logic tx_tvalid, tx_tlast, tx_tready;
  logic [7:0]  rx_tdata;  logic rx_tvalid, rx_tlast, rx_tready = 1;
  always @(posedge clk) rx_tready <= (BP == 0) ? 1'b1 : ($urandom % 4 == 0);
  logic [15:0] adc_i, adc_q;
  logic [63:0] dac;

  nb_radio_top dut (
    .clk(clk), .aresetn(aresetn),
    .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
    .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
    .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(1'b1),
    .s_axi_araddr(araddr), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
    .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(1'b1),
    .s_axis_tx_tdata(tx_tdata), .s_axis_tx_tvalid(tx_tvalid), .s_axis_tx_tlast(tx_tlast),
    .s_axis_tx_tready(tx_tready),
    .m_axis_rx_tdata(rx_tdata), .m_axis_rx_tvalid(rx_tvalid), .m_axis_rx_tlast(rx_tlast),
    .m_axis_rx_tready(rx_tready),
    .s_axis_adci_tdata(adc_i), .s_axis_adci_tvalid(1'b1), .s_axis_adci_tready(),
    .s_axis_adcq_tdata(adc_q), .s_axis_adcq_tvalid(1'b1), .s_axis_adcq_tready(),
    .m_axis_dac_tdata(dac), .m_axis_dac_tvalid(), .m_axis_dac_tready(1'b1),
    .chan_i(chan), .chan_stb_i(chan_stb), .tx_busy_o(tx_busy), .rx_busy_o(rx_busy)
  );

  // ---------------- AXI-Lite BFM ----------------
  // AXI-Lite BFM. Driven and sampled on the falling edge: with --timing, Verilator may resume a
  // process woken by @(posedge clk) before the DUT's flops evaluate that edge, so values the TB
  // assigns or reads right after a rising edge race the DUT (the TB hung in its first write).
  task automatic axil_wr(input logic [7:0] a, input logic [31:0] d);
    @(negedge clk);
    awaddr = a; wdata = d; awvalid = 1; wvalid = 1;
    while (!awready) @(negedge clk);      // stable here, so the next rising edge is the handshake
    @(negedge clk);
    awvalid = 0; wvalid = 0;
    while (!bvalid) @(negedge clk);
  endtask

  task automatic axil_rd(input logic [7:0] a, output logic [31:0] d);
    @(negedge clk);
    araddr = a; arvalid = 1;
    while (!arready) @(negedge clk);
    @(negedge clk);
    arvalid = 0;
    while (!rvalid) @(negedge clk);
    d = rdata;
  endtask

  // Channel port: as an external controller would drive it (no AXI). The strobe is held for
  // a few clocks; chan stays stable well past the modem's 2-flop synchroniser.
  task automatic set_chan(input int n);
    @(negedge clk);
    chan = 7'(n);
    repeat (2) @(negedge clk);
    chan_stb = 1;
    repeat (4) @(negedge clk);
    chan_stb = 0;
    repeat (4) @(negedge clk);
  endtask

  // ---------------- channel ----------------
  localparam int NB = 4096;
  real bi [NB], bq [NB];
  int  wp = 0;
  real ph = 0.0, pavg = 0.0, sigma = 0.0, aph = 0.0;
  real ACI = -300.0, ACI_OFF = 1.0e6;
  int  DACDUMP = 0, dfd = 0, dcnt = 0;
  longint n = 0;
  real mp_d, mp_ai, mp_aq;

  function automatic real gauss();
    real u1, u2;
    u1 = (real'($urandom) + 1.0) / 4294967297.0;
    u2 = (real'($urandom) + 1.0) / 4294967297.0;
    return $sqrt(-2.0 * $ln(u1)) * $cos(6.283185307179586 * u2);
  endfunction

  function automatic void rd_buf(input real d, output real oi, output real oq);
    real p; int i0; real fr; real h [4];
    p  = real'(wp) - 1.0 - d;
    i0 = int'($floor(p));
    fr = p - real'(i0);
    // cubic Lagrange over i0-1 .. i0+2: the fabric NCO puts the signal up to 39 MHz off DC,
    // where linear interpolation alone would add a -23 dB error
    h[0] = -fr * (fr - 1.0) * (fr - 2.0) / 6.0;
    h[1] = (fr + 1.0) * (fr - 1.0) * (fr - 2.0) / 2.0;
    h[2] = -(fr + 1.0) * fr * (fr - 2.0) / 2.0;
    h[3] = (fr + 1.0) * fr * (fr - 1.0) / 6.0;
    oi = 0.0; oq = 0.0;
    for (int k = 0; k < 4; k++) begin
      oi += h[k] * bi[((i0 - 1 + k) % NB + NB) % NB];
      oq += h[k] * bq[((i0 - 1 + k) % NB + NB) % NB];
    end
  endfunction

  function automatic logic [15:0] adc_q12(input real v);
    int k;
    k = int'($floor(v / 16.0 + 0.5));
    if (k > 2047) k = 2047;
    if (k < -2048) k = -2048;
    return 16'(k * 16);
  endfunction

  always @(posedge clk) begin
    real xi, xq, yi, yq, ei, eq_, d, ci, cs, zi, zq;
    xi = real'($signed(dac[47:32]));     // 320 MSPS sample x(n) (sample 1 of the pair)
    xq = real'($signed(dac[63:48]));
    bi[wp] = xi; bq[wp] = xq;
    wp = (wp + 1) % NB;
    // power tracker on transmitted signal (for Es/N0 calibration)
    pavg = pavg + ((xi * xi + xq * xq) - pavg) / 65536.0;
    // delay with sample-rate offset
    d = 2000.0 + real'(n) * SRO * 1e-6;     // +-1800 samples of drift headroom
    if (d < 4.0 || d > real'(NB) - 400.0) $fatal(1, "channel delay out of range");
    rd_buf(d, yi, yq);
    if (MP != 0) begin
      rd_buf(d + mp_d, ei, eq_);
      yi = yi + mp_ai * ei - mp_aq * eq_;
      yq = yq + mp_ai * eq_ + mp_aq * ei;
    end
    // adjacent-channel interferer: same waveform 3000 samples (~12 symbols at 1 MHz) earlier,
    // so different data, shifted by ACI_OFF Hz, ACI dB relative to the wanted signal
    if (ACI > -200.0) begin
      real ai, aq, ca, sa;
      rd_buf(d + 3000.0, ai, aq);
      aph = aph + 6.283185307179586 * ACI_OFF / FS;
      if (aph > 6.283185307179586) aph = aph - 6.283185307179586;
      ca = $cos(aph) * (10.0 ** (ACI / 20.0)); sa = $sin(aph) * (10.0 ** (ACI / 20.0));
      yi = yi + ai * ca - aq * sa;
      yq = yq + ai * sa + aq * ca;
    end
    // CFO
    ph = ph + 6.283185307179586 * CFO / FS;
    if (ph > 6.283185307179586) ph = ph - 6.283185307179586;
    ci = $cos(ph); cs = $sin(ph);
    zi = (yi * ci - yq * cs) * GAIN;
    zq = (yi * cs + yq * ci) * GAIN;
    // AWGN: sigma^2 (complex) = P * (fs/Rs) / (Es/N0)
    sigma = $sqrt(pavg * GAIN * GAIN * 8.0 * real'(R) / (10.0 ** (ESN0 / 10.0)) / 2.0);
    zi = zi + sigma * gauss();
    zq = zq + sigma * gauss();
    // RF front-end conventions: spectral inversion (wrong-sign NCO on a real ADC) = conjugate;
    // an I/Q lane swap is the same up to a 90 degree rotation
    if (INV != 0) zq = -zq;
    if (IQSWAP != 0) begin real t; t = zi; zi = zq; zq = t; end
    adc_i <= adc_q12(zi);
    adc_q <= adc_q12(zq);
    n++;
  end

  // ---------------- TX packet source ----------------
  int  n_ok = 0, n_crcfail = 0, n_mism = 0, n_rx = 0, bit_err = 0, bits_tot = 0;
  byte    exp_q [int][$];
  int     sent = 0;
  int     exp_mod [int];

  initial begin
    tx_tvalid = 0; tx_tlast = 0; tx_tdata = 0;
    wait (aresetn);
    repeat (2000) @(posedge clk);
    begin
      int warm;   // receiver runs continuously in a real link; let loops settle on idle symbols
      warm = 1500;
      void'($value$plusargs("WARM=%d", warm));
      repeat (warm * 8 * R) @(posedge clk);
    end
    if (TEST != 0) begin
      axil_wr(8'h08, 32'h63 | (MOD << 7) | (LPBK << 3) | 32'h4);   // enable test generator now
      wait (n_rx >= NPKT);
      sent = NPKT;
    end else
    for (int p = 0; p < NPKT; p++) begin
      int len;
      if (HOP != 0 && p > 0) begin
        // half-duplex hop: once the previous frame has left the TX and reached the RX
        wait (n_rx >= p);
        while (tx_busy || rx_busy) @(negedge clk);
        set_chan((CHAN + p * HOP) % 79);
      end
      len = LMIN + ($urandom % (LMAX - LMIN + 1));
      exp_q[p] = {};
      for (int b = 0; b < len; b++) exp_q[p].push_back(byte'($urandom));
      exp_mod[p] = MOD;
      for (int b = 0; b < len; b++) begin
        @(negedge clk);                    // same falling-edge discipline as the AXI-Lite BFM
        tx_tdata  = exp_q[p][b];
        tx_tvalid = 1;
        tx_tlast  = (b == len - 1);
        while (!tx_tready) @(negedge clk);
        @(negedge clk);
        tx_tvalid = 0;
        tx_tlast  = 0;
      end
      sent++;
    end
  end

  // ---------------- RX checker ----------------
  byte rxb [$];
  int  last_seq = -1, n_lost = 0;
  always @(posedge clk) begin
    if (rx_tvalid && rx_tready) begin
      rxb.push_back(rx_tdata);
      if (rx_tlast) begin
        int seq, st, plen, be;
        st   = rxb[rxb.size()-1];
        seq  = rxb[rxb.size()-2];
        plen = rxb.size() - 2;
        n_rx++;
        be = 0;
        if (TEST != 0) begin
          exp_q[seq] = {};
          for (int i = 0; i < 64; i++) exp_q[seq].push_back(byte'(seq + i));
        end
        if (exp_q.exists(seq) && exp_q[seq].size() == plen) begin
          for (int i = 0; i < plen; i++) be += $countones(rxb[i] ^ exp_q[seq][i]);
          bit_err  += be;
          bits_tot += plen * 8;
        end else be = -1;
        if (seq != last_seq + 1) n_lost += (seq - last_seq - 1);
        last_seq = seq;
        if (st[0]) n_ok++; else n_crcfail++;
        if (be != 0) n_mism++;
        if (st[0] && be != 0) $display("[%0t] ERROR: CRC ok but payload mismatch seq=%0d", $time, seq);
        $display("[%0t] RX frame seq=%0d len=%0d mod=%0d crc_ok=%0d biterr=%0d", $time, seq, plen, st[1], st[0], be);
        rxb = {};
      end
    end
  end

  // ---------------- raw 512 MSPS DAC stream dump (spectrum check) ----------------
  always @(posedge clk) if (DACDUMP > 0 && dcnt < DACDUMP && n > 400000) begin
    if (dfd == 0) dfd = $fopen("dac.txt", "w");
    // via int: Verilator 5.048 prints a $signed() part-select with garbage upper bits
    $fwrite(dfd, "%0d %0d %0d %0d\n", int'($signed(dac[15:0])), int'($signed(dac[31:16])),
            int'($signed(dac[47:32])), int'($signed(dac[63:48])));
    dcnt++;
    if (dcnt == DACDUMP) $fclose(dfd);
  end

  // ---------------- debug trace ----------------
  int DBG = 0;
  initial void'($value$plusargs("DBG=%d", DBG));
  always @(posedge clk) if (DBG != 0) begin
    if (dut.commit) $display("[%0t] commit busy=%0d m=%0d e=%0d", $time, dut.df_busy, dut.u_det.cand_m, dut.u_det.cand_e>>>4);
    if (DBG > 2 && dut.u_det.cdone) $display("   det cordic in=(%0d,%0d) z=%.1f deg  out=(%0d,%0d) est=%0d", dut.u_det.cx, dut.u_det.cy, real'($signed(dut.u_det.czo))/11930464.7, dut.u_det.cxo, dut.u_det.cyo, dut.u_det.est);
    if (DBG > 2 && dut.commit) for (int k=0;k<4;k++) $display("   C%0d = (%0d,%0d) ang %.1f", k, dut.u_det.cand_ci[k], dut.u_det.cand_cq[k], $atan2(real'(dut.u_det.cand_cq[k]),real'(dut.u_det.cand_ci[k]))*57.2958);
    if (dut.est_vld) $display("[%0t] est phase=%0d deg freq=%0d deg/sym", $time, $signed(dut.est_phase)/11930465, $signed(dut.est_freq)/11930465);
    if (DBG > 1 && dut.u_sym.cdone && (dut.u_sym.ptag || dut.u_deframer.st == 2 || dut.u_deframer.st == 1))
      $display("   rot tag=%0d df=%0d ang=%.0f  in_ang=%.0f phi=%.0f", dut.u_sym.ptag, dut.u_deframer.st,
        $atan2(real'(dut.u_sym.cyo), real'(dut.u_sym.cxo))*57.3, $atan2(real'(dut.u_sym.pq_), real'(dut.u_sym.pi_))*57.3,
        real'($signed(dut.u_sym.phi))/11930465.0);
    if (dut.u_deframer.st == 2 && dut.u_deframer.hcnt == 32) $display("[%0t] hdr=%08x", $time, dut.u_deframer.hdr);
  end

  // ---------------- symbol dump ----------------
  int fd;
  always @(posedge clk) if (DUMP != 0 && dut.sp_v)
    $fwrite(fd, "%0d %0d %0d %0d %0d\n", $signed(dut.sp_yi), $signed(dut.sp_yq), dut.df_busy, dut.df_mod16, dut.u_deframer.st);

  // ---------------- main ----------------
  initial begin
    logic [31:0] v, r_ok, r_crc, r_hdr, r_txf, r_ea, r_en, r_g, r_f, r_pm, r_pe, r_det;
    real evm_db, rs, cfo_est;
    int rshift, tshift, tg;
    void'($value$plusargs("R=%d", R));
    void'($value$plusargs("MOD=%d", MOD));
    void'($value$plusargs("NPKT=%d", NPKT));
    void'($value$plusargs("LMIN=%d", LMIN));
    void'($value$plusargs("LMAX=%d", LMAX));
    void'($value$plusargs("MP=%d", MP));
    void'($value$plusargs("LPBK=%d", LPBK));
    void'($value$plusargs("DUMP=%d", DUMP));
    void'($value$plusargs("SEED=%d", SEED));
    void'($value$plusargs("ESN0=%f", ESN0));
    void'($value$plusargs("CFO=%f", CFO));
    void'($value$plusargs("SRO=%f", SRO));
    void'($value$plusargs("GAIN=%f", GAIN));
    void'($value$plusargs("TEST=%d", TEST));
    void'($value$plusargs("BP=%d", BP));
    void'($value$plusargs("ACI=%f", ACI));
    void'($value$plusargs("ACI_OFF=%f", ACI_OFF));
    void'($value$plusargs("DACDUMP=%d", DACDUMP));
    void'($value$plusargs("INV=%d", INV));
    void'($value$plusargs("IQSWAP=%d", IQSWAP));
    void'($value$plusargs("CHAN=%d", CHAN));
    void'($value$plusargs("HOP=%d", HOP));
    void'($value$plusargs("TRIM=%f", TRIM));
    void'($urandom(SEED));
    mp_d  = 4.0 * real'(R);                // echo at 0.5 symbol
    mp_ai = 0.25 * $cos(0.7);
    mp_aq = 0.25 * $sin(0.7);
    rs    = FS / 8.0 / real'(R);
    if (DUMP != 0) fd = $fopen("syms.txt", "w");
    $display("=== nb_radio TB: R=%0d (Rs=%.3f MBd, BW=%.2f MHz) MOD=%s Es/N0=%.1f dB CFO=%.0f Hz SRO=%.1f ppm MP=%0d LPBK=%0d",
             R, rs / 1e6, 1.25 * rs / 1e6, MOD ? "16QAM" : "QPSK", ESN0, CFO, SRO, MP, LPBK);
    awvalid = 0; wvalid = 0; arvalid = 0;
    repeat (20) @(posedge clk);
    aresetn = 1;
    repeat (20) @(posedge clk);
    // configuration (what the software driver does)
    rshift = $clog2(R * R * R * R);        // ceil(4 log2 R)
    tshift = $clog2(R * R * R);            // ceil(3 log2 R)
    tg     = int'(16384.0 * real'(1 << tshift) / real'(R * R * R) * 0.9);
    axil_wr(8'h0C, R);
    axil_wr(8'h10, (tshift << 8) | rshift);
    axil_wr(8'h14, tg);
    if (TRIM != 0.0) axil_wr(8'h3C, 32'($rtoi(TRIM / FS * 4294967296.0)));   // RX NCO trim
    set_chan(CHAN);
    axil_wr(8'h08, 32'h63 | (MOD << 7) | (LPBK << 3) | 32'h10);   // + soft reset
    begin
      int t;
      if ($value$plusargs("TED=%h", t))  axil_wr(8'h20, t);
      if ($value$plusargs("PLL=%h", t))  axil_wr(8'h24, t);
      if ($value$plusargs("EQMU=%d", t)) axil_wr(8'h28, t);
      if ($value$plusargs("AGC=%h", t))  axil_wr(8'h1C, t);
      if ($value$plusargs("THR=%d", t))  axil_wr(8'h2C, t);
      if ($value$plusargs("CTRL=%h", t)) axil_wr(8'h08, t | 32'h10);
    end
    axil_rd(8'h00, v);
    $display("ID=%08x  rx_shift=%0d tx_shift=%0d tx_gain=0x%04x", v, rshift, tshift, tg);

    wait (sent == NPKT);
    // wait for the last frame to drain: (len+2)*8/bps + 96 symbols
    repeat (int'((real'(LMAX) + 2.0) * (MOD ? 2.0 : 4.0) + 400.0) * 8 * R) @(posedge clk);

    axil_rd(8'h40, r_ok);  axil_rd(8'h44, r_crc); axil_rd(8'h48, r_hdr); axil_rd(8'h4C, r_txf);
    axil_rd(8'h58, r_ea);  axil_rd(8'h5C, r_en);  axil_rd(8'h60, r_g);   axil_rd(8'h54, r_f);
    axil_rd(8'h64, r_pm);  axil_rd(8'h68, r_pe);  axil_rd(8'h74, r_det);
    // EVM: e scaled >>8 in acc; reference power = 2*S^2 = 2*8192^2
    evm_db  = 10.0 * $log10((real'(r_ea) * 256.0 / real'(r_en)) / (2.0 * 8192.0 * 8192.0));
    cfo_est = real'($signed(r_f)) / 4294967296.0 * rs;
    $display("--- summary ---");
    $display("TX frames %0d | RX ok %0d crc_err %0d hdr_err %0d | detections %0d | lost %0d",
             r_txf, r_ok, r_crc, r_hdr, r_det, n_lost + (NPKT - 1 - last_seq));
    $display("payload bits %0d, bit errors %0d (BER %.2e)", bits_tot, bit_err,
             bits_tot > 0 ? real'(bit_err) / real'(bits_tot) : 0.0);
    $display("last-frame EVM %.1f dB (MER %.1f dB) | AGC gain %.3f | PLL freq %.0f Hz (true %.0f) | rho %.3f",
             evm_db, -evm_db, real'(r_g) / 4096.0, cfo_est, CFO, real'(r_pm) / real'(r_pe));
    $display("EQ taps (re,im)/16384: %0d,%0d | %0d,%0d | %0d,%0d | %0d,%0d | %0d,%0d", dut.u_sym.wai[0]>>>14, dut.u_sym.waq[0]>>>14,
      dut.u_sym.wai[1]>>>14, dut.u_sym.waq[1]>>>14, dut.u_sym.wai[2]>>>14, dut.u_sym.waq[2]>>>14,
      dut.u_sym.wai[3]>>>14, dut.u_sym.waq[3]>>>14, dut.u_sym.wai[4]>>>14, dut.u_sym.waq[4]>>>14);
    $display("STATS ok=%0d crc=%0d hdr=%0d lost=%0d bits=%0d errs=%0d mer=%.2f", r_ok, r_crc, r_hdr,
             n_lost + (NPKT - 1 - last_seq), bits_tot, bit_err, -evm_db);
    if (TEST != 0) begin
      if (n_mism == 0 && n_ok == n_rx && n_rx >= NPKT) $display("RESULT: PASS");
      else                                            $display("RESULT: FAIL");
    end else if (ESN0 >= 25.0) begin
      if (r_ok == NPKT && bit_err == 0) $display("RESULT: PASS");
      else                              $display("RESULT: FAIL");
    end else begin
      // noisy run: every frame must be detected and its header decoded; bit errors are statistics
      if (r_ok + r_crc == NPKT && r_hdr == 0) $display("RESULT: PASS (BER %.2e)", bits_tot > 0 ? real'(bit_err) / real'(bits_tot) : 0.0);
      else                                    $display("RESULT: FAIL");
    end
    if (DUMP != 0) $fclose(fd);
    $finish;
  end

  initial begin
    #500ms;
    $display("TIMEOUT");
    $finish;
  end
endmodule
