// Narrowband (1-5 MHz) QPSK/16-QAM burst modem for the ZCU111 RFSoC.
//
// Clocking: single 256 MHz domain = RFDC AXI4-Stream clock
//   RF-ADC 2.048 GSPS, IQ, DDC x8 -> 256 MSPS complex, 1 sample/clk (I and Q streams)
//   RF-DAC 4.096 GSPS, IQ, DUC x8 <- 512 MSPS complex, 2 samples/clk (DAC_SPC=2, default):
//          {Q1,I1,Q0,I0}, sample 0 earliest; x2 linear interpolation in fabric (images -70 dB).
//          DAC_SPC=1: 2.048 GSPS, 1 sample/clk {Q,I} (carriers < ~1.8 GHz).
// Channel: R = 8..40 -> Rs = 32/R MBd, BW = 1.25*Rs = 40/R MHz.
//
// TX: framer -> mapper -> RRC x4 -> CFIR x2 -> CIC xR -> gain -> RF-DAC
// RX: RF-ADC -> CIC /R -> CFIR /2 -> RRC -> AGC -> Gardner/Farrow -> preamble det.
//     -> derotator/EQ/PLL -> deframer -> byte FIFO
module nb_radio_top
  import nb_pkg::*;
  import nb_coefs_pkg::*;
#(
  parameter int DAC_SPC = 2          // RF-DAC samples per AXI4-Stream cycle (1 or 2)
)(
  (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi:s_axis_tx:m_axis_rx:s_axis_adci:s_axis_adcq:m_axis_dac, ASSOCIATED_RESET aresetn, FREQ_HZ 256000000" *)
  input  logic        clk,
  input  logic        aresetn,
  // AXI4-Lite control
  input  logic [7:0]  s_axi_awaddr,
  input  logic        s_axi_awvalid,
  output logic        s_axi_awready,
  input  logic [31:0] s_axi_wdata,
  input  logic [3:0]  s_axi_wstrb,
  input  logic        s_axi_wvalid,
  output logic        s_axi_wready,
  output logic [1:0]  s_axi_bresp,
  output logic        s_axi_bvalid,
  input  logic        s_axi_bready,
  input  logic [7:0]  s_axi_araddr,
  input  logic        s_axi_arvalid,
  output logic        s_axi_arready,
  output logic [31:0] s_axi_rdata,
  output logic [1:0]  s_axi_rresp,
  output logic        s_axi_rvalid,
  input  logic        s_axi_rready,
  // TX packets (from DMA MM2S)
  input  logic [7:0]  s_axis_tx_tdata,
  input  logic        s_axis_tx_tvalid,
  input  logic        s_axis_tx_tlast,
  output logic        s_axis_tx_tready,
  // RX packets (to DMA S2MM): payload, seq, status
  output logic [7:0]  m_axis_rx_tdata,
  output logic        m_axis_rx_tvalid,
  output logic        m_axis_rx_tlast,
  input  logic        m_axis_rx_tready,
  // RF-ADC (I and Q streams)
  input  logic [15:0] s_axis_adci_tdata,
  input  logic        s_axis_adci_tvalid,
  output logic        s_axis_adci_tready,
  input  logic [15:0] s_axis_adcq_tdata,
  input  logic        s_axis_adcq_tvalid,
  output logic        s_axis_adcq_tready,
  // RF-DAC: per sample {Q,I}, sample 0 in the LSBs
  output logic [32*DAC_SPC-1:0] m_axis_dac_tdata,
  output logic        m_axis_dac_tvalid,
  input  logic        m_axis_dac_tready
);
  // ======================= registers / reset =======================
  logic        rst_hw, rst;
  logic [31:0] ctrl;
  logic [5:0]  rate, rx_shift, tx_shift;
  logic [15:0] tx_gain, ifs, agc_target;
  logic [11:0] test_len;
  logic [4:0]  agc_mu_sh, ted_kp_sh, ted_ki_sh, pll_kp_sh, pll_ki_sh, eq_mu_sh;
  logic [7:0]  det_thr;

  logic [31:0] st_rx_ok, st_rx_crc, st_rx_hdr, st_tx_frames, st_rx_ovf, st_pll_freq;
  logic [31:0] st_evm_acc, st_evm_n, st_peak_m, st_peak_e, st_last_hdr, st_det_cnt;
  logic [21:0] agc_gain;
  logic signed [31:0] ted_v;

  always_ff @(posedge clk) begin
    rst_hw <= ~aresetn;
    rst    <= ~aresetn | ctrl[4];
  end

  wire tx_en     = ctrl[0];
  wire rx_en     = ctrl[1];
  wire test_mode = ctrl[2];
  wire dig_lpbk  = ctrl[3];
  wire eq_en     = ctrl[5];
  wire pll_en    = ctrl[6];
  wire tx_mod16  = ctrl[7];

  axil_regs u_regs (
    .clk(clk), .rst(rst_hw),
    .s_axi_awaddr, .s_axi_awvalid, .s_axi_awready, .s_axi_wdata, .s_axi_wstrb, .s_axi_wvalid,
    .s_axi_wready, .s_axi_bresp, .s_axi_bvalid, .s_axi_bready, .s_axi_araddr, .s_axi_arvalid,
    .s_axi_arready, .s_axi_rdata, .s_axi_rresp, .s_axi_rvalid, .s_axi_rready,
    .ctrl, .rate, .rx_shift, .tx_shift, .tx_gain, .test_len, .ifs, .agc_target, .agc_mu_sh,
    .ted_kp_sh, .ted_ki_sh, .pll_kp_sh, .pll_ki_sh, .eq_mu_sh, .det_thr,
    .st_rx_ok, .st_rx_crc, .st_rx_hdr, .st_tx_frames, .st_rx_ovf, .st_pll_freq,
    .st_evm_acc, .st_evm_n, .st_agc_gain({10'd0, agc_gain}), .st_peak_m, .st_peak_e,
    .st_last_hdr, .st_ted_v(ted_v), .st_det_cnt
  );

  // ======================= TX =======================
  // rate strobes: ce8 = fs0/R (8 sps), ce4 = 4 sps, ce_sym = 1 sps
  logic [5:0] cnt8;
  logic       ph8;
  logic [1:0] ph4;
  logic       ce8, ce4, ce_sym;
  always_ff @(posedge clk) begin
    if (rst) begin
      cnt8 <= '0; ph8 <= 1'b0; ph4 <= '0;
    end else begin
      cnt8 <= (cnt8 >= rate - 6'd1) ? 6'd0 : cnt8 + 6'd1;
      if (ce8) ph8 <= ~ph8;
      if (ce4) ph4 <= ph4 + 2'd1;
    end
  end
  assign ce8    = (cnt8 == 6'd0);
  assign ce4    = ce8 && !ph8;
  assign ce_sym = ce4 && (ph4 == 2'd0);

  logic signed [15:0] sym_i, sym_q;
  tx_framer u_framer (
    .clk(clk), .rst(rst), .enable(tx_en), .test_mode(test_mode), .mod16(tx_mod16),
    .test_len(test_len), .ifs(ifs),
    .s_tdata(s_axis_tx_tdata), .s_tvalid(s_axis_tx_tvalid), .s_tlast(s_axis_tx_tlast),
    .s_tready(s_axis_tx_tready),
    .ce_sym(ce_sym), .sym_i(sym_i), .sym_q(sym_q), .frames_sent(st_tx_frames)
  );

  // RRC x4 (zero-stuffed)
  logic               rrc_tv;
  logic signed [15:0] rrc_ti, rrc_tq, rrc_hi, rrc_hq;
  fir_cplx #(.NTAP(RRC_NTAP), .IW(16), .CW(18), .OW(16), .SHIFT(16), .DECIM(1), .COEF(RRC_TX_COEF)) u_rrc_tx (
    .clk(clk), .rst(rst), .in_vld(ce4),
    .in_i(ph4 == 2'd0 ? sym_i : 16'sd0), .in_q(ph4 == 2'd0 ? sym_q : 16'sd0),
    .out_vld(rrc_tv), .out_i(rrc_ti), .out_q(rrc_tq)
  );
  always_ff @(posedge clk) if (rrc_tv) begin rrc_hi <= rrc_ti; rrc_hq <= rrc_tq; end

  // CFIR x2 (zero-stuffed)
  logic               cf_tv;
  logic signed [15:0] cf_ti, cf_tq, cf_hi, cf_hq;
  fir_cplx #(.NTAP(CF_NTAP), .IW(16), .CW(18), .OW(16), .SHIFT(16), .DECIM(1), .COEF(CF_TX_COEF)) u_cf_tx (
    .clk(clk), .rst(rst), .in_vld(ce8),
    .in_i(!ph8 ? rrc_hi : 16'sd0), .in_q(!ph8 ? rrc_hq : 16'sd0),
    .out_vld(cf_tv), .out_i(cf_ti), .out_q(cf_tq)
  );
  always_ff @(posedge clk) if (cf_tv) begin cf_hi <= cf_ti; cf_hq <= cf_tq; end

  // CIC xR
  logic signed [17:0] cic_ti, cic_tq;
  cic_int #(.IW(16), .OW(18), .W(44)) u_cic_tx (
    .clk(clk), .rst(rst), .shift(tx_shift), .ce(ce8), .in_i(cf_hi), .in_q(cf_hq),
    .out_i(cic_ti), .out_q(cic_tq)
  );

  // gain (Q2.14) + saturate to 16 bit
  logic signed [34:0] gpi, gpq;
  logic signed [15:0] dac_i, dac_q;
  function automatic logic signed [15:0] sat16(input logic signed [34:0] a);
    if (a > 35'sd32767)  return 16'sd32767;
    if (a < -35'sd32768) return -16'sd32768;
    return a[15:0];
  endfunction
  always_ff @(posedge clk) begin
    gpi   <= cic_ti * $signed({1'b0, tx_gain});
    gpq   <= cic_tq * $signed({1'b0, tx_gain});
    dac_i <= tx_en ? sat16(gpi >>> 14) : 16'sd0;
    dac_q <= tx_en ? sat16(gpq >>> 14) : 16'sd0;
  end
  generate
    if (DAC_SPC == 2) begin : g_x2
      // x2 linear interpolation 256 -> 512 MSPS: (mid(n-1,n), x(n)) per clock.
      // Response cos^2(pi f / 512 MHz): image at 256 MHz +- 2.5 MHz is < -70 dBc, droop < 0.001 dB.
      logic signed [15:0] pv_i, pv_q, md_i, md_q, cu_i, cu_q;
      always_ff @(posedge clk) begin
        pv_i <= dac_i;
        pv_q <= dac_q;
        md_i <= 16'((17'(pv_i) + 17'(dac_i) + 17'sd1) >>> 1);
        md_q <= 16'((17'(pv_q) + 17'(dac_q) + 17'sd1) >>> 1);
        cu_i <= dac_i;
        cu_q <= dac_q;
      end
      assign m_axis_dac_tdata = {cu_q, cu_i, md_q, md_i};
    end else begin : g_x1
      assign m_axis_dac_tdata = {dac_q, dac_i};
    end
  endgenerate
  assign m_axis_dac_tvalid = 1'b1;

  // ======================= RX =======================
  assign s_axis_adci_tready = 1'b1;
  assign s_axis_adcq_tready = 1'b1;

  logic               rx_vld;
  logic signed [15:0] rx_i, rx_q;
  always_ff @(posedge clk) begin
    rx_vld <= rx_en & (dig_lpbk ? 1'b1 : s_axis_adci_tvalid);
    rx_i   <= dig_lpbk ? dac_i : $signed(s_axis_adci_tdata);
    rx_q   <= dig_lpbk ? dac_q : $signed(s_axis_adcq_tdata);
  end

  logic               cic_rv;
  logic signed [17:0] cic_ri, cic_rq;
  cic_dec #(.IW(16), .OW(18), .W(40)) u_cic_rx (
    .clk(clk), .rst(rst), .rate(rate), .shift(rx_shift),
    .in_vld(rx_vld), .in_i(rx_i), .in_q(rx_q),
    .out_vld(cic_rv), .out_i(cic_ri), .out_q(cic_rq)
  );

  logic               cf_rv;
  logic signed [17:0] cf_ri, cf_rq;
  fir_cplx #(.NTAP(CF_NTAP), .IW(18), .CW(18), .OW(18), .SHIFT(17), .DECIM(2), .COEF(CF_RX_COEF)) u_cf_rx (
    .clk(clk), .rst(rst), .in_vld(cic_rv), .in_i(cic_ri), .in_q(cic_rq),
    .out_vld(cf_rv), .out_i(cf_ri), .out_q(cf_rq)
  );

  logic               rrc_rv;
  logic signed [17:0] rrc_ri, rrc_rq;
  fir_cplx #(.NTAP(RRC_NTAP), .IW(18), .CW(18), .OW(18), .SHIFT(17), .DECIM(1), .COEF(RRC_RX_COEF)) u_rrc_rx (
    .clk(clk), .rst(rst), .in_vld(cf_rv), .in_i(cf_ri), .in_q(cf_rq),
    .out_vld(rrc_rv), .out_i(rrc_ri), .out_q(rrc_rq)
  );

  logic               agc_v;
  logic signed [15:0] agc_i, agc_q;
  agc #(.IW(18), .OW(16)) u_agc (
    .clk(clk), .rst(rst), .target(agc_target), .mu_sh(agc_mu_sh),
    .in_vld(rrc_rv), .in_i(rrc_ri), .in_q(rrc_rq),
    .out_vld(agc_v), .out_i(agc_i), .out_q(agc_q), .gain(agc_gain)
  );

  logic               ts_v;
  logic signed [15:0] ts_i, ts_q;
  timing_rec u_ted (
    .clk(clk), .rst(rst), .kp_sh(ted_kp_sh), .ki_sh(ted_ki_sh),
    .in_vld(agc_v), .in_i(agc_i), .in_q(agc_q),
    .sym_vld(ts_v), .sym_i(ts_i), .sym_q(ts_q), .loop_v(ted_v)
  );

  logic               commit, est_vld;
  logic signed [31:0] est_phase, est_freq;
  preamble_det u_det (
    .clk(clk), .rst(rst), .thr(det_thr),
    .in_vld(ts_v), .in_i(ts_i), .in_q(ts_q),
    .commit(commit), .est_vld(est_vld), .est_phase(est_phase), .est_freq(est_freq),
    .peak_m(st_peak_m), .peak_e(st_peak_e)
  );

  logic               sp_v, sp_tag, sp_dbit, df_busy, df_mod16;
  logic signed [15:0] sp_yi, sp_yq;
  logic signed [16:0] sp_ei, sp_eq;
  logic [3:0]         sp_bits;
  sym_proc u_sym (
    .clk(clk), .rst(rst),
    .pll_kp_sh(pll_kp_sh), .pll_ki_sh(pll_ki_sh), .pll_en(pll_en),
    .eq_en(eq_en), .eq_mu_sh(eq_mu_sh),
    .in_vld(ts_v), .in_i(ts_i), .in_q(ts_q),
    .commit(commit), .est_vld(est_vld), .est_phase(est_phase), .est_freq(est_freq),
    .busy(df_busy), .mod16(df_mod16),
    .out_vld(sp_v), .out_tag(sp_tag), .y_i(sp_yi), .y_q(sp_yq), .e_i(sp_ei), .e_q(sp_eq),
    .bits(sp_bits), .dbit(sp_dbit), .pll_freq(st_pll_freq), .pll_phase()
  );

  logic       ob_vld, ob_last;
  logic [7:0] ob_data;
  deframer u_deframer (
    .clk(clk), .rst(rst),
    .in_vld(sp_v), .in_tag(sp_tag), .in_bits(sp_bits), .in_dbit(sp_dbit), .e_i(sp_ei), .e_q(sp_eq),
    .busy(df_busy), .mod16(df_mod16),
    .ob_vld(ob_vld), .ob_data(ob_data), .ob_last(ob_last),
    .cnt_ok(st_rx_ok), .cnt_crc_err(st_rx_crc), .cnt_hdr_err(st_rx_hdr),
    .evm_acc(st_evm_acc), .evm_n(st_evm_n), .last_hdr(st_last_hdr)
  );

  logic fifo_full;
  sync_fifo #(.W(9), .AW(12)) u_rx_fifo (
    .clk(clk), .rst(rst), .wr(ob_vld), .wdata({ob_last, ob_data}), .full(fifo_full),
    .rd_vld(m_axis_rx_tvalid), .rd_data({m_axis_rx_tlast, m_axis_rx_tdata}), .rd_rdy(m_axis_rx_tready)
  );

  always_ff @(posedge clk) begin
    if (rst) begin
      st_rx_ovf  <= '0;
      st_det_cnt <= '0;
    end else begin
      if (ob_vld && fifo_full) st_rx_ovf <= st_rx_ovf + 32'd1;
      if (commit) st_det_cnt <= st_det_cnt + 32'd1;
    end
  end

endmodule
