// Symbol-rate back end: alignment delay line -> derotator (pipelined CORDIC) -> 5-tap complex
// DD-LMS equalizer -> slicer -> decision-directed 2nd-order PLL.
//
// Streaming design for symbol periods down to 16 clocks (20 MHz channel, R = 2):
//   A: 12-deep alignment line; owns the phase accumulator phi (advances by frq per symbol) and
//      issues each symbol to the pipelined CORDIC (II = 1, latency 23).
//   B: fixed-latency pipeline (11 clocks): parallel EQ MAC, slicer, phase detector, PLL filter,
//      parallel LMS tap update + phase anchoring. Its recursion (taps, frq) closes before the
//      next symbol arrives. Corrections to phi are posted to A (PLL loop delay ~2-3 symbols).
//
// Data-aided acquisition: on a preamble commit the delay-line entry holding preamble symbol 62
// is tagged; when it is popped, phase/frequency are loaded from the estimator (which has ~10
// symbol periods to finish). The tag travels with the symbol; at the equalizer output (centre
// tap = 2 symbols latency) the deframer uses it as the frame reference.
module sym_proc
  import nb_pkg::*;
(
  input  logic               clk,
  input  logic               rst,
  // config
  input  logic [4:0]         pll_kp_sh,
  input  logic [4:0]         pll_ki_sh,
  input  logic               pll_en,
  input  logic               eq_en,
  input  logic [4:0]         eq_mu_sh,
  // symbols from timing recovery
  input  logic               in_vld,
  input  logic signed [15:0] in_i,
  input  logic signed [15:0] in_q,
  // from preamble detector
  input  logic               commit,
  input  logic               est_vld,
  input  logic signed [31:0] est_phase,
  input  logic signed [31:0] est_freq,
  // from deframer
  input  logic               busy,        // frame in progress: ignore commits
  input  logic               mod16,       // slicer constellation for the next output symbol
  // output
  output logic               out_vld,
  output logic               out_tag,
  output logic signed [15:0] y_i, y_q,
  output logic signed [16:0] e_i, e_q,    // d - y
  output logic [3:0]         bits,        // QPSK: bits[1:0]
  output logic               dbit,        // diagonal BPSK decision (header): yi+yq >= 0
  output logic signed [31:0] pll_freq,
  output logic signed [31:0] pll_phase
);
  localparam int D   = 12;     // alignment line depth (estimator budget ~ (D-2) symbols)
  localparam int NT  = 5;
  localparam int CT  = 2;

  function automatic logic signed [15:0] sat16(input logic signed [39:0] a);
    if (a > 40'sd32767)  return 16'sd32767;
    if (a < -40'sd32768) return -16'sd32768;
    return a[15:0];
  endfunction

  function automatic logic signed [31:0] sadd32(input logic signed [31:0] a, input logic signed [31:0] b);
    logic signed [32:0] s;
    s = 33'(a) + 33'(b);
    if (s > 33'sh0_7FFF_FFFF)  return 32'sh7FFF_FFFF;
    if (s < -33'sh0_8000_0000) return 32'sh8000_0000;
    return s[31:0];
  endfunction

  // 16-QAM per-axis slicer: returns {bits[1:0], level}
  function automatic logic [17:0] qam_slice(input logic signed [15:0] v);
    if (v >= 16'(2*D1_QAM))       return {2'b10, 16'(3*D1_QAM)};
    else if (v >= 0)              return {2'b11, 16'(D1_QAM)};
    else if (v >= -16'(2*D1_QAM)) return {2'b01, -16'(D1_QAM)};
    else                          return {2'b00, -16'(3*D1_QAM)};
  endfunction

  // ======================= state =======================
  logic signed [15:0] dli [D], dlq [D];
  logic               dlt [D];
  logic signed [15:0] pi_, pq_;
  logic               ptag, a_go;

  logic signed [31:0] phi, frq;
  logic signed [31:0] pend_phi, pend_f;
  logic               pend_v;
  logic [2:0]         hold;
  logic signed [31:0] dphi;
  logic               dphi_v;

  logic signed [15:0] xei [NT], xeq [NT];
  logic               xet [NT];
  logic signed [31:0] wai [NT], waq [NT];       // tap accumulators, 1.0 = 2^28

  // ======================= stage A =======================
  logic               init_now;
  logic signed [31:0] phi_sym, f_sym;
  assign init_now = a_go && ptag && pend_v;
  assign phi_sym  = init_now ? pend_phi : phi;
  assign f_sym    = init_now ? pend_f   : frq;

  logic               cv, ctag;
  logic signed [23:0] cxo, cyo;
  cordic_pipe #(.DW(24)) u_rot (
    .clk(clk), .rst(rst), .in_vld(a_go), .in_tag(ptag),
    .x_in(24'(pi_)), .y_in(24'(pq_)), .z_in(-phi_sym),
    .out_vld(cv), .out_tag(ctag), .x_out(cxo), .y_out(cyo)
  );

  // ======================= stage B pipeline signals =======================
  logic [10:1]        bv;   // bv[k]: stage-k registers hold the current symbol
  logic signed [33:0] pii [NT], pqq [NT], piq [NT], pqi [NT];
  logic signed [35:0] yki [NT], ykq [NT];
  logic signed [38:0] s01i, s01q, s23i, s23q, s4i, s4q;
  logic signed [39:0] acc_i, acc_q;
  logic signed [15:0] yi, yq, di, dq;
  logic [3:0]         bt;
  logic signed [33:0] pe0, pe1;
  logic signed [31:0] inc_pr, inc_ir;
  logic signed [17:0] th16;
  logic signed [31:0] thw;
  logic signed [33:0] g0 [NT], g1 [NT], g2 [NT], g3 [NT];
  logic signed [35:0] ar [NT], br [NT];
  logic signed [31:0] dwi [NT], dwq [NT];
  logic               upd8, upd9, pll8, pll9;

  always_ff @(posedge clk) begin
    if (rst) begin
      for (int n = 0; n < D; n++) begin dli[n] <= '0; dlq[n] <= '0; dlt[n] <= 1'b0; end
      for (int n = 0; n < NT; n++) begin
        xei[n] <= '0; xeq[n] <= '0; xet[n] <= 1'b0;
        wai[n] <= (n == CT) ? 32'sd1 <<< 28 : '0;
        waq[n] <= '0;
      end
      phi     <= '0;
      frq     <= '0;
      pend_v  <= 1'b0;
      hold    <= '0;
      a_go    <= 1'b0;
      bv      <= '0;
      out_vld <= 1'b0;
      dphi_v  <= 1'b0;
    end else begin
      a_go    <= 1'b0;
      out_vld <= 1'b0;
      dphi_v  <= 1'b0;
      bv      <= {bv[9:1], cv};

      // ---------------- stage A ----------------
      if (commit && !busy) begin           // tag preamble symbol 62 (index 2 after hdr0 push)
        if (in_vld) dlt[3] <= 1'b1;
        else        dlt[2] <= 1'b1;
      end
      if (est_vld) begin
        pend_phi <= est_phase;
        pend_f   <= est_freq;
        pend_v   <= 1'b1;
      end
      if (in_vld) begin
        dli[0] <= in_i; dlq[0] <= in_q; dlt[0] <= 1'b0;
        for (int n = 1; n < D; n++) begin
          dli[n] <= dli[n-1]; dlq[n] <= dlq[n-1];
          if (!(commit && !busy && n == 3)) dlt[n] <= dlt[n-1];
        end
        pi_  <= dli[D-1];
        pq_  <= dlq[D-1];
        ptag <= dlt[D-1];
        a_go <= 1'b1;
      end
      begin
        logic signed [31:0] phn;
        phn = a_go ? phi_sym + f_sym : phi;        // advance to the next symbol's phase
        if (dphi_v && !init_now) phn = phn + dphi;  // posted correction from stage B
        phi <= phn;
      end
      if (init_now) begin
        frq    <= pend_f;
        pend_v <= 1'b0;
        hold   <= 3'd5;
      end

      // ---------------- stage B ----------------
      // b0: equalizer delay line
      if (cv) begin
        xei[0] <= sat16(40'(cxo)); xeq[0] <= sat16(40'(cyo)); xet[0] <= ctag;
        for (int n = 1; n < NT; n++) begin xei[n] <= xei[n-1]; xeq[n] <= xeq[n-1]; xet[n] <= xet[n-1]; end
      end
      // b1: tap products (registered DSPs)
      for (int n = 0; n < NT; n++) begin
        logic signed [17:0] wi, wq;
        wi = 18'(wai[n] >>> 14);
        wq = 18'(waq[n] >>> 14);
        pii[n] <= 34'(wi * xei[n]); pqq[n] <= 34'(wq * xeq[n]);
        piq[n] <= 34'(wi * xeq[n]); pqi[n] <= 34'(wq * xei[n]);
      end
      // b2: complex products
      for (int n = 0; n < NT; n++) begin
        yki[n] <= 36'(pii[n]) - 36'(pqq[n]);
        ykq[n] <= 36'(piq[n]) + 36'(pqi[n]);
      end
      // b3, b4: sum
      s01i <= 39'(yki[0]) + 39'(yki[1]); s01q <= 39'(ykq[0]) + 39'(ykq[1]);
      s23i <= 39'(yki[2]) + 39'(yki[3]); s23q <= 39'(ykq[2]) + 39'(ykq[3]);
      s4i  <= 39'(yki[4]);               s4q  <= 39'(ykq[4]);
      acc_i <= 40'(s01i) + 40'(s23i) + 40'(s4i);
      acc_q <= 40'(s01q) + 40'(s23q) + 40'(s4q);
      // b5: y
      yi <= sat16(acc_i >>> 14);
      yq <= sat16(acc_q >>> 14);
      // b6: slicer (y valid at bv[6])
      if (bv[6]) begin
        if (mod16) begin
          logic [17:0] si, sq;
          si = qam_slice(yi);
          sq = qam_slice(yq);
          di <= si[15:0];
          dq <= sq[15:0];
          bt <= {si[17:16], sq[17:16]};
        end else begin
          di <= yi[15] ? -16'sd8192 : 16'sd8192;
          dq <= yq[15] ? -16'sd8192 : 16'sd8192;
          bt <= {2'b00, ~yi[15], ~yq[15]};
        end
      end
      // b7: phase-detector products, error, outputs, anchoring angle
      if (bv[7]) begin
        pe0     <= 34'(yq * di);                     // Im{y d^*} = yq*di - yi*dq
        pe1     <= 34'(yi * dq);
        out_vld <= 1'b1;
        out_tag <= xet[CT];
        y_i     <= yi;
        y_q     <= yq;
        e_i     <= 17'(di) - 17'(yi);
        e_q     <= 17'(dq) - 17'(yq);
        bits    <= bt;
        dbit    <= ($signed(17'(yi) + 17'(yq)) >= 0);
        th16    <= 18'(waq[CT] >>> 12);                                     // theta * 2^16
        thw     <= (waq[CT] <<< 1) + (waq[CT] >>> 1) + (waq[CT] >>> 4);     // theta*2^32/2pi
        upd8    <= eq_en && (hold == 0);
        pll8    <= pll_en && (hold == 0);
        if (hold != 0) hold <= hold - 3'd1;
      end
      // b8: PLL increments; LMS + anchoring products (all taps in parallel)
      begin
        logic signed [31:0] pe_s;
        pe_s   = 32'((pe0 - pe1) >>> 16);
        inc_pr <= (pe_s <<< 16) >>> pll_kp_sh;
        inc_ir <= (pe_s <<< 16) >>> pll_ki_sh;
      end
      for (int n = 0; n < NT; n++) begin
        g0[n] <= 34'(e_i * xei[n]); g1[n] <= 34'(e_q * xeq[n]);
        g2[n] <= 34'(e_q * xei[n]); g3[n] <= 34'(e_i * xeq[n]);
        ar[n] <= 36'(18'(wai[n] >>> 14) * th16);
        br[n] <= 36'(18'(waq[n] >>> 14) * th16);
      end
      upd9 <= upd8;
      pll9 <= pll8;
      // b9: tap increments = mu * e * conj(x) + w * (-j*theta)
      for (int n = 0; n < NT; n++) begin
        dwi[n] <= 32'((g0[n] + g1[n]) >>> eq_mu_sh) + 32'(br[n] >>> 2);
        dwq[n] <= 32'((g2[n] - g3[n]) >>> eq_mu_sh) - 32'(ar[n] >>> 2);
      end
      // b10: commit taps, frequency and phase corrections
      if (bv[10]) begin
        if (upd9) begin
          for (int n = 0; n < NT; n++) begin
            wai[n] <= sadd32(wai[n], dwi[n]);
            waq[n] <= sadd32(waq[n], dwq[n]);
          end
        end
        if (pll9 || upd9) begin
          dphi   <= (pll9 ? inc_ir + inc_pr : 32'sd0) - (upd9 ? thw : 32'sd0);
          dphi_v <= 1'b1;
        end
        if (pll9 && !init_now) frq <= frq + inc_ir;
      end
      if (!eq_en) begin
        for (int n = 0; n < NT; n++) begin
          wai[n] <= (n == CT) ? 32'sd1 <<< 28 : '0;
          waq[n] <= '0;
        end
      end
    end
  end

  assign pll_freq  = frq;
  assign pll_phase = phi;

`ifndef SYNTHESIS
  // stage B must finish (b10) before the next symbol reaches b1
  always @(posedge clk) if (!rst && cv && |bv[9:1])
    $error("sym_proc overrun: symbols closer than the 11-clock stage-B pipeline");
`endif
endmodule
