// Symbol-rate back end: alignment delay line -> derotator (CORDIC) -> 5-tap complex DD-LMS
// equalizer -> slicer -> decision-directed 2nd-order PLL.
//
// Two concurrent stages (each ~27 clocks, symbol period >= 64 clocks):
//   A: delay line + CORDIC derotation; owns the phase accumulator phi (advances by frq/symbol)
//   B: equalizer MAC, slicer, phase detector, PLL filter, LMS tap update, phase anchoring;
//      posts phase corrections to A (one extra symbol of PLL loop delay - negligible).
//
// Data-aided acquisition: on a preamble commit the delay-line entry holding preamble symbol 62
// is tagged; when it reaches the derotator, phase/frequency are loaded from the estimator.
// The tag travels with the symbol; at the equalizer output (centre tap = 2 symbols latency)
// the deframer uses it as the frame reference (header starts 2 symbols after the tag).
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
  localparam int D   = 4;
  localparam int NT  = 5;
  localparam int CT  = 2;

  // ---------------- alignment delay line ----------------
  logic signed [15:0] dli [D], dlq [D];
  logic               dlt [D];
  logic signed [15:0] pi_, pq_;
  logic               ptag;

  // ---------------- PLL state ----------------
  logic signed [31:0] phi, frq;
  logic signed [31:0] pend_phi, pend_f;
  logic               pend_v;
  logic [2:0]         hold;

  // ---------------- equalizer ----------------
  logic signed [15:0] xei [NT], xeq [NT];
  logic               xet [NT];
  logic signed [31:0] wai [NT], waq [NT];       // tap accumulators, 1.0 = 2^28
  logic signed [39:0] acc_i, acc_q;
  logic [2:0]         k;
  logic signed [15:0] yi, yq, di, dq;
  logic [3:0]         bt;
  logic signed [17:0] th16;
  logic signed [33:0] m0, m1, m2, m3;
  logic signed [31:0] inc_pr, inc_ir, gr_r, gi_r;
  logic signed [35:0] a_r, b_r;

  // ---------------- CORDIC ----------------
  logic               cs, cdone;
  logic signed [23:0] cxo, cyo;
  logic signed [31:0] cz;

  cordic #(.DW(24)) u_rot (
    .clk(clk), .rst(rst), .start(cs), .mode(1'b0),
    .x_in(24'(pi_)), .y_in(24'(pq_)), .z_in(cz),
    .done(cdone), .x_out(cxo), .y_out(cyo), .z_out()
  );

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

  logic signed [17:0] wi_k, wq_k;
  assign wi_k = 18'(wai[k] >>> 14);
  assign wq_k = 18'(waq[k] >>> 14);

  logic eq_upd;
  assign eq_upd = eq_en && (hold == 0);

  typedef enum logic [1:0] {A_IDLE, A_INIT, A_ROT, A_HAND} ast_t;
  typedef enum logic [3:0] {B_IDLE, B_EQ, B_SL, B_DEC, B_PE, B_PE2, B_UPD, B_WU, B_ANC0, B_ANC} bst_t;
  ast_t sa;
  bst_t sb;
  logic               xr_v;                    // derotated symbol waiting for stage B
  logic signed [15:0] xr_i, xr_q;
  logic               xr_t;
  logic signed [31:0] dphi;                    // correction posted by B (applied next cycle)
  logic               dphi_v;

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
      sa      <= A_IDLE;
      sb      <= B_IDLE;
      cs      <= 1'b0;
      xr_v    <= 1'b0;
      out_vld <= 1'b0;
      dphi_v  <= 1'b0;
    end else begin
      logic signed [31:0] phi_n;
      cs      <= 1'b0;
      out_vld <= 1'b0;
      dphi_v  <= 1'b0;
      phi_n   = phi;

      // tag request from a preamble commit (not during a frame)
      if (commit && !busy) begin
        if (in_vld) dlt[3] <= 1'b1;   // shift happening this cycle
        else        dlt[2] <= 1'b1;
      end
      if (est_vld) begin
        pend_phi <= est_phase;
        pend_f   <= est_freq;
        pend_v   <= 1'b1;
      end
      if (!eq_en) begin
        for (int n = 0; n < NT; n++) begin
          wai[n] <= (n == CT) ? 32'sd1 <<< 28 : '0;
          waq[n] <= '0;
        end
      end

      // ======================= stage A =======================
      if (in_vld) begin
        dli[0] <= in_i; dlq[0] <= in_q; dlt[0] <= 1'b0;
        for (int n = 1; n < D; n++) begin
          dli[n] <= dli[n-1]; dlq[n] <= dlq[n-1];
          if (!(commit && !busy && n == 3)) dlt[n] <= dlt[n-1];
        end
        pi_  <= dli[D-1];
        pq_  <= dlq[D-1];
        ptag <= dlt[D-1];
      end
      case (sa)
        A_IDLE: if (in_vld) sa <= A_INIT;
        A_INIT: begin
          if (ptag && pend_v) begin
            // data-aided acquisition: phase/frequency estimate valid for this symbol
            cz     <= -pend_phi;
            phi_n  = pend_phi;
            frq    <= pend_f;
            pend_v <= 1'b0;
            hold   <= 3'd5;
          end else begin
            cz     <= -phi;
          end
          cs <= 1'b1;
          sa <= A_ROT;
        end
        A_ROT: if (cdone) begin
          xr_i  <= sat16(40'(cxo));
          xr_q  <= sat16(40'(cyo));
          xr_t  <= ptag;
          phi_n = phi_n + frq;              // predicted phase of the next symbol
          sa    <= A_HAND;
        end
        A_HAND: if (!xr_v) begin
          xr_v <= 1'b1;
          sa   <= A_IDLE;
        end
        default: sa <= A_IDLE;
      endcase
      if (dphi_v && !(sa == A_INIT && ptag && pend_v)) phi_n = phi_n + dphi;
      phi <= phi_n;

      // ======================= stage B =======================
      case (sb)
        B_IDLE: if (xr_v) begin
          xr_v   <= 1'b0;
          xei[0] <= xr_i; xeq[0] <= xr_q; xet[0] <= xr_t;
          for (int n = 1; n < NT; n++) begin
            xei[n] <= xei[n-1]; xeq[n] <= xeq[n-1]; xet[n] <= xet[n-1];
          end
          acc_i <= '0;
          acc_q <= '0;
          k     <= '0;
          sb    <= B_EQ;
        end
        B_EQ: begin
          // y += w_k * x_k : products registered at k, accumulated at k+1
          if (k < 3'(NT)) begin
            m0 <= 34'(wi_k * xei[k]); m1 <= 34'(wq_k * xeq[k]);
            m2 <= 34'(wi_k * xeq[k]); m3 <= 34'(wq_k * xei[k]);
          end
          if (k != 0) begin
            acc_i <= acc_i + 40'(m0) - 40'(m1);
            acc_q <= acc_q + 40'(m2) + 40'(m3);
          end
          k <= k + 3'd1;
          if (k == 3'(NT)) sb <= B_SL;
        end
        B_SL: begin
          yi <= sat16(acc_i >>> 14);
          yq <= sat16(acc_q >>> 14);
          sb <= B_DEC;
        end
        B_DEC: begin
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
          sb <= B_PE;
        end
        B_PE: begin
          m0 <= 34'(yq * di);                         // Im{y d^*} = yq*di - yi*dq
          m1 <= 34'(yi * dq);
          sb <= B_PE2;
        end
        B_PE2: begin
          logic signed [31:0] pe_s;
          pe_s   = 32'((m0 - m1) >>> 16);
          inc_pr <= (pe_s <<< 16) >>> pll_kp_sh;
          inc_ir <= (pe_s <<< 16) >>> pll_ki_sh;
          sb <= B_UPD;
        end
        B_UPD: begin
          if (pll_en && hold == 0) begin
            frq    <= frq + inc_ir;
            dphi   <= inc_ir + inc_pr;
            dphi_v <= 1'b1;
          end
          // outputs (symbol at the equalizer centre)
          out_vld <= 1'b1;
          out_tag <= xet[CT];
          y_i     <= yi;
          y_q     <= yq;
          e_i     <= 17'(di) - 17'(yi);
          e_q     <= 17'(dq) - 17'(yq);
          bits    <= bt;
          dbit    <= ($signed(17'(yi) + 17'(yq)) >= 0);
          k       <= '0;
          sb      <= eq_upd ? B_WU : B_IDLE;
          if (hold != 0) hold <= hold - 3'd1;
        end
        B_WU: begin
          // w_k += mu * e * conj(x_k): products at k, gradient at k+1, tap update at k+2
          if (k < 3'(NT)) begin
            m0 <= 34'(e_i * xei[k]); m1 <= 34'(e_q * xeq[k]);
            m2 <= 34'(e_q * xei[k]); m3 <= 34'(e_i * xeq[k]);
          end
          if (k >= 3'd1 && k <= 3'(NT)) begin
            gr_r <= 32'((m0 + m1) >>> eq_mu_sh);
            gi_r <= 32'((m2 - m3) >>> eq_mu_sh);
          end
          if (k >= 3'd2) begin
            wai[k-2] <= sadd32(wai[k-2], gr_r);
            waq[k-2] <= sadd32(waq[k-2], gi_r);
          end
          k <= k + 3'd1;
          if (k == 3'(NT+1)) sb <= B_ANC0;
        end
        // Phase anchoring: move the centre-tap angle theta ~= Im(w_c)/Re(w_c) into the PLL and
        // counter-rotate all taps. Keeps the taps phase-neutral so the k*90deg DD ambiguity lives
        // only in the PLL phase, which the preamble estimate overwrites.
        B_ANC0: begin
          th16   <= 18'(waq[CT] >>> 12);                                    // theta * 2^16
          dphi   <= -((waq[CT] <<< 1) + (waq[CT] >>> 1) + (waq[CT] >>> 4)); // theta*2^32/2pi
          dphi_v <= 1'b1;
          k      <= '0;
          sb     <= B_ANC;
        end
        B_ANC: begin
          // w_k <- w_k * (1 - j*theta): products at k (on w>>>14, one DSP), update at k+1
          if (k < 3'(NT)) begin
            a_r <= 36'(18'(wai[k] >>> 14) * th16);
            b_r <= 36'(18'(waq[k] >>> 14) * th16);
          end
          if (k != 0) begin
            wai[k-1] <= sadd32(wai[k-1], 32'(b_r >>> 2));
            waq[k-1] <= sadd32(waq[k-1], -32'(a_r >>> 2));
          end
          k <= k + 3'd1;
          if (k == 3'(NT)) sb <= B_IDLE;
        end
        default: sb <= B_IDLE;
      endcase
    end
  end

  assign pll_freq  = frq;
  assign pll_phase = phi;

`ifndef SYNTHESIS
  always @(posedge clk) if (!rst && in_vld && sa != A_IDLE)
    $error("sym_proc overrun: new input before stage A finished");
  always @(posedge clk) if (!rst && sa == A_HAND && cdone)
    $error("sym_proc: stage B did not consume previous symbol in time");
`endif
endmodule
