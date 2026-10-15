// Preamble detector + data-aided carrier phase/frequency estimator.
// Operates on timing-recovered symbols (1 sps, before derotation).
//  * 64-symbol window, correlation split into 4 quarters of 16 (non-coherent metric tolerates
//    CFO up to ~Rs/32).
//  * m = sum_q |C_q/16|^2, E = sum |x|^2 over the window. Normalised metric
//    rho = 16*m/E (1.0 for a noise-free preamble). Detect when rho >= thr/256.
//  * Peak picking: candidate kept while metric increases; committed on first non-increase,
//    i.e. exactly one symbol after the true peak.
//  * On commit: w = arg(sum C_{q+1} C_q^*)/16 rad/sym; quarters aligned with w and summed
//    coherently; outputs the phase of preamble symbol 62 and w (see estimator below).
// Requires >= ~24 clocks between input symbols (guaranteed: symbol period = 8R >= 64).
module preamble_det
  import nb_coefs_pkg::*;
(
  input  logic               clk,
  input  logic               rst,
  input  logic [7:0]         thr,
  input  logic               in_vld,
  input  logic signed [15:0] in_i,
  input  logic signed [15:0] in_q,
  output logic               commit,       // pulse
  output logic               est_vld,      // pulse, ~50 cycles after commit
  output logic signed [31:0] est_phase,    // phase of preamble symbol 62 (2^32 = 2pi)
  output logic signed [31:0] est_freq,     // rad/symbol (2^32 = 2pi)
  output logic [31:0]        peak_m,       // m of last committed peak
  output logic [31:0]        peak_e        // E/16 of last committed peak  (rho = peak_m/peak_e)
);
  localparam int L = 64;
  logic signed [15:0] wi [L], wq [L];       // [0] newest
  logic signed [39:0] E;
  logic signed [21:0] ci [4], cq [4];
  logic [3:0]         j;
  logic [1:0]         q;
  logic [47:0]        m_acc;
  logic [43:0]        thr_e;
  logic [43:0]        sq_i, sq_q;
  logic               mag_v;

  typedef enum logic [2:0] {D_IDLE, D_ACC, D_MAG, D_MAG2, D_CMP} dst_t;
  dst_t dst;

  logic signed [21:0] cand_ci [4], cand_cq [4];
  logic [47:0]        cand_m;
  logic [39:0]        cand_e;
  logic               cand_v;

  function automatic logic signed [31:0] sqr(input logic signed [15:0] a);
    return a * a;
  endfunction

  // ---------------- window + energy ----------------
  logic signed [32:0] e_in, e_out;
  logic               e_v;
  always_ff @(posedge clk) begin
    if (rst) begin
      for (int k = 0; k < L; k++) begin wi[k] <= '0; wq[k] <= '0; end
      E   <= '0;
      e_v <= 1'b0;
    end else begin
      e_v <= in_vld;
      if (in_vld) begin
        wi[0] <= in_i; wq[0] <= in_q;
        for (int k = 1; k < L; k++) begin wi[k] <= wi[k-1]; wq[k] <= wq[k-1]; end
        e_in  <= 33'(sqr(in_i)) + 33'(sqr(in_q));
        e_out <= 33'(sqr(wi[L-1])) + 33'(sqr(wq[L-1]));
      end
      if (e_v) E <= E + 40'(e_in) - 40'(e_out);
    end
  end

  // ---------------- detection ----------------
  logic signed [21:0] ci_s, cq_s;
  always_comb begin
    ci_s = ci[q] >>> 4;
    cq_s = cq[q] >>> 4;
  end

  logic start_est;

  always_ff @(posedge clk) begin
    if (rst) begin
      dst       <= D_IDLE;
      cand_v    <= 1'b0;
      cand_m    <= '0;
      commit    <= 1'b0;
      start_est <= 1'b0;
      peak_m    <= '0;
      peak_e    <= '0;
    end else begin
      commit    <= 1'b0;
      start_est <= 1'b0;
      case (dst)
        D_IDLE: if (in_vld) begin
          for (int k = 0; k < 4; k++) begin ci[k] <= '0; cq[k] <= '0; end
          j   <= '0;
          dst <= D_ACC;
        end
        D_ACC: begin
          // preamble symbol n sits at window index 63-n
          for (int k = 0; k < 4; k++) begin
            if (PREAMBLE[16*k + int'(j)]) begin
              ci[k] <= ci[k] + 22'(wi[63 - 16*k - int'(j)]);
              cq[k] <= cq[k] + 22'(wq[63 - 16*k - int'(j)]);
            end else begin
              ci[k] <= ci[k] - 22'(wi[63 - 16*k - int'(j)]);
              cq[k] <= cq[k] - 22'(wq[63 - 16*k - int'(j)]);
            end
          end
          j    <= j + 4'd1;
          thr_e <= 44'(thr) * 44'($unsigned(E[39:8]));   // E is stable during ACC
          if (j == 4'd15) begin
            dst   <= D_MAG;
            q     <= '0;
            m_acc <= '0;
            mag_v <= 1'b0;
          end
        end
        D_MAG: begin
          // registered squares of quarter q, accumulated one cycle later
          sq_i  <= 44'(ci_s * ci_s);
          sq_q  <= 44'(cq_s * cq_s);
          if (mag_v) m_acc <= m_acc + 48'(sq_i) + 48'(sq_q);
          mag_v <= 1'b1;
          q     <= q + 2'd1;
          if (q == 2'd3) dst <= D_MAG2;
        end
        D_MAG2: begin
          m_acc <= m_acc + 48'(sq_i) + 48'(sq_q);
          dst   <= D_CMP;
        end
        D_CMP: begin
          logic above, higher;
          above  = ({m_acc, 4'b0} >= 52'(thr_e)) && (E > 40'sd16777216);   // m*4096 >= thr*E, and signal present
          higher = (m_acc > cand_m);
          dst <= D_IDLE;
          if (above && (!cand_v || higher)) begin
            cand_v <= 1'b1;
            cand_m <= m_acc;
            cand_e <= E;
            for (int k = 0; k < 4; k++) begin cand_ci[k] <= ci[k]; cand_cq[k] <= cq[k]; end
          end else if (cand_v) begin
            cand_v    <= 1'b0;
            cand_m    <= '0;
            commit    <= 1'b1;
            start_est <= 1'b1;
            peak_m    <= cand_m[31:0];
            peak_e    <= cand_e[35:4];
          end
        end
        default: dst <= D_IDLE;
      endcase
    end
  end

  // ---------------- estimation (runs concurrently with detection) ----------------
  // w   = arg(sum_k C_{k+1} C_k^*) / 16                      (valid for |16w| < pi)
  // u   = exp(-j*16w)  (Q2.16)                                (one CORDIC rotation)
  // S   = C0*u^* + C1 + C2*u + C3*u^2   -> all quarters aligned to the centre of quarter 1
  //       (symbol 23.5); robust even when the quarters span more than 180 degrees.
  // phi62 = arg(S) - pi/4 + 38.5*w
  typedef enum logic [3:0] {E_IDLE, E_FQ, E_FQ2, E_FS, E_FW, E_UW, E_U2A, E_U2B, E_S, E_SL, E_PS, E_PW} est_t;
  est_t est;
  logic signed [21:0] ei [4], eq [4];     // snapshot of committed candidate
  logic signed [47:0] zi, zq;
  logic signed [31:0] w_est;
  logic [1:0]         ek;
  logic               cs, cmode, cdone;
  logic signed [23:0] cx, cy, cxo, cyo;
  logic signed [31:0] cz, czo;
  logic signed [17:0] ui, uq, u2i, u2q;
  logic               sv;
  logic signed [27:0] si, sq;
  logic signed [47:0] p0, p1, p2, p3;

  cordic #(.DW(24)) u_cordic (
    .clk(clk), .rst(rst), .start(cs), .mode(cmode),
    .x_in(cx), .y_in(cy), .z_in(cz),
    .done(cdone), .x_out(cxo), .y_out(cyo), .z_out(czo)
  );

  always_ff @(posedge clk) begin
    if (rst) begin
      est     <= E_IDLE;
      est_vld <= 1'b0;
      cs      <= 1'b0;
      cmode   <= 1'b1;
    end else begin
      est_vld <= 1'b0;
      cs      <= 1'b0;
      case (est)
        E_IDLE: if (start_est) begin
          for (int k = 0; k < 4; k++) begin ei[k] <= cand_ci[k]; eq[k] <= cand_cq[k]; end
          zi  <= '0; zq <= '0;
          ek  <= '0;
          est <= E_FQ;
        end
        E_FQ: begin
          // Z = sum_{k=0..2} C_{k+1} * conj(C_k)  (on C>>4): products registered, summed next cycle
          logic signed [21:0] ai, aq, bi, bq;
          ai = ei[ek+1] >>> 4; aq = eq[ek+1] >>> 4;
          bi = ei[ek]   >>> 4; bq = eq[ek]   >>> 4;
          p0 <= 48'(ai * bi); p1 <= 48'(aq * bq); p2 <= 48'(aq * bi); p3 <= 48'(ai * bq);
          if (ek != 2'd0) begin
            zi <= zi + p0 + p1;
            zq <= zq + p2 - p3;
          end
          ek <= ek + 2'd1;
          if (ek == 2'd2) est <= E_FQ2;
        end
        E_FQ2: begin
          zi  <= zi + p0 + p1;
          zq  <= zq + p2 - p3;
          est <= E_FS;
        end
        E_FS: begin
          cx    <= 24'(zi >>> 12);
          cy    <= 24'(zq >>> 12);
          cz    <= '0;
          cmode <= 1'b1;
          cs    <= 1'b1;
          est   <= E_FW;
        end
        E_FW: if (cdone) begin
          w_est <= czo >>> 4;
          // u = exp(-j*16w): rotate (2^16, 0) by -arg(Z)
          cx    <= 24'sd65536;
          cy    <= '0;
          cz    <= -czo;
          cmode <= 1'b0;
          cs    <= 1'b1;
          est   <= E_UW;
        end
        E_UW: if (cdone) begin
          ui  <= 18'(cxo);
          uq  <= 18'(cyo);
          est <= E_U2A;
        end
        E_U2A: begin   // u^2, products registered
          p0 <= 36'(ui * ui); p1 <= 36'(uq * uq); p2 <= 36'(ui * uq);
          est <= E_U2B;
        end
        E_U2B: begin
          u2i <= 18'((p0 - p1) >>> 16);
          u2q <= 18'((p2 + p2) >>> 16);
          si  <= '0;
          sq  <= '0;
          ek  <= '0;
          sv  <= 1'b0;
          est <= E_S;
        end
        // S = sum_j C_j * m_j with m = {conj(u), 1, u, u^2}; 2-stage pipeline (products | sum)
        E_S: begin
          logic signed [17:0] a, b, c, d;
          a = 18'(ei[ek] >>> 4);
          b = 18'(eq[ek] >>> 4);
          case (ek)
            2'd0:    begin c = ui;           d = -uq;  end
            2'd1:    begin c = 18'sd65536;   d = '0;   end
            2'd2:    begin c = ui;           d = uq;   end
            default: begin c = u2i;          d = u2q;  end
          endcase
          p0 <= 36'(a * c); p1 <= 36'(b * d); p2 <= 36'(a * d); p3 <= 36'(b * c);
          if (sv) begin
            si <= si + 28'((p0 - p1) >>> 16);
            sq <= sq + 28'((p2 + p3) >>> 16);
          end
          sv <= 1'b1;
          ek <= ek + 2'd1;
          if (ek == 2'd3) est <= E_SL;
        end
        E_SL: begin
          si  <= si + 28'((p0 - p1) >>> 16);
          sq  <= sq + 28'((p2 + p3) >>> 16);
          est <= E_PS;
        end
        E_PS: begin
          cx    <= 24'(si);
          cy    <= 24'(sq);
          cz    <= '0;
          cmode <= 1'b1;
          cs    <= 1'b1;
          est   <= E_PW;
        end
        E_PW: if (cdone) begin
          // phi62 = arg(S) - pi/4 + 38.5*w   (modulo 2^32; never halve a wrapped value)
          est_phase <= czo - 32'sh2000_0000 + (w_est <<< 5) + (w_est <<< 2) + (w_est <<< 1) + (w_est >>> 1);
          est_freq  <= w_est;
          est_vld   <= 1'b1;
          est       <= E_IDLE;
        end
        default: est <= E_IDLE;
      endcase
    end
  end
`ifndef SYNTHESIS
  always @(posedge clk) if (!rst && in_vld && dst != D_IDLE) $error("preamble_det overrun: new input before processing finished");
`endif
endmodule
