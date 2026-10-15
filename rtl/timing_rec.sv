// Symbol timing recovery: cubic Lagrange Farrow interpolator + Gardner TED + PI loop
// filter + modulo-1 decrementing NCO (Gardner/Rice structure).
// Input: 4 samples/symbol (strobed). Interpolants at 2 samples/symbol. Output: 1 sample/symbol.
// eta: unsigned, 1.0 = 2^24. W nominal = 0.5 (2 input samples per interpolant).
module timing_rec (
  input  logic               clk,
  input  logic               rst,
  input  logic [4:0]         kp_sh,
  input  logic [4:0]         ki_sh,
  input  logic               in_vld,
  input  logic signed [15:0] in_i,
  input  logic signed [15:0] in_q,
  output logic               sym_vld,
  output logic signed [15:0] sym_i,
  output logic signed [15:0] sym_q,
  output logic signed [31:0] loop_v      // NCO control word (debug / status)
);
  localparam logic [25:0] ONE  = 26'd1 << 24;
  localparam logic [25:0] HALF = 26'd1 << 23;

  logic signed [15:0] xi [4], xq [4];     // [0] newest
  logic [25:0]        eta;
  logic signed [31:0] vacc, v;
  logic [16:0]        mu;                 // Q0.16 (0..65535)

  // Farrow FSM
  typedef enum logic [3:0] {F_IDLE, F_C, F_M3, F_A3, F_M2, F_A2, F_M1, F_A1, F_MY, F_Y, F_TED, F_LF1, F_LF2} fst_t;
  fst_t fst;
  logic signed [19:0] c1i, c2i, c3i, c1q, c2q, c3q;   // 6*c
  logic signed [15:0] c0i, c0q;
  logic signed [23:0] ti, tq;
  logic signed [15:0] yi, yq;

  // Gardner state
  logic               ontime;
  logic signed [15:0] mid_i, mid_q, prev_i, prev_q;
  logic signed [33:0] ted_e;
  logic signed [32:0] ted_pi, ted_pq;
  logic signed [23:0] pmi, pmq;
  logic signed [31:0] inc_p;

  function automatic logic signed [15:0] sat16(input logic signed [31:0] a);
    if (a > 32'sd32767)  return 16'sd32767;
    if (a < -32'sd32768) return -16'sd32768;
    return a[15:0];
  endfunction

  // t*mu >>> 16 (mu unsigned Q0.16)
  function automatic logic signed [23:0] mulmu(input logic signed [23:0] t, input logic [16:0] m);
    logic signed [41:0] p;
    p = t * $signed({1'b0, m});
    return 24'(p >>> 16);
  endfunction

  always_ff @(posedge clk) begin
    if (rst) begin
      eta     <= ONE >> 1;
      vacc    <= '0;
      v       <= '0;
      fst     <= F_IDLE;
      sym_vld <= 1'b0;
      ontime  <= 1'b0;
      for (int k = 0; k < 4; k++) begin xi[k] <= '0; xq[k] <= '0; end
      mid_i <= '0; mid_q <= '0; prev_i <= '0; prev_q <= '0;
    end else begin
      sym_vld <= 1'b0;

      if (in_vld) begin
        logic signed [26:0] en;
        logic [25:0] w;
        w  = 26'(HALF + 26'(v));
        en = $signed({1'b0, eta}) - $signed({1'b0, w});
        xi[0] <= in_i; xq[0] <= in_q;
        for (int k = 1; k < 4; k++) begin xi[k] <= xi[k-1]; xq[k] <= xq[k-1]; end
        if (en < 0) begin
          // underflow -> strobe. mu = eta/W ~= 2*eta
          logic [26:0] m2;
          m2  = {eta, 1'b0};
          mu  <= (m2 >= 27'(ONE)) ? 17'h0FFFF : {1'b0, m2[23:8]};
          eta <= 26'(en + $signed({1'b0, ONE}));
          fst <= F_C;
        end else begin
          eta <= 26'(en);
        end
      end

      case (fst)
        F_C: begin
          // samples after the shift: s_{-1}=x[3], s0=x[2], s1=x[1], s2=x[0]
          c0i <= xi[2];
          c0q <= xq[2];
          c1i <= -20'(2*xi[3]) - 20'(3*xi[2]) + 20'(6*xi[1]) - 20'(xi[0]);
          c1q <= -20'(2*xq[3]) - 20'(3*xq[2]) + 20'(6*xq[1]) - 20'(xq[0]);
          c2i <=  20'(3*xi[3]) - 20'(6*xi[2]) + 20'(3*xi[1]);
          c2q <=  20'(3*xq[3]) - 20'(6*xq[2]) + 20'(3*xq[1]);
          c3i <= -20'(xi[3])   + 20'(3*xi[2]) - 20'(3*xi[1]) + 20'(xi[0]);
          c3q <= -20'(xq[3])   + 20'(3*xq[2]) - 20'(3*xq[1]) + 20'(xq[0]);
          fst <= F_M3;
        end
        // Horner evaluation, multiply and add in separate cycles (DSP M-reg, then add)
        F_M3: begin pmi <= mulmu(24'(c3i), mu); pmq <= mulmu(24'(c3q), mu); fst <= F_A3; end
        F_A3: begin ti <= 24'(c2i) + pmi; tq <= 24'(c2q) + pmq;             fst <= F_M2; end
        F_M2: begin pmi <= mulmu(ti, mu);       pmq <= mulmu(tq, mu);       fst <= F_A2; end
        F_A2: begin ti <= 24'(c1i) + pmi; tq <= 24'(c1q) + pmq;             fst <= F_M1; end
        F_M1: begin ti <= mulmu(ti, mu);        tq <= mulmu(tq, mu);        fst <= F_MY; end
        F_MY: begin
          // t/6 with 1/6 ~= 10923/65536
          logic signed [41:0] pi_, pq_;
          pi_ = ti * 42'sd10923;
          pq_ = tq * 42'sd10923;
          pmi <= 24'(pi_ >>> 16);
          pmq <= 24'(pq_ >>> 16);
          fst <= F_Y;
        end
        F_Y: begin
          yi  <= sat16(32'(c0i) + 32'(pmi));
          yq  <= sat16(32'(c0q) + 32'(pmq));
          fst <= F_TED;
        end
        F_TED: begin
          if (ontime) begin
            // Gardner: e = Re{ mid * (prev - now)^* }  (pre-add + multiply, registered)
            ted_pi <= 33'(mid_i * (17'(prev_i) - 17'(yi)));
            ted_pq <= 33'(mid_q * (17'(prev_q) - 17'(yq)));
            prev_i <= yi;
            prev_q <= yq;
            sym_i  <= yi;
            sym_q  <= yq;
            sym_vld <= 1'b1;
            fst    <= F_LF1;
          end else begin
            mid_i <= yi;
            mid_q <= yq;
            fst   <= F_IDLE;
          end
          ontime <= ~ontime;
        end
        F_LF1: begin
          logic signed [33:0] e;
          logic signed [31:0] es;
          e      = 34'(ted_pi) + 34'(ted_pq);
          ted_e  <= e;
          es     = -32'(e >>> 15);
          inc_p  <= (es <<< 8) >>> kp_sh;
          vacc   <= clampv(vacc + ((es <<< 8) >>> ki_sh));
          fst    <= F_LF2;
        end
        F_LF2: begin
          v   <= clampv(vacc + inc_p);
          fst <= F_IDLE;
        end
        default: ;
      endcase
    end
  end

  function automatic logic signed [31:0] clampv(input logic signed [31:0] a);
    if (a > 32'sd1048576)  return 32'sd1048576;    // +-1/16 of a sample per interpolant
    if (a < -32'sd1048576) return -32'sd1048576;
    return a;
  endfunction

  assign loop_v = v;
`ifndef SYNTHESIS
  always @(posedge clk) if (!rst && in_vld && !(fst == F_IDLE || fst == F_LF2 || fst == F_LF1)) $error("timing_rec overrun: new input before processing finished");
`endif
endmodule
