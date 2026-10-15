// Symbol timing recovery: cubic Lagrange Farrow interpolator + Gardner TED + PI loop filter
// + modulo-1 decrementing NCO (Gardner/Rice structure).
// Input: 4 samples/symbol (strobed, >= 4 clocks apart). Interpolants at 2 samples/symbol.
// Output: 1 sample/symbol. eta: unsigned, 1.0 = 2^24. W nominal = 0.5.
//
// Fully pipelined (initiation interval 1): Farrow 8 stages, Gardner + loop filter 3 stages.
// The NCO uses the loop word v as it stands; the extra latency (~11 clocks, < 1 symbol at the
// fastest rate) is negligible for the loop bandwidth used.
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

  logic signed [15:0] xi [3], xq [3];     // [0] newest previous sample
  logic [25:0]        eta;
  logic signed [31:0] vacc, v;

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

  function automatic logic signed [31:0] clampv(input logic signed [31:0] a);
    if (a > 32'sd1048576)  return 32'sd1048576;    // +-1/16 of a sample per interpolant
    if (a < -32'sd1048576) return -32'sd1048576;
    return a;
  endfunction

  // ---------------- NCO + sample capture ----------------
  logic               s_v;                          // stage 0 valid
  logic signed [15:0] sm1i, s0i, s1i, s2i, sm1q, s0q, s1q, s2q;
  logic [16:0]        mu0;

  always_ff @(posedge clk) begin
    if (rst) begin
      eta <= ONE >> 1;
      s_v <= 1'b0;
      for (int k = 0; k < 3; k++) begin xi[k] <= '0; xq[k] <= '0; end
    end else begin
      s_v <= 1'b0;
      if (in_vld) begin
        logic signed [26:0] en;
        logic [25:0] w;
        w  = 26'(HALF + 26'(v));
        en = $signed({1'b0, eta}) - $signed({1'b0, w});
        xi[0] <= in_i; xq[0] <= in_q;
        for (int k = 1; k < 3; k++) begin xi[k] <= xi[k-1]; xq[k] <= xq[k-1]; end
        if (en < 0) begin
          // underflow -> strobe; interpolate between x(n-2) and x(n-1), mu = eta/W ~= 2*eta
          logic [26:0] m2;
          m2   = {eta, 1'b0};
          mu0  <= (m2 >= 27'(ONE)) ? 17'h0FFFF : {1'b0, m2[23:8]};
          sm1i <= xi[2]; s0i <= xi[1]; s1i <= xi[0]; s2i <= in_i;
          sm1q <= xq[2]; s0q <= xq[1]; s1q <= xq[0]; s2q <= in_q;
          s_v  <= 1'b1;
          eta  <= 26'(en + $signed({1'b0, ONE}));
        end else begin
          eta <= 26'(en);
        end
      end
    end
  end

  // ---------------- Farrow pipeline (Horner, every product registered) ----------------
  // 6*c1 = -2 s-1 - 3 s0 + 6 s1 - s2 ; 6*c2 = 3 s-1 - 6 s0 + 3 s1 ; 6*c3 = -s-1 + 3 s0 - 3 s1 + s2
  logic [8:1]         fv;
  logic [16:0]        mu [1:7];
  logic signed [15:0] c0i [1:7], c0q [1:7];
  logic signed [19:0] c1i [1:4], c1q [1:4], c2i [1:2], c2q [1:2], c3i, c3q;
  logic signed [23:0] ti3, tq3, ti5, tq5, pi2, pq2, pi4, pq4, pi6, pq6, pi7, pq7;
  logic signed [15:0] yi, yq;

  always_ff @(posedge clk) begin
    fv <= rst ? '0 : {fv[7:1], s_v};
    // P1: coefficients
    c0i[1] <= s0i; c0q[1] <= s0q; mu[1] <= mu0;
    c1i[1] <= -20'(2*sm1i) - 20'(3*s0i) + 20'(6*s1i) - 20'(s2i);
    c1q[1] <= -20'(2*sm1q) - 20'(3*s0q) + 20'(6*s1q) - 20'(s2q);
    c2i[1] <=  20'(3*sm1i) - 20'(6*s0i) + 20'(3*s1i);
    c2q[1] <=  20'(3*sm1q) - 20'(6*s0q) + 20'(3*s1q);
    c3i    <= -20'(sm1i) + 20'(3*s0i) - 20'(3*s1i) + 20'(s2i);
    c3q    <= -20'(sm1q) + 20'(3*s0q) - 20'(3*s1q) + 20'(s2q);
    // P2: c3*mu
    pi2 <= mulmu(24'(c3i), mu[1]); pq2 <= mulmu(24'(c3q), mu[1]);
    c2i[2] <= c2i[1]; c2q[2] <= c2q[1]; c1i[2] <= c1i[1]; c1q[2] <= c1q[1];
    // P3: c2 + .
    ti3 <= 24'(c2i[2]) + pi2; tq3 <= 24'(c2q[2]) + pq2;
    c1i[3] <= c1i[2]; c1q[3] <= c1q[2];
    // P4: (.)*mu
    pi4 <= mulmu(ti3, mu[3]); pq4 <= mulmu(tq3, mu[3]);
    c1i[4] <= c1i[3]; c1q[4] <= c1q[3];
    // P5: c1 + .
    ti5 <= 24'(c1i[4]) + pi4; tq5 <= 24'(c1q[4]) + pq4;
    // P6: (.)*mu
    pi6 <= mulmu(ti5, mu[5]); pq6 <= mulmu(tq5, mu[5]);
    // P7: /6 (10923/65536)
    begin
      logic signed [41:0] a, b;
      a = pi6 * 42'sd10923;
      b = pq6 * 42'sd10923;
      pi7 <= 24'(a >>> 16);
      pq7 <= 24'(b >>> 16);
    end
    // P8: y = c0 + t/6
    yi <= sat16(32'(c0i[7]) + 32'(pi7));
    yq <= sat16(32'(c0q[7]) + 32'(pq7));
    for (int k = 2; k <= 7; k++) begin
      mu[k]  <= mu[k-1];
      c0i[k] <= c0i[k-1];
      c0q[k] <= c0q[k-1];
    end
  end

  // ---------------- Gardner TED + PI loop filter ----------------
  logic               ontime;
  logic signed [15:0] mid_i, mid_q, prev_i, prev_q;
  logic signed [32:0] ted_pi, ted_pq;
  logic signed [31:0] inc_p;
  logic [2:1]         lv;

  always_ff @(posedge clk) begin
    if (rst) begin
      ontime  <= 1'b0;
      sym_vld <= 1'b0;
      lv      <= '0;
      vacc    <= '0;
      v       <= '0;
      mid_i <= '0; mid_q <= '0; prev_i <= '0; prev_q <= '0;
    end else begin
      sym_vld <= 1'b0;
      lv      <= {lv[1], 1'b0};
      if (fv[8]) begin
        if (ontime) begin
          // e = Re{ mid * (prev - now)^* }   (pre-add + multiply, registered)
          ted_pi  <= 33'(mid_i * (17'(prev_i) - 17'(yi)));
          ted_pq  <= 33'(mid_q * (17'(prev_q) - 17'(yq)));
          prev_i  <= yi;
          prev_q  <= yq;
          sym_i   <= yi;
          sym_q   <= yq;
          sym_vld <= 1'b1;
          lv[1]   <= 1'b1;
        end else begin
          mid_i <= yi;
          mid_q <= yq;
        end
        ontime <= ~ontime;
      end
      if (lv[1]) begin
        logic signed [33:0] e;
        logic signed [31:0] es;
        e     = 34'(ted_pi) + 34'(ted_pq);
        es    = -32'(e >>> 15);
        inc_p <= (es <<< 8) >>> kp_sh;
        vacc  <= clampv(vacc + ((es <<< 8) >>> ki_sh));
      end
      if (lv[2]) v <= clampv(vacc + inc_p);
    end
  end

  assign loop_v = v;

`ifndef SYNTHESIS
  always @(posedge clk) if (!rst && in_vld && s_v)
    $error("timing_rec: input samples closer than 2 clocks");
`endif
endmodule
