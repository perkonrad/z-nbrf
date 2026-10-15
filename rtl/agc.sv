// Feedback AGC. y = sat16(x * g), g unsigned Q10.12 (1/256 .. 1024). Detector: alpha-max-beta-min |y|
// (max + min/2). Multiplicative loop: gacc += ((target - |y|) * g) >>> mu_sh, g = gacc[35:14]
// -> relative settling time independent of input level (log-domain behaviour).
module agc #(
  parameter int IW = 18,
  parameter int OW = 16
)(
  input  logic                 clk,
  input  logic                 rst,
  input  logic [15:0]          target,
  input  logic [4:0]           mu_sh,     // 31 = frozen
  input  logic                 in_vld,
  input  logic signed [IW-1:0] in_i,
  input  logic signed [IW-1:0] in_q,
  output logic                 out_vld,
  output logic signed [OW-1:0] out_i,
  output logic signed [OW-1:0] out_q,
  output logic [21:0]          gain
);
  logic [35:0] gacc;
  logic signed [IW+22:0] pi, pq;
  logic v1;

  assign gain = gacc[35:14];

  function automatic logic signed [OW-1:0] sat(input logic signed [IW+22:0] r);
    if (r > (IW+23)'((1 <<< (OW-1)) - 1)) return OW'((1 <<< (OW-1)) - 1);
    if (r < -(IW+23)'(1 <<< (OW-1)))      return OW'(-(1 <<< (OW-1)));
    return OW'(r);
  endfunction


  // loop update pipeline (one update per input sample, >= 16 clocks apart):
  //   v2: |y| estimate  ->  v3: error/step  ->  v4: gacc update + clamp
  logic [OW-1:0]        ai, aq, amax, amin;
  logic [OW:0]          amag;
  logic signed [OW+1:0] err;
  logic signed [37:0]   step;
  logic signed [40:0]   eg;
  logic signed [37:0]   nx;
  logic                 v2, v3, v4;
  logic [2:0]           v4_d;

  function automatic logic [OW-1:0] absq(input logic signed [OW-1:0] a);
    return a[OW-1] ? OW'(-a) : OW'(a);
  endfunction

  always_ff @(posedge clk) begin
    ai <= absq(out_i);
    aq <= absq(out_q);
    amax <= (absq(out_i) > absq(out_q)) ? absq(out_i) : absq(out_q);
    amin <= (absq(out_i) > absq(out_q)) ? absq(out_q) : absq(out_i);
    amag <= {1'b0, amax} + {2'b0, amin[OW-1:1]};
    err  <= $signed({2'b0, target}) - $signed({1'b0, amag});
    eg   <= 41'(err * $signed({1'b0, gain}));        // DSP, registered
    step <= 38'(eg >>> mu_sh);
    nx   <= $signed({2'b0, gacc}) + step;
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      gacc    <= 36'd4096 << 14;   // g = 1.0
      v1      <= 1'b0;
      out_vld <= 1'b0;
      {v2, v3, v4} <= '0;
    end else begin
      v1      <= in_vld;
      out_vld <= v1;
      // out regs valid from out_vld; amax/amin at +1, amag +2, err +3, eg +4, step +5, nx +6
      v2 <= out_vld;
      v3 <= v2;
      v4 <= v3;
      if (v4_d[2] && mu_sh != 5'd31) begin
        if (nx < 38'sd262144)              gacc <= 36'd262144;        // g >= 1/256
        else if (nx > 38'sh0_F_FFFF_0000)  gacc <= 36'hF_FFFF_0000;   // g <  1024
        else                               gacc <= nx[35:0];
      end
    end
  end
  always_ff @(posedge clk) v4_d <= rst ? 3'b0 : {v4_d[1:0], v4};

  always_ff @(posedge clk) begin
    if (in_vld) begin
      pi <= (IW+23)'(in_i * $signed({1'b0, gain}));
      pq <= (IW+23)'(in_q * $signed({1'b0, gain}));
    end
    if (v1) begin
      out_i <= sat(pi >>> 12);
      out_q <= sat(pq >>> 12);
    end
  end
endmodule
