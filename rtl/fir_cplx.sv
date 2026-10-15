// Complex FIR (real coefficients), fully parallel, pipelined adder tree.
// Data advances on in_vld. Optional decimation by 2 (output every 2nd input).
// For interpolation, the caller zero-stuffs the input stream.
// Latency: LEV+3 cycles from in_vld to out_vld.
module fir_cplx #(
  parameter int NTAP  = 23,
  parameter int IW    = 18,
  parameter int CW    = 18,
  parameter int OW    = 18,
  parameter int SHIFT = 17,      // out = sat(round(acc >>> SHIFT))
  parameter int DECIM = 1,       // 1 or 2
  parameter logic [NTAP*CW-1:0] COEF = '0
)(
  input  logic                 clk,
  input  logic                 rst,
  input  logic                 in_vld,
  input  logic signed [IW-1:0] in_i,
  input  logic signed [IW-1:0] in_q,
  output logic                 out_vld,
  output logic signed [OW-1:0] out_i,
  output logic signed [OW-1:0] out_q
);
  localparam int LEV = $clog2(NTAP);
  localparam int NP2 = 1 << LEV;
  localparam int PW  = IW + CW;
  localparam int AW  = PW + LEV;

  logic signed [IW-1:0] xi [NTAP];
  logic signed [IW-1:0] xq [NTAP];
  logic signed [AW-1:0] ti [LEV+1][NP2];
  logic signed [AW-1:0] tq [LEV+1][NP2];
  logic [LEV+1:0] vp;
  logic ph;

  function automatic logic signed [CW-1:0] coef(input int k);
    return $signed(COEF[k*CW +: CW]);
  endfunction

  always_ff @(posedge clk) begin
    if (in_vld) begin
      xi[0] <= in_i;
      xq[0] <= in_q;
      for (int k = 1; k < NTAP; k++) begin
        xi[k] <= xi[k-1];
        xq[k] <= xq[k-1];
      end
    end
    for (int k = 0; k < NP2; k++) begin
      if (k < NTAP) begin
        ti[0][k] <= AW'(xi[k] * coef(k));
        tq[0][k] <= AW'(xq[k] * coef(k));
      end else begin
        ti[0][k] <= '0;
        tq[0][k] <= '0;
      end
    end
    for (int l = 1; l <= LEV; l++) begin
      for (int k = 0; k < (NP2 >> l); k++) begin
        ti[l][k] <= ti[l-1][2*k] + ti[l-1][2*k+1];
        tq[l][k] <= tq[l-1][2*k] + tq[l-1][2*k+1];
      end
    end
  end

  // valid / decimation phase pipeline
  always_ff @(posedge clk) begin
    if (rst) begin
      vp <= '0;
      ph <= 1'b0;
    end else begin
      if (in_vld) ph <= ~ph;
      vp <= {vp[LEV:0], in_vld & ((DECIM == 1) | ph)};
    end
  end

  // round + saturate
  function automatic logic signed [OW-1:0] rnd_sat(input logic signed [AW-1:0] a);
    logic signed [AW:0] r;
    r = (AW+1)'(a) + (AW+1)'(1 <<< (SHIFT-1));
    r = r >>> SHIFT;
    if (r > (AW+1)'((1 <<< (OW-1)) - 1))  return OW'((1 <<< (OW-1)) - 1);
    if (r < -(AW+1)'(1 <<< (OW-1)))       return OW'(-(1 <<< (OW-1)));
    return OW'(r);
  endfunction

  always_ff @(posedge clk) begin
    if (rst) out_vld <= 1'b0;
    else     out_vld <= vp[LEV+1];
    if (vp[LEV+1]) begin
      out_i <= rnd_sat(ti[LEV][0]);
      out_q <= rnd_sat(tq[LEV][0]);
    end
  end

endmodule
