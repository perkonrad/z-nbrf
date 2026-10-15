// Fully pipelined CORDIC rotator (initiation interval 1). Angle unit: 2^32 = 2*pi.
// (x,y) rotated by z, gain-compensated. Latency: CORDIC_NIT + 3 cycles.
// A tag bit travels with each sample.
module cordic_pipe
  import nb_coefs_pkg::*;
#(
  parameter int DW = 24
)(
  input  logic                 clk,
  input  logic                 rst,
  input  logic                 in_vld,
  input  logic                 in_tag,
  input  logic signed [DW-1:0] x_in,
  input  logic signed [DW-1:0] y_in,
  input  logic signed [31:0]   z_in,
  output logic                 out_vld,
  output logic                 out_tag,
  output logic signed [DW-1:0] x_out,
  output logic signed [DW-1:0] y_out
);
  localparam int XW = DW + 2;
  localparam int N  = CORDIC_NIT;

  logic signed [XW-1:0] x [N+1], y [N+1];
  logic signed [31:0]   z [N+1];
  logic [N+2:0]         v, t;
  logic signed [XW+18:0] gx, gy;

  function automatic logic signed [31:0] atan_i(input int i);
    return $signed(CORDIC_ATAN[i*32 +: 32]);
  endfunction

  function automatic logic signed [DW-1:0] satd(input logic signed [XW+18:0] a);
    if (a > (XW+19)'((1 <<< (DW-1)) - 1)) return DW'((1 <<< (DW-1)) - 1);
    if (a < -(XW+19)'(1 <<< (DW-1)))      return DW'(-(1 <<< (DW-1)));
    return DW'(a);
  endfunction

  always_ff @(posedge clk) begin
    v <= rst ? '0 : {v[N+1:0], in_vld};
    t <= {t[N+1:0], in_tag};
    // stage 0: bring z into [-pi/2, pi/2)
    if (z_in[31] != z_in[30]) begin
      x[0] <= -XW'(x_in); y[0] <= -XW'(y_in); z[0] <= z_in + 32'sh8000_0000;
    end else begin
      x[0] <= XW'(x_in);  y[0] <= XW'(y_in);  z[0] <= z_in;
    end
    // micro-rotations (constant shifts -> no barrel shifters)
    for (int i = 0; i < N; i++) begin
      if (!z[i][31]) begin
        x[i+1] <= x[i] - (y[i] >>> i);
        y[i+1] <= y[i] + (x[i] >>> i);
        z[i+1] <= z[i] - atan_i(i);
      end else begin
        x[i+1] <= x[i] + (y[i] >>> i);
        y[i+1] <= y[i] - (x[i] >>> i);
        z[i+1] <= z[i] + atan_i(i);
      end
    end
    // gain compensation (registered product), then saturate
    gx    <= (XW+19)'(x[N] * (XW+19)'(CORDIC_INVK_Q17));
    gy    <= (XW+19)'(y[N] * (XW+19)'(CORDIC_INVK_Q17));
    x_out <= satd(gx >>> 17);
    y_out <= satd(gy >>> 17);
  end

  assign out_vld = v[N+2];
  assign out_tag = t[N+2];
endmodule
