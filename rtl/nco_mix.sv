// Complex mixer with a quarter-wave sine ROM: y = x * e^(+j*phase) (CONJ = 0) or
// x * e^(-j*phase) (CONJ = 1). Phase is mod 2^32 (2^32 = 2*pi); the top 12 bits address
// the table, so phase-truncation spurs sit near -72 dBc. Latency 6 clocks from x/phase to y.
module nco_mix
  import nb_coefs_pkg::*;
#(
  parameter bit CONJ = 1'b0
)(
  input  logic               clk,
  input  logic [31:0]        phase,
  input  logic signed [15:0] x_i,
  input  logic signed [15:0] x_q,
  output logic signed [15:0] y_i,
  output logic signed [15:0] y_q
);
  localparam int AW = $clog2(NCO_N);   // 10: quarter-wave address bits

  (* rom_style = "block" *) logic [NCO_W-1:0] rom [NCO_N];
  initial for (int k = 0; k < NCO_N; k++) rom[k] = NCO_LUT[k*NCO_W +: NCO_W];

  // s1: quadrant split for sin(p) and cos(p) = sin(p + pi/2)
  logic [AW+1:0]      ps, pc;
  logic [AW-1:0]      as1, ac1;
  logic               ns1, nc1, ns2, nc2;
  assign ps = phase[31 -: AW+2];
  assign pc = ps + (AW+2)'(1 << AW);
  always_ff @(posedge clk) begin
    as1 <= ps[AW] ? ~ps[AW-1:0] : ps[AW-1:0];
    ac1 <= pc[AW] ? ~pc[AW-1:0] : pc[AW-1:0];
    ns1 <= ps[AW+1];
    nc1 <= pc[AW+1];
  end

  // s2: ROM read (registered, maps onto the BRAM output stage)
  logic [NCO_W-1:0] rs2, rc2;
  always_ff @(posedge clk) begin
    rs2 <= rom[as1];
    rc2 <= rom[ac1];
    ns2 <= ns1;
    nc2 <= nc1;
  end

  // s3: apply quadrant sign
  logic signed [NCO_W:0] s3, c3;
  always_ff @(posedge clk) begin
    s3 <= ns2 ? -$signed({1'b0, rs2}) : $signed({1'b0, rs2});
    c3 <= nc2 ? -$signed({1'b0, rc2}) : $signed({1'b0, rc2});
  end

  // x delayed to line up with s3/c3
  logic signed [15:0] xi_d [3], xq_d [3];
  always_ff @(posedge clk) begin
    xi_d[0] <= x_i;      xq_d[0] <= x_q;
    xi_d[1] <= xi_d[0];  xq_d[1] <= xq_d[0];
    xi_d[2] <= xi_d[1];  xq_d[2] <= xq_d[1];
  end

  // s4: products (registered before the adders)
  logic signed [NCO_W+16:0] p_ic, p_qs, p_qc, p_is;
  always_ff @(posedge clk) begin
    p_ic <= xi_d[2] * c3;
    p_qs <= xq_d[2] * s3;
    p_qc <= xq_d[2] * c3;
    p_is <= xi_d[2] * s3;
  end

  // s5: (xi + j xq)(c +/- j s)
  logic signed [NCO_W+17:0] a_i, a_q;
  always_ff @(posedge clk) begin
    if (CONJ) begin
      a_i <= p_ic + p_qs;
      a_q <= p_qc - p_is;
    end else begin
      a_i <= p_ic - p_qs;
      a_q <= p_qc + p_is;
    end
  end

  // s6: round, scale back by 2^(W-1), saturate
  function automatic logic signed [15:0] rsat(input logic signed [NCO_W+17:0] a);
    logic signed [NCO_W+17:0] r;
    r = (a + (NCO_W+18)'(1 << (NCO_W-2))) >>> (NCO_W-1);
    if (r > 32767)  return 16'sd32767;
    if (r < -32768) return -16'sd32768;
    return r[15:0];
  endfunction
  always_ff @(posedge clk) begin
    y_i <= rsat(a_i);
    y_q <= rsat(a_q);
  end
endmodule
