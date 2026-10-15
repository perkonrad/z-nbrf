// Iterative CORDIC. Angle unit: 2^32 = 2*pi (wraps naturally).
//   mode 0 (rotate): (x,y) rotated by z, gain-compensated.
//   mode 1 (vector): z_out = atan2(y,x); x_out = |(x,y)| * K (uncompensated).
// Latency: CORDIC_NIT + 4 cycles.
module cordic
  import nb_coefs_pkg::*;
#(
  parameter int DW = 24
)(
  input  logic                 clk,
  input  logic                 rst,
  input  logic                 start,
  input  logic                 mode,
  input  logic signed [DW-1:0] x_in,
  input  logic signed [DW-1:0] y_in,
  input  logic signed [31:0]   z_in,
  output logic                 done,
  output logic signed [DW-1:0] x_out,
  output logic signed [DW-1:0] y_out,
  output logic signed [31:0]   z_out
);
  localparam int XW = DW + 2;
  logic signed [XW-1:0] x, y;
  logic signed [31:0]   z;
  logic [4:0]           it;
  logic                 busy, md, fin, fin2;
  logic signed [XW+18:0] gx, gy;

  function automatic logic signed [31:0] atan_i(input logic [4:0] i);
    return $signed(CORDIC_ATAN[i*32 +: 32]);
  endfunction

  function automatic logic signed [DW-1:0] satd(input logic signed [XW+18:0] a);
    if (a > (XW+19)'((1 <<< (DW-1)) - 1)) return DW'((1 <<< (DW-1)) - 1);
    if (a < -(XW+19)'(1 <<< (DW-1)))      return DW'(-(1 <<< (DW-1)));
    return DW'(a);
  endfunction

  always_ff @(posedge clk) begin
    if (rst) begin
      busy <= 1'b0;
      done <= 1'b0;
      fin  <= 1'b0;
      fin2 <= 1'b0;
    end else begin
      done <= 1'b0;
      fin  <= 1'b0;
      if (start && !busy) begin
        md   <= mode;
        busy <= 1'b1;
        it   <= '0;
        if (!mode) begin
          // rotate: bring z into [-pi/2, pi/2)
          if (z_in[31] != z_in[30]) begin
            x <= -XW'(x_in); y <= -XW'(y_in); z <= z_in + 32'sh8000_0000;
          end else begin
            x <= XW'(x_in);  y <= XW'(y_in);  z <= z_in;
          end
        end else begin
          if (x_in < 0) begin
            x <= -XW'(x_in); y <= -XW'(y_in); z <= 32'sh8000_0000;
          end else begin
            x <= XW'(x_in);  y <= XW'(y_in);  z <= '0;
          end
        end
      end else if (busy) begin
        logic dir;   // 1: rotate counter-clockwise
        dir = md ? y[XW-1] : ~z[31];
        if (dir) begin
          x <= x - (y >>> it);
          y <= y + (x >>> it);
          z <= md ? z - atan_i(it) : z - atan_i(it);
        end else begin
          x <= x + (y >>> it);
          y <= y - (x >>> it);
          z <= md ? z + atan_i(it) : z + atan_i(it);
        end
        if (it == 5'(CORDIC_NIT - 1)) begin
          busy <= 1'b0;
          fin  <= 1'b1;
        end
        it <= it + 5'd1;
      end
      // fin: gain-compensation products registered; fin2: saturate + done
      fin2 <= fin;
      if (fin) begin
        gx <= (XW+19)'(x * (XW+19)'(CORDIC_INVK_Q17));
        gy <= (XW+19)'(y * (XW+19)'(CORDIC_INVK_Q17));
        z_out <= z;
      end
      if (fin2) begin
        if (!md) begin
          x_out <= satd(gx >>> 17);
          y_out <= satd(gy >>> 17);
        end else begin
          x_out <= satd(gx >>> 17);   // vector mode: magnitude*K/K' (not used downstream)
          y_out <= '0;
        end
        done <= 1'b1;
      end
    end
  end
endmodule
