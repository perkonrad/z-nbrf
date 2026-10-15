// Complex CIC decimator, N=4, M=1, programmable rate R (2..63), runtime output shift.
// Gain = R^4; output = sat(round(acc >>> shift)).
module cic_dec #(
  parameter int IW = 16,
  parameter int OW = 18,
  parameter int W  = 40          // >= IW + 4*log2(Rmax)
)(
  input  logic                 clk,
  input  logic                 rst,
  input  logic [5:0]           rate,
  input  logic [5:0]           shift,
  input  logic                 in_vld,
  input  logic signed [IW-1:0] in_i,
  input  logic signed [IW-1:0] in_q,
  output logic                 out_vld,
  output logic signed [OW-1:0] out_i,
  output logic signed [OW-1:0] out_q
);
  localparam int N = 4;
  logic signed [W-1:0] ii [N], iq [N];       // integrators
  logic signed [W-1:0] ci [N+1], cq [N+1];   // comb stage outputs (ci[0] = decimated sample)
  logic signed [W-1:0] di [N], dq [N];       // comb delays
  logic [N:0] cv;
  logic [5:0] cnt;

  always_ff @(posedge clk) begin
    if (rst) begin
      for (int k = 0; k < N; k++) begin ii[k] <= '0; iq[k] <= '0; di[k] <= '0; dq[k] <= '0; end
      cnt <= '0;
      cv  <= '0;
    end else begin
      cv <= {cv[N-1:0], 1'b0};
      if (in_vld) begin
        ii[0] <= ii[0] + W'(in_i);
        iq[0] <= iq[0] + W'(in_q);
        for (int k = 1; k < N; k++) begin
          ii[k] <= ii[k] + ii[k-1];
          iq[k] <= iq[k] + iq[k-1];
        end
        if (cnt >= rate - 6'd1) begin
          cnt   <= '0;
          ci[0] <= ii[N-1];
          cq[0] <= iq[N-1];
          cv[0] <= 1'b1;
        end else begin
          cnt <= cnt + 6'd1;
        end
      end
      // pipelined combs, one stage per cycle
      for (int k = 0; k < N; k++) begin
        if (cv[k]) begin
          ci[k+1] <= ci[k] - di[k];
          cq[k+1] <= cq[k] - dq[k];
          di[k]   <= ci[k];
          dq[k]   <= cq[k];
        end
      end
    end
  end

  // two-stage output: (1) round + barrel shift, (2) saturate
  logic signed [W:0] shi, shq;
  logic              shv;
  logic signed [W:0] rbit;
  always_comb rbit = (shift == 0) ? '0 : ((W+1)'(1) <<< (shift - 6'd1));

  function automatic logic signed [OW-1:0] sat(input logic signed [W:0] r);
    if (r > (W+1)'((1 <<< (OW-1)) - 1)) return OW'((1 <<< (OW-1)) - 1);
    if (r < -(W+1)'(1 <<< (OW-1)))      return OW'(-(1 <<< (OW-1)));
    return OW'(r);
  endfunction

  always_ff @(posedge clk) begin
    if (rst) begin
      shv     <= 1'b0;
      out_vld <= 1'b0;
    end else begin
      shv     <= cv[N];
      out_vld <= shv;
    end
    if (cv[N]) begin
      shi <= ((W+1)'(ci[N]) + rbit) >>> shift;   // low rate: inputs stable for >= 8 clocks
      shq <= ((W+1)'(cq[N]) + rbit) >>> shift;
    end
    if (shv) begin
      out_i <= sat(shi);
      out_q <= sat(shq);
    end
  end
endmodule
