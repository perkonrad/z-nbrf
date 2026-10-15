// Complex CIC interpolator, N=4, M=1. Combs run on ce (input rate fs0/R), integrators
// every clock (output rate fs0). Gain = R^3. Output = sat(round(acc >>> shift)).
module cic_int #(
  parameter int IW = 18,
  parameter int OW = 18,
  parameter int W  = 44
)(
  input  logic                 clk,
  input  logic                 rst,
  input  logic [5:0]           shift,
  input  logic                 ce,        // one pulse per input sample
  input  logic signed [IW-1:0] in_i,      // sampled on ce
  input  logic signed [IW-1:0] in_q,
  output logic signed [OW-1:0] out_i,     // valid every clock
  output logic signed [OW-1:0] out_q
);
  localparam int N = 4;
  logic signed [W-1:0] ci [N+1], cq [N+1];
  logic signed [W-1:0] di [N],   dq [N];
  logic signed [W-1:0] ii [N],   iq [N];
  logic [N:0] cv;

  always_ff @(posedge clk) begin
    if (rst) begin
      for (int k = 0; k < N; k++) begin
        di[k] <= '0; dq[k] <= '0; ii[k] <= '0; iq[k] <= '0;
      end
      for (int k = 0; k <= N; k++) begin ci[k] <= '0; cq[k] <= '0; end
      cv <= '0;
    end else begin
      cv <= {cv[N-1:0], ce};
      if (ce) begin
        ci[0] <= W'(in_i);
        cq[0] <= W'(in_q);
      end
      for (int k = 0; k < N; k++) begin
        if (cv[k]) begin
          ci[k+1] <= ci[k] - di[k];
          cq[k+1] <= cq[k] - dq[k];
          di[k]   <= ci[k];
          dq[k]   <= cq[k];
        end
      end
      // zero-stuffed input to the integrator chain
      ii[0] <= ii[0] + (cv[N] ? ci[N] : '0);
      iq[0] <= iq[0] + (cv[N] ? cq[N] : '0);
      for (int k = 1; k < N; k++) begin
        ii[k] <= ii[k] + ii[k-1];
        iq[k] <= iq[k] + iq[k-1];
      end
    end
  end

  logic signed [W:0] shi, shq, rbit;
  always_comb rbit = (shift == 0) ? '0 : ((W+1)'(1) <<< (shift - 6'd1));

  function automatic logic signed [OW-1:0] sat(input logic signed [W:0] r);
    if (r > (W+1)'((1 <<< (OW-1)) - 1)) return OW'((1 <<< (OW-1)) - 1);
    if (r < -(W+1)'(1 <<< (OW-1)))      return OW'(-(1 <<< (OW-1)));
    return OW'(r);
  endfunction

  logic signed [W:0] adi, adq;
  always_ff @(posedge clk) begin
    adi   <= (W+1)'(ii[N-1]) + rbit;     // round
    adq   <= (W+1)'(iq[N-1]) + rbit;
    shi   <= adi >>> shift;              // barrel shift
    shq   <= adq >>> shift;
    out_i <= sat(shi);                   // saturate
    out_q <= sat(shq);
  end
endmodule
