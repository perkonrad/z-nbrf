// Simple synchronous FIFO, first-word-fall-through output, BRAM-inferable storage.
module sync_fifo #(
  parameter int W  = 9,
  parameter int AW = 12
)(
  input  logic         clk,
  input  logic         rst,
  input  logic         wr,
  input  logic [W-1:0] wdata,
  output logic         full,
  output logic         rd_vld,
  output logic [W-1:0] rd_data,
  input  logic         rd_rdy
);
  (* ram_style = "block" *) logic [W-1:0] mem [1 << AW];
  logic [AW:0]  wp, rp;
  logic [W-1:0] q;
  logic         q_v, empty, ren;

  assign full  = (wp[AW] != rp[AW]) && (wp[AW-1:0] == rp[AW-1:0]);
  assign empty = (wp == rp);
  // prefetch into output register
  assign ren   = !empty && (!rd_vld || rd_rdy) && !q_v;

  always_ff @(posedge clk) begin
    if (wr && !full) mem[wp[AW-1:0]] <= wdata;
    if (ren) q <= mem[rp[AW-1:0]];
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      wp     <= '0;
      rp     <= '0;
      q_v    <= 1'b0;
      rd_vld <= 1'b0;
    end else begin
      if (wr && !full) wp <= wp + 1'b1;
      if (ren) rp <= rp + 1'b1;
      q_v <= ren;
      if (q_v) begin
        rd_vld  <= 1'b1;
        rd_data <= q;
      end else if (rd_vld && rd_rdy) begin
        rd_vld <= 1'b0;
      end
    end
  end
endmodule
