// AXI4-Lite register file (radio clock domain; cross with SmartConnect in the BD).
// See docs/README.md for the register map.
module axil_regs
  import nb_pkg::*;
(
  input  logic        clk,
  input  logic        rst,
  // AXI4-Lite
  input  logic [7:0]  s_axi_awaddr,
  input  logic        s_axi_awvalid,
  output logic        s_axi_awready,
  input  logic [31:0] s_axi_wdata,
  input  logic [3:0]  s_axi_wstrb,
  input  logic        s_axi_wvalid,
  output logic        s_axi_wready,
  output logic [1:0]  s_axi_bresp,
  output logic        s_axi_bvalid,
  input  logic        s_axi_bready,
  input  logic [7:0]  s_axi_araddr,
  input  logic        s_axi_arvalid,
  output logic        s_axi_arready,
  output logic [31:0] s_axi_rdata,
  output logic [1:0]  s_axi_rresp,
  output logic        s_axi_rvalid,
  input  logic        s_axi_rready,
  // control outputs
  output logic [31:0] ctrl,
  output logic [5:0]  rate,
  output logic [5:0]  rx_shift,
  output logic [5:0]  tx_shift,
  output logic [15:0] tx_gain,
  output logic [11:0] test_len,
  output logic [15:0] ifs,
  output logic [15:0] agc_target,
  output logic [4:0]  agc_mu_sh,
  output logic [4:0]  ted_kp_sh,
  output logic [4:0]  ted_ki_sh,
  output logic [4:0]  pll_kp_sh,
  output logic [4:0]  pll_ki_sh,
  output logic [4:0]  eq_mu_sh,
  output logic [7:0]  det_thr,
  // status inputs
  input  logic [31:0] st_rx_ok,
  input  logic [31:0] st_rx_crc,
  input  logic [31:0] st_rx_hdr,
  input  logic [31:0] st_tx_frames,
  input  logic [31:0] st_rx_ovf,
  input  logic [31:0] st_pll_freq,
  input  logic [31:0] st_evm_acc,
  input  logic [31:0] st_evm_n,
  input  logic [31:0] st_agc_gain,
  input  logic [31:0] st_peak_m,
  input  logic [31:0] st_peak_e,
  input  logic [31:0] st_last_hdr,
  input  logic [31:0] st_ted_v,
  input  logic [31:0] st_det_cnt
);
  logic [31:0] r_rate, r_shift, r_txg, r_test, r_agc, r_ted, r_pll, r_eq, r_det;

  assign rate       = r_rate[5:0];
  assign rx_shift   = r_shift[5:0];
  assign tx_shift   = r_shift[13:8];
  assign tx_gain    = r_txg[15:0];
  assign test_len   = r_test[11:0];
  assign ifs        = r_test[31:16];
  assign agc_target = r_agc[15:0];
  assign agc_mu_sh  = r_agc[20:16];
  assign ted_kp_sh  = r_ted[4:0];
  assign ted_ki_sh  = r_ted[12:8];
  assign pll_kp_sh  = r_pll[4:0];
  assign pll_ki_sh  = r_pll[12:8];
  assign eq_mu_sh   = r_eq[4:0];
  assign det_thr    = r_det[7:0];

  // ---------------- write channel ----------------
  logic       wr;
  logic [5:0] wa;
  assign s_axi_awready = s_axi_awvalid && s_axi_wvalid && !s_axi_bvalid;
  assign s_axi_wready  = s_axi_awready;
  assign wr            = s_axi_awready;
  assign wa            = s_axi_awaddr[7:2];
  assign s_axi_bresp   = 2'b00;
  assign s_axi_rresp   = 2'b00;

  function automatic logic [31:0] wmask(input logic [31:0] old, input logic [31:0] d, input logic [3:0] s);
    logic [31:0] r;
    for (int b = 0; b < 4; b++) r[8*b +: 8] = s[b] ? d[8*b +: 8] : old[8*b +: 8];
    return r;
  endfunction

  always_ff @(posedge clk) begin
    if (rst) begin
      s_axi_bvalid <= 1'b0;
      ctrl    <= 32'h0000_0063;           // tx_en | rx_en | eq_en | pll_en
      r_rate  <= 32'd10;                  // 5 MHz channel (4 MBd at 320 MHz)
      r_shift <= {18'd0, 6'd10, 2'd0, 6'd14};   // tx ceil(3 log2 R), rx ceil(4 log2 R)
      r_txg   <= 32'h4000;                // 1.0 (Q2.14)
      r_test  <= {16'd32, 4'd0, 12'd64};
      r_agc   <= {11'd0, 5'd9, 16'd11650};   // calibrated: QPSK symbols at +-8192 after timing recovery
      r_ted   <= {19'd0, 5'd10, 3'd0, 5'd3};
      r_pll   <= {19'd0, 5'd10, 3'd0, 5'd3};
      r_eq    <= 32'd6;
      r_det   <= 32'd102;                 // rho >= 0.4
    end else begin
      ctrl[4] <= 1'b0;                    // soft reset is self-clearing
      if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 1'b0;
      if (wr) begin
        s_axi_bvalid <= 1'b1;
        case (wa)
          6'h02: ctrl    <= wmask(ctrl,    s_axi_wdata, s_axi_wstrb);
          6'h03: r_rate  <= wmask(r_rate,  s_axi_wdata, s_axi_wstrb);
          6'h04: r_shift <= wmask(r_shift, s_axi_wdata, s_axi_wstrb);
          6'h05: r_txg   <= wmask(r_txg,   s_axi_wdata, s_axi_wstrb);
          6'h06: r_test  <= wmask(r_test,  s_axi_wdata, s_axi_wstrb);
          6'h07: r_agc   <= wmask(r_agc,   s_axi_wdata, s_axi_wstrb);
          6'h08: r_ted   <= wmask(r_ted,   s_axi_wdata, s_axi_wstrb);
          6'h09: r_pll   <= wmask(r_pll,   s_axi_wdata, s_axi_wstrb);
          6'h0A: r_eq    <= wmask(r_eq,    s_axi_wdata, s_axi_wstrb);
          6'h0B: r_det   <= wmask(r_det,   s_axi_wdata, s_axi_wstrb);
          default: ;
        endcase
      end
    end
  end

  // ---------------- read channel ----------------
  assign s_axi_arready = !s_axi_rvalid;
  always_ff @(posedge clk) begin
    if (rst) begin
      s_axi_rvalid <= 1'b0;
    end else begin
      if (s_axi_rvalid && s_axi_rready) s_axi_rvalid <= 1'b0;
      if (s_axi_arvalid && s_axi_arready) begin
        s_axi_rvalid <= 1'b1;
        case (s_axi_araddr[7:2])
          6'h00: s_axi_rdata <= NB_ID;
          6'h01: s_axi_rdata <= NB_VERSION;
          6'h02: s_axi_rdata <= ctrl;
          6'h03: s_axi_rdata <= r_rate;
          6'h04: s_axi_rdata <= r_shift;
          6'h05: s_axi_rdata <= r_txg;
          6'h06: s_axi_rdata <= r_test;
          6'h07: s_axi_rdata <= r_agc;
          6'h08: s_axi_rdata <= r_ted;
          6'h09: s_axi_rdata <= r_pll;
          6'h0A: s_axi_rdata <= r_eq;
          6'h0B: s_axi_rdata <= r_det;
          6'h10: s_axi_rdata <= st_rx_ok;
          6'h11: s_axi_rdata <= st_rx_crc;
          6'h12: s_axi_rdata <= st_rx_hdr;
          6'h13: s_axi_rdata <= st_tx_frames;
          6'h14: s_axi_rdata <= st_rx_ovf;
          6'h15: s_axi_rdata <= st_pll_freq;
          6'h16: s_axi_rdata <= st_evm_acc;
          6'h17: s_axi_rdata <= st_evm_n;
          6'h18: s_axi_rdata <= st_agc_gain;
          6'h19: s_axi_rdata <= st_peak_m;
          6'h1A: s_axi_rdata <= st_peak_e;
          6'h1B: s_axi_rdata <= st_last_hdr;
          6'h1C: s_axi_rdata <= st_ted_v;
          6'h1D: s_axi_rdata <= st_det_cnt;
          default: s_axi_rdata <= 32'hDEAD_BEEF;
        endcase
      end
    end
  end
endmodule
