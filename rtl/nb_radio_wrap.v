// Verilog-2001 wrapper so nb_radio can be used as a Vivado IP Integrator module reference
// (module references cannot have a SystemVerilog top file).
`timescale 1ns/1ps
module nb_radio_wrap #(
  parameter DAC_SPC = 2
) (
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk CLK" *)
  (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi:s_axis_tx:m_axis_rx:s_axis_adci:s_axis_adcq:m_axis_dac, ASSOCIATED_RESET aresetn, FREQ_HZ 320000000" *)
  input  wire        clk,
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
  (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
  input  wire        aresetn,
  input  wire [7:0]  s_axi_awaddr,
  input  wire        s_axi_awvalid,
  output wire        s_axi_awready,
  input  wire [31:0] s_axi_wdata,
  input  wire [3:0]  s_axi_wstrb,
  input  wire        s_axi_wvalid,
  output wire        s_axi_wready,
  output wire [1:0]  s_axi_bresp,
  output wire        s_axi_bvalid,
  input  wire        s_axi_bready,
  input  wire [7:0]  s_axi_araddr,
  input  wire        s_axi_arvalid,
  output wire        s_axi_arready,
  output wire [31:0] s_axi_rdata,
  output wire [1:0]  s_axi_rresp,
  output wire        s_axi_rvalid,
  input  wire        s_axi_rready,
  input  wire [7:0]  s_axis_tx_tdata,
  input  wire        s_axis_tx_tvalid,
  input  wire        s_axis_tx_tlast,
  output wire        s_axis_tx_tready,
  output wire [7:0]  m_axis_rx_tdata,
  output wire        m_axis_rx_tvalid,
  output wire        m_axis_rx_tlast,
  input  wire        m_axis_rx_tready,
  input  wire [15:0] s_axis_adci_tdata,
  input  wire        s_axis_adci_tvalid,
  output wire        s_axis_adci_tready,
  input  wire [15:0] s_axis_adcq_tdata,
  input  wire        s_axis_adcq_tvalid,
  output wire        s_axis_adcq_tready,
  output wire [32*DAC_SPC-1:0] m_axis_dac_tdata,
  output wire        m_axis_dac_tvalid,
  input  wire        m_axis_dac_tready,
  // direct channel control (see nb_radio_top.sv); strobe is synchronised inside
  input  wire [6:0]  chan_i,
  input  wire        chan_stb_i,
  output wire        tx_busy_o,
  output wire        rx_busy_o
);
  nb_radio_top #(.DAC_SPC(DAC_SPC)) u_top (
    .clk(clk), .aresetn(aresetn),
    .s_axi_awaddr(s_axi_awaddr), .s_axi_awvalid(s_axi_awvalid), .s_axi_awready(s_axi_awready),
    .s_axi_wdata(s_axi_wdata), .s_axi_wstrb(s_axi_wstrb), .s_axi_wvalid(s_axi_wvalid),
    .s_axi_wready(s_axi_wready), .s_axi_bresp(s_axi_bresp), .s_axi_bvalid(s_axi_bvalid),
    .s_axi_bready(s_axi_bready), .s_axi_araddr(s_axi_araddr), .s_axi_arvalid(s_axi_arvalid),
    .s_axi_arready(s_axi_arready), .s_axi_rdata(s_axi_rdata), .s_axi_rresp(s_axi_rresp),
    .s_axi_rvalid(s_axi_rvalid), .s_axi_rready(s_axi_rready),
    .s_axis_tx_tdata(s_axis_tx_tdata), .s_axis_tx_tvalid(s_axis_tx_tvalid),
    .s_axis_tx_tlast(s_axis_tx_tlast), .s_axis_tx_tready(s_axis_tx_tready),
    .m_axis_rx_tdata(m_axis_rx_tdata), .m_axis_rx_tvalid(m_axis_rx_tvalid),
    .m_axis_rx_tlast(m_axis_rx_tlast), .m_axis_rx_tready(m_axis_rx_tready),
    .s_axis_adci_tdata(s_axis_adci_tdata), .s_axis_adci_tvalid(s_axis_adci_tvalid),
    .s_axis_adci_tready(s_axis_adci_tready),
    .s_axis_adcq_tdata(s_axis_adcq_tdata), .s_axis_adcq_tvalid(s_axis_adcq_tvalid),
    .s_axis_adcq_tready(s_axis_adcq_tready),
    .m_axis_dac_tdata(m_axis_dac_tdata), .m_axis_dac_tvalid(m_axis_dac_tvalid),
    .m_axis_dac_tready(m_axis_dac_tready),
    .chan_i(chan_i), .chan_stb_i(chan_stb_i), .tx_busy_o(tx_busy_o), .rx_busy_o(rx_busy_o)
  );
endmodule
