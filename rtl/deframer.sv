// Deframer: frame reference from sym_proc tag -> 32 BPSK-diagonal header symbols (CRC-8) ->
// payload (QPSK/16-QAM) -> descramble -> CRC-16 check.
// Output byte stream per frame: payload[len], seq, status{6'b0, mod16, crc_ok} (tlast).
// Header (32 b, MSB first): mod[3:0] | len[11:0] | seq[7:0] | crc8(bits 31..8)
module deframer
  import nb_pkg::*;
(
  input  logic               clk,
  input  logic               rst,
  input  logic               in_vld,
  input  logic               in_tag,
  input  logic [3:0]         in_bits,
  input  logic               in_dbit,
  input  logic signed [16:0] e_i, e_q,
  output logic               busy,
  output logic               mod16,      // constellation of the next symbol
  // byte output
  output logic               ob_vld,
  output logic [7:0]         ob_data,
  output logic               ob_last,
  // status
  output logic [31:0]        cnt_ok,
  output logic [31:0]        cnt_crc_err,
  output logic [31:0]        cnt_hdr_err,
  output logic [31:0]        evm_acc,    // sum |e|^2 >> 8 over last frame payload
  output logic [31:0]        evm_n,      // payload symbols in last frame
  output logic [31:0]        last_hdr
);
  typedef enum logic [2:0] {S_SEARCH, S_SKIP, S_HDR, S_PAY, S_SEQ, S_STAT} st_t;
  st_t st;

  logic [5:0]  hcnt;
  logic [31:0] hdr;
  logic [11:0] len;
  logic        m16;
  logic [7:0]  seq;
  logic [15:0] nsym, scnt;
  logic [11:0] bcnt;          // bytes assembled (payload + 2 crc)
  logic [7:0]  sh;
  logic [3:0]  nb;            // bits in sh
  logic [14:0] prbs;
  logic [15:0] crc;
  logic [31:0] eacc, en, esq;
  logic        esq_v, esq_v2;

  assign busy  = (st != S_SEARCH);
  assign mod16 = (st == S_PAY) ? m16 : 1'b0;

  always_ff @(posedge clk) begin
    if (rst) begin
      st          <= S_SEARCH;
      ob_vld      <= 1'b0;
      ob_last     <= 1'b0;
      cnt_ok      <= '0;
      cnt_crc_err <= '0;
      cnt_hdr_err <= '0;
      evm_acc     <= '0;
      evm_n       <= '0;
      last_hdr    <= '0;
    end else begin
      ob_vld  <= 1'b0;
      ob_last <= 1'b0;
      esq_v   <= 1'b0;
      esq     <= 32'((e_i * e_i + e_q * e_q) >>> 8);   // DSP squares, registered
      if (esq_v2) eacc <= eacc + esq;
      esq_v2  <= esq_v;
      case (st)
        S_SEARCH: if (in_vld && in_tag) begin
          st <= S_SKIP;                 // tag = preamble symbol 62
        end
        S_SKIP: if (in_vld) begin       // preamble symbol 63; header follows
          hcnt <= 6'd0;
          hdr  <= '0;
          st   <= S_HDR;
        end
        S_HDR: begin
          if (in_vld) begin
            hdr  <= {hdr[30:0], in_dbit};
            hcnt <= hcnt + 6'd1;
          end
          if (hcnt == 6'd32) begin
            last_hdr <= hdr;
            if (crc8_24(hdr[31:8]) == hdr[7:0] && hdr[31:28] <= 4'd1 && hdr[27:16] != 12'd0) begin
              m16  <= hdr[28];
              len  <= hdr[27:16];
              seq  <= hdr[15:8];
              nsym <= hdr[28] ? 16'({4'b0, hdr[27:16]} + 16'd2) << 1 : 16'({4'b0, hdr[27:16]} + 16'd2) << 2;
              scnt <= '0;
              bcnt <= '0;
              nb   <= '0;
              prbs <= PRBS15_INIT;
              crc  <= 16'hFFFF;
              eacc <= '0;
              en   <= '0;
              st   <= S_PAY;
            end else begin
              cnt_hdr_err <= cnt_hdr_err + 32'd1;
              st          <= S_SEARCH;
            end
          end
        end
        S_PAY: begin
          if (in_vld) begin
            logic [7:0]  b;
            logic [3:0]  n2;
            logic [7:0]  s2;
            if (m16) begin s2 = {sh[3:0], in_bits};       n2 = nb + 4'd4; end
            else     begin s2 = {sh[5:0], in_bits[1:0]};  n2 = nb + 4'd2; end
            esq_v <= 1'b1;
            en    <= en + 32'd1;
            scnt <= scnt + 16'd1;
            if (n2 == 4'd8) begin
              logic [22:0] pk;
              pk   = prbs15_byte(prbs);
              prbs <= pk[22:8];
              b    = s2 ^ pk[7:0];
              crc  <= crc16_byte(crc, b);
              bcnt <= bcnt + 12'd1;
              nb   <= '0;
              if (bcnt < len) begin
                ob_vld  <= 1'b1;
                ob_data <= b;
              end
            end else begin
              nb <= n2;
            end
            sh <= s2;
            if (scnt == nsym - 16'd1) st <= S_SEQ;
          end
        end
        S_SEQ: begin
          ob_vld  <= 1'b1;
          ob_data <= seq;
          st      <= S_STAT;
        end
        S_STAT: begin
          ob_vld  <= 1'b1;
          ob_last <= 1'b1;
          ob_data <= {6'b0, m16, (crc == 16'h0000)};
          if (crc == 16'h0000) cnt_ok <= cnt_ok + 32'd1;
          else                 cnt_crc_err <= cnt_crc_err + 32'd1;
          evm_acc <= eacc;   // last payload symbol's |e|^2 lands 2 cycles after it: S_SEQ+S_STAT
          evm_n   <= en;
          st      <= S_SEARCH;
        end
        default: st <= S_SEARCH;
      endcase
    end
  end
endmodule
