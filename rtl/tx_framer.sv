// TX framer + mapper.
// Packet source: AXI4-Stream bytes (tlast delimits a frame, max 4095 bytes) or an internal
// test generator (payload byte i = seq + i). The whole packet is buffered (4 KB BRAM) so the
// header can carry its length; the CRC-16 is computed while buffering.
// Frame: 64 preamble | 32 header (BPSK on I=Q diagonal) | payload+CRC16 (scrambled, QPSK or 16-QAM).
// Between frames: continuous PRBS-23 QPSK idle symbols (keeps RX loops locked), >= ifs symbols.
module tx_framer
  import nb_pkg::*;
  import nb_coefs_pkg::*;
(
  input  logic               clk,
  input  logic               rst,
  input  logic               enable,
  input  logic               test_mode,
  input  logic               mod16,
  input  logic [11:0]        test_len,
  input  logic [15:0]        ifs,
  // packet input
  input  logic [7:0]         s_tdata,
  input  logic               s_tvalid,
  input  logic               s_tlast,
  output logic               s_tready,
  // symbol output
  input  logic               ce_sym,
  output logic signed [15:0] sym_i,
  output logic signed [15:0] sym_q,
  output logic [31:0]        frames_sent
);
  // ---------------- packet buffer ----------------
  (* ram_style = "block" *) logic [7:0] mem [4096];
  logic        we;
  logic [11:0] waddr, raddr;
  logic [7:0]  wdata, rdata;
  always_ff @(posedge clk) begin
    if (we) mem[waddr] <= wdata;
    rdata <= mem[raddr];
  end

  typedef enum logic [1:0] {F_IDLE, F_AXIS, F_GEN} fst_t;
  fst_t        fst;
  logic        full;
  logic [11:0] widx, flen;
  logic [15:0] fcrc, wcrc;
  logic        fmod;
  logic [7:0]  seq;

  // symbol FSM signals used by fill FSM
  logic        release_buf;

  always_comb begin
    s_tready = (fst == F_AXIS) && !full;
    we       = 1'b0;
    waddr    = widx;
    wdata    = (fst == F_GEN) ? 8'(seq + widx[7:0]) : s_tdata;
    if (fst == F_AXIS && s_tvalid && !full) we = 1'b1;
    if (fst == F_GEN) we = 1'b1;
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      fst  <= F_IDLE;
      full <= 1'b0;
      widx <= '0;
      wcrc <= 16'hFFFF;
    end else begin
      if (release_buf) full <= 1'b0;
      case (fst)
        F_IDLE: if (!full && !release_buf && enable) begin
          widx <= '0;
          wcrc <= 16'hFFFF;
          fst  <= test_mode ? F_GEN : F_AXIS;
        end
        F_AXIS: if (test_mode && widx == 12'd0 && !s_tvalid) begin
          fst <= F_IDLE;                       // switched to test generator while waiting
        end else if (s_tvalid && !full) begin
          wcrc <= crc16_byte(wcrc, s_tdata);
          widx <= widx + 12'd1;
          if (s_tlast || widx == 12'd4094) begin
            flen <= widx + 12'd1;
            fcrc <= crc16_byte(wcrc, s_tdata);
            fmod <= mod16;
            full <= 1'b1;
            fst  <= F_IDLE;
          end
        end
        F_GEN: begin
          wcrc <= crc16_byte(wcrc, wdata);
          widx <= widx + 12'd1;
          if (widx >= test_len - 12'd1 || test_len == 12'd0) begin
            flen <= widx + 12'd1;
            fcrc <= crc16_byte(wcrc, wdata);
            fmod <= mod16;
            full <= 1'b1;
            fst  <= F_IDLE;
          end
        end
        default: fst <= F_IDLE;
      endcase
    end
  end

  // ---------------- byte fetch + scrambler ----------------
  logic        fetch_en, rd_pend, nb_v;
  logic [12:0] rptr;
  logic [7:0]  nbyte;
  logic [14:0] prbs;
  logic [12:0] nbytes_tot;
  assign nbytes_tot = 13'(flen) + 13'd2;
  assign raddr      = rptr[11:0];

  // ---------------- symbol FSM ----------------
  typedef enum logic [1:0] {T_IDLE, T_PRE, T_HDR, T_PAY} tst_t;
  tst_t        tst;
  logic [15:0] icnt;
  logic [5:0]  pcnt;
  logic [4:0]  hcnt;
  logic [31:0] hdr;
  logic [15:0] scnt, nsym;
  logic [7:0]  sb;
  logic [3:0]  sbn;
  logic [22:0] idle_lfsr;
  logic        start_fetch;

  always_ff @(posedge clk) begin
    if (rst) begin
      rd_pend <= 1'b0;
      nb_v    <= 1'b0;
      rptr    <= '0;
    end else begin
      if (start_fetch) begin
        rptr    <= '0;
        nb_v    <= 1'b0;
        rd_pend <= 1'b0;
        prbs    <= PRBS15_INIT;
      end else begin
        if (fetch_en && !nb_v && !rd_pend && rptr < nbytes_tot) rd_pend <= 1'b1;   // raddr = rptr now
        if (rd_pend) begin
          logic [7:0]  raw;
          logic [22:0] pk;
          rd_pend <= 1'b0;
          if (rptr < 13'(flen))           raw = rdata;
          else if (rptr == 13'(flen))     raw = fcrc[15:8];
          else                            raw = fcrc[7:0];
          pk    = prbs15_byte(prbs);
          prbs  <= pk[22:8];
          nbyte <= raw ^ pk[7:0];
          nb_v  <= 1'b1;
          rptr  <= rptr + 13'd1;
        end
        if (tst == T_PAY && ce_sym && sbn == 0) nb_v <= 1'b0;   // consumed
      end
    end
  end

  // header word
  logic [23:0] hdr_hi;
  assign hdr_hi = {3'b000, fmod, flen, seq};

  always_ff @(posedge clk) begin
    if (rst) begin
      tst         <= T_IDLE;
      icnt        <= '0;
      seq         <= '0;
      frames_sent <= '0;
      idle_lfsr   <= 23'h5A5A5A;
      sym_i       <= '0;
      sym_q       <= '0;
      release_buf <= 1'b0;
      start_fetch <= 1'b0;
      fetch_en    <= 1'b0;
      sbn         <= '0;
    end else begin
      release_buf <= 1'b0;
      start_fetch <= 1'b0;
      if (ce_sym) begin
        case (tst)
          T_IDLE: begin
            logic [22:0] l;
            logic b0, b1;
            l  = idle_lfsr;
            b1 = l[22] ^ l[17]; l = {l[21:0], b1};
            b0 = l[22] ^ l[17]; l = {l[21:0], b0};
            idle_lfsr <= l;
            sym_i <= b1 ? 16'sd8192 : -16'sd8192;
            sym_q <= b0 ? 16'sd8192 : -16'sd8192;
            if (icnt != 16'hFFFF) icnt <= icnt + 16'd1;
            if (full && enable && icnt >= ifs) begin
              tst  <= T_PRE;
              pcnt <= '0;
            end
          end
          T_PRE: begin
            sym_i <= PREAMBLE[pcnt] ? 16'sd8192 : -16'sd8192;
            sym_q <= PREAMBLE[pcnt] ? 16'sd8192 : -16'sd8192;
            pcnt  <= pcnt + 6'd1;
            if (pcnt == 6'd63) begin
              tst         <= T_HDR;
              hcnt        <= '0;
              hdr         <= {hdr_hi, crc8_24(hdr_hi)};
              start_fetch <= 1'b1;
              fetch_en    <= 1'b1;
              sbn         <= '0;
            end
          end
          T_HDR: begin
            // header: one bit per symbol, BPSK on the diagonal (+3 dB vs QPSK bits)
            sym_i <= hdr[31] ? 16'sd8192 : -16'sd8192;
            sym_q <= hdr[31] ? 16'sd8192 : -16'sd8192;
            hdr   <= {hdr[30:0], 1'b0};
            hcnt  <= hcnt + 5'd1;
            if (hcnt == 5'd31) begin
              tst  <= T_PAY;
              scnt <= '0;
              nsym <= fmod ? 16'(nbytes_tot) << 1 : 16'(nbytes_tot) << 2;
            end
          end
          T_PAY: begin
            logic [7:0] cur;
            logic [3:0] cn;
            cur = (sbn == 0) ? nbyte : sb;
            cn  = (sbn == 0) ? 4'd8 : sbn;
            if (fmod) begin
              sym_i <= qam_lvl(cur[7:6]);
              sym_q <= qam_lvl(cur[5:4]);
              sb    <= {cur[3:0], 4'b0};
              sbn   <= cn - 4'd4;
            end else begin
              sym_i <= cur[7] ? 16'sd8192 : -16'sd8192;
              sym_q <= cur[6] ? 16'sd8192 : -16'sd8192;
              sb    <= {cur[5:0], 2'b0};
              sbn   <= cn - 4'd2;
            end
            scnt <= scnt + 16'd1;
            if (scnt == nsym - 16'd1) begin
              tst         <= T_IDLE;
              icnt        <= '0;
              fetch_en    <= 1'b0;
              release_buf <= 1'b1;
              seq         <= seq + 8'd1;
              frames_sent <= frames_sent + 32'd1;
            end
          end
          default: tst <= T_IDLE;
        endcase
      end
    end
  end
endmodule
