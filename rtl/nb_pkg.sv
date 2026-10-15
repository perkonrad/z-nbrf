// nb_radio common definitions
package nb_pkg;

  // Constellation scale (16-bit domain). QPSK points (+-S, +-S); 16-QAM levels +-D1, +-3*D1
  // with equal average power (D1 = S/sqrt(5)).
  localparam int S_QPSK = 8192;
  localparam int D1_QAM = 3664;

  localparam logic [31:0] NB_ID      = 32'h4E42_5231;  // "NBR1"
  localparam logic [31:0] NB_VERSION = 32'h0001_0000;

  localparam int PRE_LEN = 64;
  localparam int HDR_SYM = 32;

  // CRC-8 (poly 0x07, init 0x00) over 24 header bits, MSB first
  function automatic logic [7:0] crc8_24(input logic [23:0] d);
    logic [7:0] c;
    c = 8'h00;
    for (int i = 23; i >= 0; i--) begin
      logic fb;
      fb = c[7] ^ d[i];
      c  = {c[6:0], 1'b0};
      if (fb) c = c ^ 8'h07;
    end
    return c;
  endfunction

  // CRC-16-CCITT (poly 0x1021), one byte, MSB first
  function automatic logic [15:0] crc16_byte(input logic [15:0] c_in, input logic [7:0] d);
    logic [15:0] c;
    c = c_in;
    for (int i = 7; i >= 0; i--) begin
      logic fb;
      fb = c[15] ^ d[i];
      c  = {c[14:0], 1'b0};
      if (fb) c = c ^ 16'h1021;
    end
    return c;
  endfunction

  // PRBS15 (1 + x^14 + x^15) additive scrambler, 8 steps. Returns {new_state, keystream byte}
  // keystream bit order: first generated bit -> byte MSB.
  function automatic logic [22:0] prbs15_byte(input logic [14:0] s_in);
    logic [14:0] s;
    logic [7:0]  k;
    s = s_in;
    for (int i = 7; i >= 0; i--) begin
      logic b;
      b    = s[14] ^ s[13];
      k[i] = b;
      s    = {s[13:0], b};
    end
    return {s, k};
  endfunction

  localparam logic [14:0] PRBS15_INIT = 15'b100101010000000;

  // 16-QAM Gray level for 2 bits: 00:-3 01:-1 11:+1 10:+3
  function automatic logic signed [15:0] qam_lvl(input logic [1:0] b);
    case (b)
      2'b00:   return -16'(3 * D1_QAM);
      2'b01:   return -16'(D1_QAM);
      2'b11:   return 16'(D1_QAM);
      default: return 16'(3 * D1_QAM);
    endcase
  endfunction

endpackage
