/* nb_radio register map (AXI4-Lite, 32-bit). Default base in the reference BD: 0xA0040000 */
#ifndef NB_RADIO_REGS_H
#define NB_RADIO_REGS_H

#define NB_ID           0x00  /* RO  0x4E425231 "NBR1"                                   */
#define NB_VERSION      0x04  /* RO                                                      */
#define NB_CTRL         0x08  /* RW  see NB_CTRL_*                                       */
#define NB_RATE         0x0C  /* RW  [5:0] R = 8..63 -> Rs = 32/R MBd, BW = 40/R MHz     */
#define NB_CIC_SHIFT    0x10  /* RW  [5:0] rx = ceil(4 log2 R), [13:8] tx = ceil(3 log2 R) */
#define NB_TX_GAIN      0x14  /* RW  [15:0] Q2.14 digital gain before the RF-DAC          */
#define NB_TEST         0x18  /* RW  [11:0] test frame length, [31:16] min idle symbols    */
#define NB_AGC          0x1C  /* RW  [15:0] target (11650), [20:16] mu shift (31 = freeze) */
#define NB_TED          0x20  /* RW  [4:0] Kp shift, [12:8] Ki shift                       */
#define NB_PLL          0x24  /* RW  [4:0] Kp shift, [12:8] Ki shift                       */
#define NB_EQ           0x28  /* RW  [4:0] LMS mu shift                                    */
#define NB_DET          0x2C  /* RW  [7:0] detection threshold rho*256 (default 102=0.4)   */
#define NB_RX_OK        0x40  /* RO  frames with good CRC-16                               */
#define NB_RX_CRC_ERR   0x44  /* RO  frames with bad CRC-16                                */
#define NB_RX_HDR_ERR   0x48  /* RO  detections with bad header CRC-8                      */
#define NB_TX_FRAMES    0x4C  /* RO                                                        */
#define NB_RX_OVF       0x50  /* RO  bytes dropped (RX FIFO full)                          */
#define NB_PLL_FREQ     0x54  /* RO  s32, CFO = v / 2^32 * Rs  [Hz]                         */
#define NB_EVM_ACC      0x58  /* RO  sum |e|^2 >> 8 over last payload                       */
#define NB_EVM_N        0x5C  /* RO  symbols in last payload; EVM = ACC*256/N / (2*8192^2) */
#define NB_AGC_GAIN     0x60  /* RO  Q10.12 (1/256 .. 1024)                                 */
#define NB_PEAK_M       0x64  /* RO  rho = PEAK_M / PEAK_E of last detection               */
#define NB_PEAK_E       0x68  /* RO                                                        */
#define NB_LAST_HDR     0x6C  /* RO  mod[31:28] len[27:16] seq[15:8] crc8[7:0]             */
#define NB_TED_V        0x70  /* RO  s32 timing loop control (2^24 = 1 sample)             */
#define NB_DET_CNT      0x74  /* RO  preamble detections                                   */

#define NB_CTRL_TX_EN   (1u << 0)
#define NB_CTRL_RX_EN   (1u << 1)
#define NB_CTRL_TEST    (1u << 2)   /* internal frame generator instead of AXIS input      */
#define NB_CTRL_LPBK    (1u << 3)   /* digital loopback DAC->ADC inside the modem          */
#define NB_CTRL_SRST    (1u << 4)   /* datapath soft reset (self-clearing)                 */
#define NB_CTRL_EQ      (1u << 5)
#define NB_CTRL_PLL     (1u << 6)
#define NB_CTRL_16QAM   (1u << 7)   /* TX payload modulation (RX follows the header)       */

#endif
