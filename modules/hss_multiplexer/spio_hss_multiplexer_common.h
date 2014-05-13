/**
 * Common values used by spio_hss_multiplexer.
 */

`ifndef SPIO_HSS_MUTLIPLEXER_COMMON_H
`define SPIO_HSS_MUTLIPLEXER_COMMON_H

// Protocol Version Identifier (checked at time of handshake)
`define VERSION 8'h01

////////////////////////////////////////////////////////////////////////////////
// K-Characters
////////////////////////////////////////////////////////////////////////////////

// 8b/10b K28.0: Used for clock correction. A clock correction sequence consists
// of four of these.
`define KCH_CLKC 8'h1C

// 8b/10b K28.2: Used within the rx/tx_control handshake phase to uniqely
// identify the handshake words.
`define KCH_HANDSHAKE 8'h5C

// 8b/10b K28.5: Commma symbol used to ensure byte-allignment
`define KCH_COMMA 8'hBC

`define CLKC              8'h1c   // K28.0
`define COMMA             8'hbc   // K28.5

`define KCH_DATA          `COMMA
`define KCH_OOC           8'hf7   // K23.7
`define KCH_ACK           8'h7c   // K28.3
`define KCH_NAK           8'h9c   // K28.4
`define KCH_CFC           8'hfe   // K30.7

////////////////////////////////////////////////////////////////////////////////
// Protocol Constants
////////////////////////////////////////////////////////////////////////////////

`define ACK_T             1'b1
`define NAK_T             1'b0

////////////////////////////////////////////////////////////////////////////////
// Bit sizes and ranges
////////////////////////////////////////////////////////////////////////////////

`define NUM_CHANS        8

`define PKT_BITS         72
`define PKT_HDR_RNG       0 +: 8
`define PKT_KEY_RNG       8 +: 32
`define PKT_PLD_RNG      40 +: 32

`define FRM_BITS         32
`define FRM_BYTES        (`FRM_BITS / 8)
`define KCH_BITS         `FRM_BYTES

`define SEQ_BITS         7

`define CLR_BITS         1

`define CRDT_BITS        3
`define CRDT_CNT         {`CRDT_BITS {1'b1}}

`define BUF_BITS         3
`define BUF_LEN          (1 << `BUF_BITS)

`define CLKC_BITS        8
`define CLKC_CNT         {`CLKC_BITS {1'b1}}

`define CRC_BITS         16
`define CRC_PAD          {`CRC_BITS {1'b0}}

`define A_CNT_BITS       3
//#`define ACK_CNT          2
`define ACK_CNT          0

`define N_CNT_BITS       3
`define NAK_CNT          7

`define O_CNT_BITS       3
`define OOC_CNT          7

`define C_CNT_BITS       3
`define CFC_CNT          7

////////////////////////////////////////////////////////////////////////////////
// Frames
////////////////////////////////////////////////////////////////////////////////

`define FRM_KCH_RNG       31 -: 8
`define FRM_CLR_RNG       23 -: `CLR_BITS
`define FRM_SEQ_RNG       22 -: `SEQ_BITS
`define FRM_CRC_RNG        0 +: 16

`define DFRM_LEN_RNG      15 -: `NUM_CHANS
`define DFRM_PRE_RNG       0 +: `NUM_CHANS
`define DFRM_CFC_RNG      23 -: `NUM_CHANS

`define DFRM_HD0_RNG       0 +: 8
`define DFRM_HD1_RNG       8 +: 8
`define DFRM_HD2_RNG      16 +: 8
`define DFRM_HD3_RNG      24 +: 8
`define DFRM_HD4_RNG       0 +: 8
`define DFRM_HD5_RNG       8 +: 8
`define DFRM_HD6_RNG      16 +: 8
`define DFRM_HD7_RNG      24 +: 8

`define DATA_KBITS       4'b1000
`define ACK_KBITS        4'b1000
`define NAK_KBITS        4'b1000
`define OOC_KBITS        4'b1000
`define CFC_KBITS        4'b1000

`define CLKC_KBITS       4'b0001
`define ZERO_KBITS       {`KCH_BITS {1'b0}}

// XXX: Clock correction now done by tx_control/rx_control, should eventually be
// removed.
`define CLKC_FRM         {`FRM_BYTES {`KCH_CLKC}}

// XXX: Now sent by tx_control/rx_control, should eventually be removed.
`define SYNC_FRM         {`KCH_COMMA , `KCH_HANDSHAKE , 8'h01 , `VERSION}
`define SYNC_KBITS       4'b1100

`define ZERO_FRM         {`FRM_BITS {1'b0}}


`endif
