// -------------------------------------------------------------------------
// $Id: spiNNlink_definitions.v 2515 2013-08-19 07:50:12Z plana $
//  spiNNlink global constants and parameters
//
// -------------------------------------------------------------------------
// AUTHOR
//  lap - luis.plana@manchester.ac.uk
//
// -------------------------------------------------------------------------
// Taken from:
// https://solem.cs.man.ac.uk/svn/spiNNlink/testing/src/spiNNlink_definitions.v
// Revision 2515 (Last-modified date: Date: 2013-08-19 08:50:12 +0100)
//
// -------------------------------------------------------------------------
// COPYRIGHT
//  Copyright (c) The University of Manchester, 2012. All rights reserved.
//  SpiNNaker Project
//  Advanced Processor Technologies Group
//  School of Computer Science
// -------------------------------------------------------------------------

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`ifndef SCD_DEF
`define SCD_DEF
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


// ----------------------------------------------------------------
// ---------------------------- globals ---------------------------
// ----------------------------------------------------------------
`define NUM_CHANS         8

`define ACK_T             1'b1
`define NAK_T             1'b0


// ----------------------------------------------------------------
// ---------------------- bit sizes and ranges --------------------
// ----------------------------------------------------------------
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

// ----------------------------------------------------------------
// --------------------- K characters and frames ------------------
// ----------------------------------------------------------------
`define CLKC              8'h1c   // K28.0
`define SYNC              8'h5c   // K28.2
`define COMMA             8'hbc   // K28.5

`define KCH_DATA          `COMMA
`define KCH_CLKC          `CLKC
`define KCH_OOC           8'hf7   // K23.7
`define KCH_ACK           8'h7c   // K28.3
`define KCH_NAK           8'h9c   // K28.4
`define KCH_CFC           8'hfe   // K30.7

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
`define SYNC_KBITS       4'b1010
`define ZERO_KBITS       {`KCH_BITS {1'b0}}

`define CLKC_FRM         {`FRM_BYTES {`KCH_CLKC}}
`define SYNC_FRM         {`COMMA, 8'h03, `SYNC, 8'h00}
`define ZERO_FRM         {`FRM_BITS {1'b0}}


// ----------------------------------------------------------------
// --------------------------- registers --------------------------
// ----------------------------------------------------------------

`define VERSION          32'd14021300

`define REGA_BITS        5
`define REGD_BITS       32

// Version of the interface
`define VERS_REG         0

// Count of frames with a CRC error
`define CRCE_REG         1

// Count of frames containing some other error
`define FRME_REG         2

// Frames rejected while busy
`define BUSY_REG         3
`define LNAK_REG         4
`define RNAK_REG         5
`define LACK_REG         6
`define RACK_REG         7
`define LOOC_REG         8
`define ROOC_REG         9
`define CRDT_REG         10
`define SFRM_REG         11
`define TFRM_REG         12
`define DFRM_REG         13
`define RFRM_REG         14
`define EMPT_REG         15
`define FULL_REG         16
`define CFCL_REG         17
`define CFCR_REG         18

`define PKT0_CTR         0
`define PKT1_CTR         1
`define PKT2_CTR         2
`define PKT3_CTR         3
`define PKT4_CTR         4
`define PKT5_CTR         5
`define PKT6_CTR         6
`define PKT7_CTR         7
`define PKT8_CTR         8
`define PKT9_CTR         9
`define PKT10_CTR        10
`define PKT11_CTR        11
`define PKT12_CTR        12
`define PKT13_CTR        13
`define PKT14_CTR        14
`define PKT15_CTR        15


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

