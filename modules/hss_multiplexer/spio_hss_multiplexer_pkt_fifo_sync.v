// -------------------------------------------------------------------------
// $Id: spinn_driver.v 2615 2013-10-02 10:39:58Z plana $
// -------------------------------------------------------------------------
// COPYRIGHT
// Copyright (c) The University of Manchester, 2012. All rights reserved.
// SpiNNaker Project
// Advanced Processor Technologies Group
// School of Computer Science
// -------------------------------------------------------------------------
// Project            : spiNNlink
// Module             : synchronization fifo
// Author             : lap
// Status             : Review pending
// $HeadURL$
// Last modified on   : $Date$
// Last modified by   : $Author$
// Version            : $Revision$
// -------------------------------------------------------------------------
// -------------------------------------------------------------------------
// TODO
// -------------------------------------------------------------------------


// ----------------------------------------------------------------
// include spiNNlink global constants and parameters
//
`include "../../modules/hss_multiplexer/spio_hss_multiplexer_common.h"
// ----------------------------------------------------------------

`timescale 1ns / 1ps
module spio_hss_multiplexer_pkt_fifo_sync
(
  input  wire                   RESET_IN,
  input  wire                   WCLK_IN,
  input  wire                   RCLK_IN,

  // synchronous input packet (write) interface
  input  wire [`PKT_BITS - 1:0] SFI_DATA_IN,
  input  wire                   SFI_VLD_IN,
  output reg                    SFI_RDY_OUT,

  // synchronous output packet (read) interface
  output reg  [`PKT_BITS - 1:0] SFO_DATA_OUT,
  output reg                    SFO_VLD_OUT,
  input  wire                   SFO_RDY_IN
);

  //---------------------------------------------------------------
  // constants
  //---------------------------------------------------------------
  localparam ADDR_WIDTH = 2;
  localparam BUFF_DEPTH = 4;


  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  // data buffer
  reg   [`PKT_BITS - 1:0] buffer [BUFF_DEPTH - 1:0];

  reg  [ADDR_WIDTH - 1:0] rdpg;    // read pointer
  wire [ADDR_WIDTH - 1:0] s_rdpg;  // synchronized read pointer
  reg  [ADDR_WIDTH - 1:0] wrpg;    // write pointer
  wire [ADDR_WIDTH - 1:0] s_wrpg;  // synchronized write pointer

  // buffer status
  reg  busy;
  reg  empty;
  reg  full;
  reg  nf;

  // buffer operations
  reg  vld_rd;
  reg  vld_wr;


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //-------------------- input packet interface -------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  always @(posedge WCLK_IN or posedge RESET_IN)
    if (RESET_IN)
      SFI_RDY_OUT <= 1'b1;
    else
      if (full || (nf && vld_wr))  // stall if full or going full
        SFI_RDY_OUT <= 1'b0;
      else
        SFI_RDY_OUT <= 1'b1;

  // write to the buffer if valid write
  always @ (posedge WCLK_IN)
    if (vld_wr)
      buffer[wrpg] <= SFI_DATA_IN;
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //------------------- output packet interface -------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  always @(posedge RCLK_IN or posedge RESET_IN)
    if (RESET_IN)
      SFO_VLD_OUT <= 1'b0;
    else
      if (vld_rd || busy)
        SFO_VLD_OUT <= 1'b1;
      else
        SFO_VLD_OUT <= 1'b0;

  // output data from buffer on valid read
  always @ (posedge RCLK_IN)
    if (vld_rd)
      SFO_DATA_OUT <= buffer[rdpg];
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //----------------------------- fifo ----------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // valid read operation
  always @ (*)
    vld_rd = !busy && !empty;

  // read pointer in gray code -- to cross clock boundary
  always @ (posedge RCLK_IN or posedge RESET_IN)
    if (RESET_IN)
      rdpg <= 0;
    else
      if (vld_rd)      // update pointer on valid read
        case (rdpg)
          2'b00: rdpg <= 2'b01;
          2'b01: rdpg <= 2'b11;
          2'b11: rdpg <= 2'b10;
          2'b10: rdpg <= 2'b00;
	endcase
      else
        rdpg <= rdpg;  // no change!
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // valid write operation
  always @ (*)
    vld_wr = SFI_VLD_IN && SFI_RDY_OUT;

  // write pointer in gray code -- to cross clock boundary
  always @ (posedge WCLK_IN or posedge RESET_IN)
    if (RESET_IN)
      wrpg <= 0;
    else
      if (vld_wr)      // update pointer on valid write
        case (wrpg)
          2'b00: wrpg <= 2'b01;
          2'b01: wrpg <= 2'b11;
          2'b11: wrpg <= 2'b10;
          2'b10: wrpg <= 2'b00;
	endcase
      else
        wrpg <= wrpg;  // no change!
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------- control --------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // synchronize the buffer pointers -- cross clock boundaries
  //---------------------------------------------------------------
  spio_spinnaker_link_sync #(.SIZE (ADDR_WIDTH)) rdp_sync
  (
    .CLK_IN (WCLK_IN),
    .IN     (rdpg),
    .OUT    (s_rdpg)
  );

  spio_spinnaker_link_sync #(.SIZE (ADDR_WIDTH)) wrp_sync
  (
    .CLK_IN (RCLK_IN),
    .IN     (wrpg),
    .OUT    (s_wrpg)
  );
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // output packet interface not ready to accept new packet
  always @(*)
    busy = SFO_VLD_OUT && !SFO_RDY_IN;

  // empty is used only by the read interface
  // rdpg and wrpg are gray encoded
  always @(*)
    empty = (rdpg == s_wrpg);

  // full is used only by the write interface
  // rdpg and wrpg are gray encoded
  always @(*)
    case ({wrpg, s_rdpg})
      4'b00_01,
      4'b01_11,
      4'b11_10,
      4'b10_00: full = #1 1'b1;
      default:  full = #1 1'b0;
    endcase

  // near full is used only by the write interface
  // rdpg and wrpg are gray encoded
  always @(*)
    case ({wrpg, s_rdpg})
      4'b00_11,
      4'b01_10,
      4'b11_00,
      4'b10_01: nf = #1 1'b1;
      default:  nf = #1 1'b0;
    endcase
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
endmodule
