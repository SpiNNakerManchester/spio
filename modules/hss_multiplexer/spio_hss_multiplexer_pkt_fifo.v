// -------------------------------------------------------------------------
// $Id$
//  spiNNlink packet dispatch fifo module
//
// -------------------------------------------------------------------------
// AUTHOR
//  lap - luis.plana@manchester.ac.uk
//
// -------------------------------------------------------------------------
// DETAILS
//  Created on       : 28 Nov 2013
//  Version          : $Revision$
//  Last modified on : $Date$
//  Last modified by : $Author$
//  $HeadURL$
//
// -------------------------------------------------------------------------
// COPYRIGHT
//  Copyright (c) The University of Manchester, 2013-2016.
//  SpiNNaker Project
//  Advanced Processor Technologies Group
//  School of Computer Science
// -------------------------------------------------------------------------
// TODO
//  * fix operation for FIFO_DEPTH == 1 (FIFO_BITS = 0)
// -------------------------------------------------------------------------


// ----------------------------------------------------------------
// include spiNNlink global constants and parameters
//
`include "spio_hss_multiplexer_common.h"
// ----------------------------------------------------------------

`timescale 1ns / 1ps
module spio_hss_multiplexer_pkt_fifo
(
  input  wire 			 clk,
  input  wire 			 rst,

  // control and status
  input  wire                    go_frm,
  output reg                     busy,
  output reg                     cfcf,

  // incoming packet
  input  wire  [`PKT_BITS - 1:0] ipkt_data,
  input  wire 			 ipkt_vld,

  // output packet
  output reg   [`PKT_BITS - 1:0] pkt_data,
  output reg  			 pkt_vld,
  input  wire 			 pkt_rdy
);

  //---------------------------------------------------------------
  // constants
  //---------------------------------------------------------------
  localparam FIFO_BITS = 3;
  localparam FIFO_DEPTH  = 1 << FIFO_BITS;

  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  reg  [`PKT_BITS - 1:0] fifo [0 : FIFO_DEPTH - 1];
  reg  [FIFO_BITS - 1:0] wtp;
  reg  [FIFO_BITS - 1:0] rdp;
  reg      [FIFO_BITS:0] occ;
  reg      [FIFO_BITS:0] nxt_occ;
   
  reg                    writing;
  reg                    reading;
  reg                    full;
  reg                    empty;

  reg                    pkt_vld_int;
  reg                    pkt_rdy_int;

  //---------------------------------------------------------------
  // fifo operations and flags
  //---------------------------------------------------------------
  always @ (*)
    writing = go_frm && ipkt_vld;  // write request will succeed

  always @ (*)
    reading = pkt_vld_int && pkt_rdy_int;  // read request will succeed

  always @ (posedge clk)
    if (writing)
      fifo[wtp] <= ipkt_data;

  always @ (posedge clk or posedge rst)
    if (rst)
      wtp <= 0;
    else
      if (writing)
	wtp <= wtp + 1;
      else
        wtp <= wtp;  // no change

  always @ (posedge clk or posedge rst)
    if (rst)
      rdp <= 0;
    else
      if (reading)
	rdp <= rdp + 1;
      else
        rdp <= rdp;  // no change

  always @ (posedge clk or posedge rst)
    if (rst)
      full <= 1'b0;
    else
      full <= (nxt_occ == FIFO_DEPTH);

  always @ (posedge clk or posedge rst)
    if (rst)
      empty <= 1'b1;
    else
      empty <= (nxt_occ == 0);

  always @ (*)
    case ({reading, writing})
      2'b10:   nxt_occ = occ - 1;
      2'b01:   nxt_occ = occ + 1;
      default: nxt_occ = occ;  // no change!
    endcase

  always @ (posedge clk or posedge rst)
    if (rst)
      occ <= 1'b0;
    else
      occ <= nxt_occ;
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // output packet interface
  //NOTE: registered pkt_data would allow the use of block RAM
  //---------------------------------------------------------------
  always @ (*)
    pkt_data = fifo[rdp];

  always @ (*)
    pkt_vld = pkt_vld_int;

  always @ (*)
    pkt_rdy_int = pkt_rdy;

  // output packet valid if not empty
  always @ (*)
    pkt_vld_int = !empty;
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // status
  //---------------------------------------------------------------
  // cannot accept a new packet (combinatorial)
  always @ (*)
    busy = full && ipkt_vld && !pkt_rdy_int;

  // control packet flow (stop when near full)
  always @ (posedge clk or posedge rst)
    if (rst)
      cfcf <= 1'b1;
    else
      if (nxt_occ >= (3 * FIFO_DEPTH / 4))
	cfcf <= 1'b0;
      else if (nxt_occ <= (FIFO_DEPTH / 4))
	cfcf <= 1'b1;
      else
	cfcf <= cfcf;  // no change!
  //---------------------------------------------------------------
endmodule
