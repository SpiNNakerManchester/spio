// -------------------------------------------------------------------------
// $Id$
//  spiNNlink pseudo-random number generation module
//  lfsr with polynomial: x16 + x15 + x13 + x4 + 1
//
// -------------------------------------------------------------------------
// AUTHOR
//  lap - luis.plana@manchester.ac.uk
//
// -------------------------------------------------------------------------
// DETAILS
//  Created on       : 28 Nov 2012
//  Version          : $Revision$
//  Last modified on : $Date$
//  Last modified by : $Author$
//  $HeadURL$
//
// -------------------------------------------------------------------------
// COPYRIGHT
//  Copyright (c) The University of Manchester, 2012-2016.
//  SpiNNaker Project
//  Advanced Processor Technologies Group
//  School of Computer Science
// -------------------------------------------------------------------------
// TODO
// -------------------------------------------------------------------------


// -------------------------------------------------------------------------
// include spiNNlink global constants and parameters
//
`include "spio_hss_multiplexer_common.h"
// -------------------------------------------------------------------------

`timescale 1ns / 1ps
module spio_hss_multiplexer_lfsr
(
  input wire 	    clk,
  input wire 	    rst,

  // generate next pseudo-random number
  input wire 	    next,

  // pseudo-random data output
  output reg [15:0] data_out
);

  reg [15:0] lfsr;
  reg [15:0] nxt_lfsr;
  reg [15:0] nxt_data;

  always @(*)
  begin
    nxt_lfsr[0]  = lfsr[0] ^ lfsr[1] ^ lfsr[2]  ^ lfsr[4]  ^ lfsr[7]
                     ^ lfsr[8]  ^ lfsr[9]  ^ lfsr[11] ^ lfsr[12] ^ lfsr[15];
    nxt_lfsr[1]  = lfsr[1] ^ lfsr[2] ^ lfsr[3]  ^ lfsr[5]  ^ lfsr[8]
                     ^ lfsr[9]  ^ lfsr[10] ^ lfsr[12] ^ lfsr[13];
    nxt_lfsr[2]  = lfsr[2] ^ lfsr[3] ^ lfsr[4]  ^ lfsr[6]  ^ lfsr[9]
                     ^ lfsr[10] ^ lfsr[11] ^ lfsr[13] ^ lfsr[14];
    nxt_lfsr[3]  = lfsr[3] ^ lfsr[4] ^ lfsr[5]  ^ lfsr[7]  ^ lfsr[10]
                     ^ lfsr[11] ^ lfsr[12] ^ lfsr[14] ^ lfsr[15];
    nxt_lfsr[4]  = lfsr[0] ^ lfsr[1] ^ lfsr[2]  ^ lfsr[5]  ^ lfsr[6]
                     ^ lfsr[7]  ^ lfsr[9]  ^ lfsr[13];
    nxt_lfsr[5]  = lfsr[1] ^ lfsr[2] ^ lfsr[3]  ^ lfsr[6]  ^ lfsr[7]
                     ^ lfsr[8]  ^ lfsr[10] ^ lfsr[14];
    nxt_lfsr[6]  = lfsr[2] ^ lfsr[3] ^ lfsr[4]  ^ lfsr[7]  ^ lfsr[8]
                     ^ lfsr[9]  ^ lfsr[11] ^ lfsr[15];
    nxt_lfsr[7]  = lfsr[3] ^ lfsr[4] ^ lfsr[5]  ^ lfsr[8]  ^ lfsr[9]
                     ^ lfsr[10] ^ lfsr[12];
    nxt_lfsr[8]  = lfsr[4] ^ lfsr[5] ^ lfsr[6]  ^ lfsr[9]  ^ lfsr[10]
                     ^ lfsr[11] ^ lfsr[13];
    nxt_lfsr[9]  = lfsr[5] ^ lfsr[6] ^ lfsr[7]  ^ lfsr[10] ^ lfsr[11]
                     ^ lfsr[12] ^ lfsr[14];
    nxt_lfsr[10] = lfsr[6] ^ lfsr[7] ^ lfsr[8]  ^ lfsr[11] ^ lfsr[12]
                     ^ lfsr[13] ^ lfsr[15];
    nxt_lfsr[11] = lfsr[7] ^ lfsr[8] ^ lfsr[9]  ^ lfsr[12] ^ lfsr[13]
                     ^ lfsr[14];
    nxt_lfsr[12] = lfsr[8] ^ lfsr[9] ^ lfsr[10] ^ lfsr[13] ^ lfsr[14]
                     ^ lfsr[15];
    nxt_lfsr[13] = lfsr[0] ^ lfsr[1] ^ lfsr[2]  ^ lfsr[4]  ^ lfsr[7]
                     ^ lfsr[8]  ^ lfsr[10] ^ lfsr[12] ^ lfsr[14];
    nxt_lfsr[14] = lfsr[1] ^ lfsr[2] ^ lfsr[3]  ^ lfsr[5]  ^ lfsr[8]
                     ^ lfsr[9]  ^ lfsr[11] ^ lfsr[13] ^ lfsr[15];
    nxt_lfsr[15] = lfsr[0] ^ lfsr[1] ^ lfsr[3]  ^ lfsr[6]  ^ lfsr[7]
                     ^ lfsr[8]  ^ lfsr[10] ^ lfsr[11] ^ lfsr[14] ^ lfsr[15];
  end

  always @(*)
  begin
    nxt_data[0]  = lfsr[15];
    nxt_data[1]  = lfsr[14] ^ lfsr[15];
    nxt_data[2]  = lfsr[13] ^ lfsr[14] ^ lfsr[15];
    nxt_data[3]  = lfsr[12] ^ lfsr[13] ^ lfsr[14];
    nxt_data[4]  = lfsr[11] ^ lfsr[12] ^ lfsr[13] ^ lfsr[15];
    nxt_data[5]  = lfsr[10] ^ lfsr[11] ^ lfsr[12] ^ lfsr[14];
    nxt_data[6]  = lfsr[9]  ^ lfsr[10] ^ lfsr[11] ^ lfsr[13];
    nxt_data[7]  = lfsr[8]  ^ lfsr[9]  ^ lfsr[10] ^ lfsr[12] ^ lfsr[15];
    nxt_data[8]  = lfsr[7]  ^ lfsr[8]  ^ lfsr[9]  ^ lfsr[11] ^ lfsr[14]
                     ^ lfsr[15];
    nxt_data[9]  = lfsr[6]  ^ lfsr[7]  ^ lfsr[8]  ^ lfsr[10] ^ lfsr[13]
                     ^ lfsr[14] ^ lfsr[15];
    nxt_data[10] = lfsr[5]  ^ lfsr[6]  ^ lfsr[7]  ^ lfsr[9]  ^ lfsr[12]
                     ^ lfsr[13] ^ lfsr[14];
    nxt_data[11] = lfsr[4]  ^ lfsr[5]  ^ lfsr[6]  ^ lfsr[8]  ^ lfsr[11]
                     ^ lfsr[12] ^ lfsr[13] ^ lfsr[15];
    nxt_data[12] = lfsr[3]  ^ lfsr[4]  ^ lfsr[5]  ^ lfsr[7]  ^ lfsr[10]
                     ^ lfsr[11] ^ lfsr[12] ^ lfsr[14] ^ lfsr[15];
    nxt_data[13] = lfsr[2]  ^ lfsr[3]  ^ lfsr[4]  ^ lfsr[6]  ^ lfsr[9]
                     ^ lfsr[10] ^ lfsr[11] ^ lfsr[13] ^ lfsr[14];
    nxt_data[14] = lfsr[1]  ^ lfsr[2]  ^ lfsr[3]  ^ lfsr[5]  ^ lfsr[8]
                     ^ lfsr[9]  ^ lfsr[10] ^ lfsr[12] ^ lfsr[13];
    nxt_data[15] = lfsr[0]  ^ lfsr[1]  ^ lfsr[2]  ^ lfsr[4]  ^ lfsr[7]
                     ^ lfsr[8]  ^ lfsr[9]  ^ lfsr[11] ^ lfsr[12] ^ lfsr[15];
  end

  always @ (posedge clk or posedge rst)
    if (rst)
      lfsr = 16'hffff;  // conventional initialization
    else
      if (next)
        lfsr = nxt_lfsr;

  always @ (posedge clk or posedge rst)
    if (rst)
      data_out = 16'h0000;  // conventional initialization
    else
      if (next)
        data_out = nxt_data;
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
endmodule
