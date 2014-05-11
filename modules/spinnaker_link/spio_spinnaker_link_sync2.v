// -------------------------------------------------------------------------
//  $Id$
// spiNNlink 2-flop synchronizer with separate clocks
//
// -------------------------------------------------------------------------
// AUTHOR
//  lap - luis.plana@manchester.ac.uk
//
// -------------------------------------------------------------------------
// DETAILS
//  Created on       : 29 Nov 2013
//  Version          : $Revision$
//  Last modified on : $Date$
//  Last modified by : $Author$
//  $HeadURL$
//
// -------------------------------------------------------------------------
// COPYRIGHT
//  Copyright (c) The University of Manchester, 2013. All rights reserved.
//  SpiNNaker Project
//  Advanced Processor Technologies Group
//  School of Computer Science
// -------------------------------------------------------------------------
//
// TODO
// -------------------------------------------------------------------------


`timescale 1ns / 1ps
module spio_spinnaker_link_sync2 #
(
  parameter SIZE = 1
)
(
  input                   clk0,
  input                   clk1,
  input      [SIZE - 1:0] in,
  output reg [SIZE - 1:0] out
);

  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  reg  [SIZE - 1:0] sync;

   
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //--------------------------- datapath --------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // flops
  always @ (posedge clk0)
       sync <= in;

  always @ (posedge clk1)
       out  <= sync;
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
endmodule
