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
//  Copyright (c) The University of Manchester, 2013-2016.
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
  input                   CLK0_IN,
  input                   CLK1_IN,
  input      [SIZE - 1:0] IN,
  output reg [SIZE - 1:0] OUT
);

  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  reg  [SIZE - 1:0] sync;

   
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //--------------------------- datapath --------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // flops
  always @ (posedge CLK0_IN)
       sync <= IN;

  always @ (posedge CLK1_IN)
       OUT  <= sync;
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
endmodule
