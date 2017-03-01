// -------------------------------------------------------------------------
// $Id: synchronizer.v 2615 2013-10-02 10:39:58Z plana $
// -------------------------------------------------------------------------
// COPYRIGHT
// Copyright (c) The University of Manchester, 2012. All rights reserved.
// SpiNNaker Project
// Advanced Processor Technologies Group
// School of Computer Science
// -------------------------------------------------------------------------
// Project            : bidirectional SpiNNaker link to AER device interface
// Module             : simple cascaded flip-flop synchronizer
// Author             : lap/Jeff Pepper/Simon Davidson
// Status             : Review pending
// $HeadURL: https://solem.cs.man.ac.uk/svn/spinn_aer2_if/synchronizer.v $
// Last modified on   : $Date: 2013-10-02 11:39:58 +0100 (Wed, 02 Oct 2013) $
// Last modified by   : $Author: plana $
// Version            : $Revision: 2615 $
// -------------------------------------------------------------------------


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//------------------------- synchronizer ------------------------
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`timescale 1ns / 1ps
module synchronizer #
(
  parameter SIZE = 1,
  parameter DEPTH = 2
)
(
  input                   clk,
  input      [SIZE - 1:0] in,
  output reg [SIZE - 1:0] out
);

  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  reg  [SIZE - 1:0] sync [DEPTH - 2:0];

  genvar i;
   
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //--------------------------- datapath --------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // flops
  always @ (posedge clk)
    out <= sync[DEPTH - 2];

  always @ (posedge clk)
    sync[0] <= in;

  generate
    for (i = 0; i < (DEPTH - 2); i = i + 1)
    begin : flops
      always @ (posedge clk)
        sync[i + 1] <= sync[i];
    end
  endgenerate
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
endmodule
