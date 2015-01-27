// -------------------------------------------------------------------------
//  $Id$
// spiNNlink lfsr testbench
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
//  Copyright (c) The University of Manchester, 2012. All rights reserved.
//  SpiNNaker Project
//  Advanced Processor Technologies Group
//  School of Computer Science
// -------------------------------------------------------------------------


// ----------------------------------------------------------------
// include spiNNlink global constants and parameters
//
`include "spio_hss_multiplexer_common.h"
// ----------------------------------------------------------------

`timescale 1ns / 1ps
module spio_hss_multiplexer_lfsr_tb;
  // constants

  // ----------------------------------
  // internal signals
  // ----------------------------------

  // clock signals
  reg  tb_clk;   // tb
  reg  uut_clk;  // unit under test

  // reset signals
  reg  tb_rst;   // tb
  reg  uut_rst;  // unit under test

  // signals to drive the uut
  reg         uut_next;
  wire [15:0] uut_data;


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------- unit under test ------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  spio_hss_multiplexer_lfsr uut
  (
    .clk      (uut_clk),
    .rst      (uut_rst),

    .next     (uut_next),
    .data_out (uut_data)
  );
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //----------------------- drive the uut -------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  always @(posedge tb_clk or posedge tb_rst)
    if (tb_rst)
      uut_next = 1'b0;
    else
      uut_next = 1'b1;
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //----------------- clocks, resets and counters -----------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // generate clocks
  //---------------------------------------------------------------
  initial
  begin
    tb_clk = 1'b0;
    forever #6.66666667 tb_clk = ~tb_clk;
  end

  initial
  begin
    uut_clk = 1'b0;
    forever #6.66666667 uut_clk = ~uut_clk;
  end
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // generate resets
  //---------------------------------------------------------------
  initial
  begin
    tb_rst  = 1'b1;
    #50
    tb_rst  = 1'b0;
  end

  initial
  begin
    uut_rst  = 1'b1;
    #50
    uut_rst  = 1'b0;
  end
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
endmodule
