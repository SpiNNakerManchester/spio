// -------------------------------------------------------------------------
// $Id: pkt_ctr.v 2517 2013-08-19 09:33:30Z plana $
//  spiNNlink packet counter module
//
// -------------------------------------------------------------------------
// AUTHOR
//  lap - luis.plana@manchester.ac.uk
//
// -------------------------------------------------------------------------
// DETAILS
//  Created on       : 28 Mar 2013
//  Version          : $Revision: 2517 $
//  Last modified on : $Date: 2013-08-19 10:33:30 +0100 (Mon, 19 Aug 2013) $
//  Last modified by : $Author: plana $
//  $HeadURL: https://solem.cs.man.ac.uk/svn/spiNNlink/testing/src/pkt_ctr.v $
//
// -------------------------------------------------------------------------
// COPYRIGHT
//  Copyright (c) The University of Manchester, 2012. All rights reserved.
//  SpiNNaker Project
//  Advanced Processor Technologies Group
//  School of Computer Science
// -------------------------------------------------------------------------
// TODO
// -------------------------------------------------------------------------


// ----------------------------------------------------------------
// include spiNNlink global constants and parameters
//
`include "spio_packet_counter.h"
// ----------------------------------------------------------------

`timescale 1ns / 1ps
module spio_packet_counter
(
  input  wire 			 CLK_IN,
  input  wire 			 RESET_IN,

  // packet interface
  input  wire             [15:0] pkt_vld,
  input  wire             [15:0] pkt_rdy,

  // error interface
  input  wire             [15:0] flt_err,
  input  wire             [15:0] frm_err,

  // register access interface
  input  wire [`CTRA_BITS - 1:0] ctr_addr,
  output reg  [`CTRD_BITS - 1:0] ctr_data
);

  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  reg  [`CTRD_BITS - 1:0] pkt_ctr [15:0];
  reg  [`CTRD_BITS - 1:0] fle_ctr [15:0];
  reg  [`CTRD_BITS - 1:0] fre_ctr [15:0];


  //---------------------------------------------------------------
  // packet and error counters
  //---------------------------------------------------------------
  genvar i;

  generate for (i = 0; i < 16; i = i + 1)
  begin : counters
    always @ (posedge CLK_IN or posedge RESET_IN)
      if (RESET_IN)
        pkt_ctr[i] <= 1'b0;
      else
        if (pkt_vld[i] && pkt_rdy[i])
          pkt_ctr[i] <= pkt_ctr[i] + 1;

    always @ (posedge CLK_IN or posedge RESET_IN)
      if (RESET_IN)
        fle_ctr[i] <= 1'b0;
      else
        if (flt_err[i])
          fle_ctr[i] <= fle_ctr[i] + 1;

    always @ (posedge CLK_IN or posedge RESET_IN)
      if (RESET_IN)
        fre_ctr[i] <= 1'b0;
      else
        if (frm_err[i])
          fre_ctr[i] <= fre_ctr[i] + 1;
  end
  endgenerate
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // register output selection
  //---------------------------------------------------------------
  always @ (*)
    case (ctr_addr[5:4])
      `PKT_CNT_BITS: ctr_data = pkt_ctr[ctr_addr[3:0]];
      `FLT_ERR_BITS: ctr_data = fle_ctr[ctr_addr[3:0]];
      `FRM_ERR_BITS: ctr_data = fre_ctr[ctr_addr[3:0]];
      default:       ctr_data = {`CTRD_BITS {1'b1}};
    endcase
  //---------------------------------------------------------------
endmodule
