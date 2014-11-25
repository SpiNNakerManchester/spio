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

  // packet interfaces
  input  wire                    pkt_vld0,
  input  wire                    pkt_rdy0,
  input  wire                    pkt_vld1,
  input  wire                    pkt_rdy1,
  input  wire                    pkt_vld2,
  input  wire                    pkt_rdy2,
  input  wire                    pkt_vld3,
  input  wire                    pkt_rdy3,
  input  wire                    pkt_vld4,
  input  wire                    pkt_rdy4,
  input  wire                    pkt_vld5,
  input  wire                    pkt_rdy5,
  input  wire                    pkt_vld6,
  input  wire                    pkt_rdy6,
  input  wire                    pkt_vld7,
  input  wire                    pkt_rdy7,
  input  wire                    pkt_vld8,
  input  wire                    pkt_rdy8,
  input  wire                    pkt_vld9,
  input  wire                    pkt_rdy9,
  input  wire                    pkt_vld10,
  input  wire                    pkt_rdy10,
  input  wire                    pkt_vld11,
  input  wire                    pkt_rdy11,
  input  wire                    pkt_vld12,
  input  wire                    pkt_rdy12,
  input  wire                    pkt_vld13,
  input  wire                    pkt_rdy13,
  input  wire                    pkt_vld14,
  input  wire                    pkt_rdy14,
  input  wire                    pkt_vld15,
  input  wire                    pkt_rdy15,

  // register access interface
  input  wire [`CTRA_BITS - 1:0] ctr_addr,
  output reg  [`CTRD_BITS - 1:0] ctr_data
);

  //---------------------------------------------------------------
  // constants
  //---------------------------------------------------------------
   

  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  reg  [`CTRD_BITS - 1:0] pkt0_ctr;
  reg  [`CTRD_BITS - 1:0] pkt1_ctr;
  reg  [`CTRD_BITS - 1:0] pkt2_ctr;
  reg  [`CTRD_BITS - 1:0] pkt3_ctr;
  reg  [`CTRD_BITS - 1:0] pkt4_ctr;
  reg  [`CTRD_BITS - 1:0] pkt5_ctr;
  reg  [`CTRD_BITS - 1:0] pkt6_ctr;
  reg  [`CTRD_BITS - 1:0] pkt7_ctr;
  reg  [`CTRD_BITS - 1:0] pkt8_ctr;
  reg  [`CTRD_BITS - 1:0] pkt9_ctr;
  reg  [`CTRD_BITS - 1:0] pkt10_ctr;
  reg  [`CTRD_BITS - 1:0] pkt11_ctr;
  reg  [`CTRD_BITS - 1:0] pkt12_ctr;
  reg  [`CTRD_BITS - 1:0] pkt13_ctr;
  reg  [`CTRD_BITS - 1:0] pkt14_ctr;
  reg  [`CTRD_BITS - 1:0] pkt15_ctr;


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //--------------------------- datapath --------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // packet counters
  //---------------------------------------------------------------
  always @ (posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      pkt0_ctr <= 1'b0;
    else
      if (pkt_vld0 && pkt_rdy0)
        pkt0_ctr <= pkt0_ctr + 1;

  always @ (posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      pkt1_ctr <= 1'b0;
    else
      if (pkt_vld1 && pkt_rdy1)
        pkt1_ctr <= pkt1_ctr + 1;

  always @ (posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      pkt2_ctr <= 1'b0;
    else
      if (pkt_vld2 && pkt_rdy2)
        pkt2_ctr <= pkt2_ctr + 1;

  always @ (posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      pkt3_ctr <= 1'b0;
    else
      if (pkt_vld3 && pkt_rdy3)
        pkt3_ctr <= pkt3_ctr + 1;

  always @ (posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      pkt4_ctr <= 1'b0;
    else
      if (pkt_vld4 && pkt_rdy4)
        pkt4_ctr <= pkt4_ctr + 1;

  always @ (posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      pkt5_ctr <= 1'b0;
    else
      if (pkt_vld5 && pkt_rdy5)
        pkt5_ctr <= pkt5_ctr + 1;

  always @ (posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      pkt6_ctr <= 1'b0;
    else
      if (pkt_vld6 && pkt_rdy6)
        pkt6_ctr <= pkt6_ctr + 1;

  always @ (posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      pkt7_ctr <= 1'b0;
    else
      if (pkt_vld7 && pkt_rdy7)
        pkt7_ctr <= pkt7_ctr + 1;

  always @ (posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      pkt8_ctr <= 1'b0;
    else
      if (pkt_vld8 && pkt_rdy8)
        pkt8_ctr <= pkt8_ctr + 1;

  always @ (posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      pkt9_ctr <= 1'b0;
    else
      if (pkt_vld9 && pkt_rdy9)
        pkt9_ctr <= pkt9_ctr + 1;

  always @ (posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      pkt10_ctr <= 1'b0;
    else
      if (pkt_vld10 && pkt_rdy10)
        pkt10_ctr <= pkt10_ctr + 1;

  always @ (posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      pkt11_ctr <= 1'b0;
    else
      if (pkt_vld11 && pkt_rdy11)
        pkt11_ctr <= pkt11_ctr + 1;

  always @ (posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      pkt12_ctr <= 1'b0;
    else
      if (pkt_vld12 && pkt_rdy12)
        pkt12_ctr <= pkt12_ctr + 1;

  always @ (posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      pkt13_ctr <= 1'b0;
    else
      if (pkt_vld13 && pkt_rdy13)
        pkt13_ctr <= pkt13_ctr + 1;

  always @ (posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      pkt14_ctr <= 1'b0;
    else
      if (pkt_vld14 && pkt_rdy14)
        pkt14_ctr <= pkt14_ctr + 1;

  always @ (posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      pkt15_ctr <= 1'b0;
    else
      if (pkt_vld15 && pkt_rdy15)
        pkt15_ctr <= pkt15_ctr + 1;
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // register output selection
  //---------------------------------------------------------------
  always @ (*)
    case (ctr_addr)
      `PKT0_CTR:  ctr_data = pkt0_ctr;
      `PKT1_CTR:  ctr_data = pkt1_ctr;
      `PKT2_CTR:  ctr_data = pkt2_ctr;
      `PKT3_CTR:  ctr_data = pkt3_ctr;
      `PKT4_CTR:  ctr_data = pkt4_ctr;
      `PKT5_CTR:  ctr_data = pkt5_ctr;
      `PKT6_CTR:  ctr_data = pkt6_ctr;
      `PKT7_CTR:  ctr_data = pkt7_ctr;
      `PKT8_CTR:  ctr_data = pkt8_ctr;
      `PKT9_CTR:  ctr_data = pkt9_ctr;
      `PKT10_CTR: ctr_data = pkt10_ctr;
      `PKT11_CTR: ctr_data = pkt11_ctr;
      `PKT12_CTR: ctr_data = pkt12_ctr;
      `PKT13_CTR: ctr_data = pkt13_ctr;
      `PKT14_CTR: ctr_data = pkt14_ctr;
      `PKT15_CTR: ctr_data = pkt15_ctr;
      default:    ctr_data = {`CTRD_BITS {1'b1}};
    endcase
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

endmodule
