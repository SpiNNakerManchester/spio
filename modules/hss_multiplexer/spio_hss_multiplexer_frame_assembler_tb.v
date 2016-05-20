// -------------------------------------------------------------------------
//  $Id: frame_assembler_tb.v 3085 2014-02-06 13:39:37Z plana $
// spiNNlink frame assembler testbench
//
// -------------------------------------------------------------------------
// AUTHOR
//  lap - luis.plana@manchester.ac.uk
//  Based on work by J Pepper (Date 08/08/2012)
//
// -------------------------------------------------------------------------
// DETAILS
//  Created on       : 28 Nov 2012
//  Version          : $Revision: 3085 $
//  Last modified on : $Date: 2014-02-06 13:39:37 +0000 (Thu, 06 Feb 2014) $
//  Last modified by : $Author: plana $
//  $HeadURL: https://solem.cs.man.ac.uk/svn/spiNNlink/testing/testbench/frame_assembler_tb.v $
//
// -------------------------------------------------------------------------
// COPYRIGHT
//  Copyright (c) The University of Manchester, 2012-2016.
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
module spio_hss_multiplexer_frame_assembler_tb;
  // constants

  // ----------------------------------
  // internal signals
  // ----------------------------------

  // clock signals
  reg                     tbs_clk;  // tb "sender"    
  reg                     tbr_clk;  // tb "receiver"  
  reg                     uut_clk;  // unit under test

  // reset signals
  reg                     tbs_rst;  // tb "sender"    
  reg                     tbr_rst;  // tb "receiver"  
  reg                     uut_rst;  // unit under test

  // tb "sender" packet components
  wire              [7:0] tbs_hdr[7:0]; 
  reg              [31:0] tbs_key[7:0];
  reg              [31:0] tbs_pld[7:0];
  reg              [31:0] tbs_cnt;  //tbs clock cycle counter

  // tb "receiver" packet components
  reg               [7:0] tbr_hdr[7:0]; 
  reg              [31:0] tbr_key[7:0];
  reg              [31:0] tbr_pld[7:0];
  reg              [31:0] tbr_cnt;  // tbr clock cycle counter 
  reg   [`SEQ_BITS - 1:0] tbr_seq;  // tbr sequence counter
  reg               [4:0] tbr_fwc;  // tbr frame word counter


  // drive packet signals
  wire             [71:0] pkt_dat[7:0];
  reg                     pkt_vld[7:0];
  wire                    pkt_rdy[7:0];

  // drive frame signals
  wire             [31:0] frm_dat;
  wire              [3:0] frm_kch;
  wire                    frm_lst;
  wire                    frm_vld;
  reg                     frm_rdy;

  // drive ack signals
  reg                     ack_type;
  reg   [`CLR_BITS - 1:0] ack_colour;
  reg   [`SEQ_BITS - 1:0] ack_seq;
  reg                     ack_vld;

  // drive channel-flow control signals
  reg  [`NUM_CHANS - 1:0] cfc_rem;

  // drive out-of-credit signals
  wire  [`CLR_BITS - 1:0] ooc_colour;
  wire                    ooc_rts;

  // drive the register bank signals
  wire                    reg_sfrm;
  wire                    reg_rnak;
  wire                    reg_rack;
  wire                    reg_looc;
  wire [`CRDT_BITS - 1:0] reg_crdt;
  wire [`NUM_CHANS - 1:0] reg_empt;
  wire [`NUM_CHANS - 1:0] reg_full;
   

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------- unit under test ------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  spio_hss_multiplexer_frame_assembler uut
  (
    .clk        (uut_clk),
    .rst        (uut_rst),

    // register interface
    .reg_sfrm   (reg_sfrm),
    .reg_rnak   (reg_rnak),
    .reg_rack   (reg_rack),
    .reg_looc   (reg_looc),
    .reg_crdt   (reg_crdt),
    .reg_empt   (reg_empt),
    .reg_full   (reg_full),

    // packet inputs
    .pkt_data0  (pkt_dat[0]),
    .pkt_vld0   (pkt_vld[0]),
    .pkt_rdy0   (pkt_rdy[0]),

    .pkt_data1  (pkt_dat[1]),
    .pkt_vld1   (pkt_vld[1]),
    .pkt_rdy1   (pkt_rdy[1]),

    .pkt_data2  (pkt_dat[2]),
    .pkt_vld2   (pkt_vld[2]),
    .pkt_rdy2   (pkt_rdy[2]),

    .pkt_data3  (pkt_dat[3]),
    .pkt_vld3   (pkt_vld[3]),
    .pkt_rdy3   (pkt_rdy[3]),

    .pkt_data4  (pkt_dat[4]),
    .pkt_vld4   (pkt_vld[4]),
    .pkt_rdy4   (pkt_rdy[4]),

    .pkt_data5  (pkt_dat[5]),
    .pkt_vld5   (pkt_vld[5]),
    .pkt_rdy5   (pkt_rdy[5]),

    .pkt_data6  (pkt_dat[6]),
    .pkt_vld6   (pkt_vld[6]),
    .pkt_rdy6   (pkt_rdy[6]),

    .pkt_data7  (pkt_dat[7]),
    .pkt_vld7   (pkt_vld[7]),
    .pkt_rdy7   (pkt_rdy[7]),

    // ack/nack interface
    .ack_type   (ack_type),
    .ack_colour (ack_colour),
    .ack_seq    (ack_seq),
    .ack_vld    (ack_vld),

    // channel flow control interface
    .cfc_rem    (cfc_rem),

    // frame output interface
    .frm_data   (frm_dat),
    .frm_kchr   (frm_kch),
    .frm_last   (frm_lst),
    .frm_vld    (frm_vld),
    .frm_rdy    (frm_rdy),

    // out-of-credit interface
    .ooc_colour (ooc_colour),
    .ooc_rts    (ooc_rts)
  );
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //----------------------- drive the uut -------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // packet interface (tb sender)
  //---------------------------------------------------------------
  assign tbs_hdr[0] = {7'b0000001, ^(tbs_key[0] ^ tbs_pld[0])};
  assign pkt_dat[0] = {tbs_pld[0], tbs_key[0], tbs_hdr[0]};

  assign tbs_hdr[1] = {7'b0000001, ^(tbs_key[1] ^ tbs_pld[1])};
  assign pkt_dat[1] = {tbs_pld[1], tbs_key[1], tbs_hdr[1]};

  assign tbs_hdr[2] = {7'b0000001, ^(tbs_key[2] ^ tbs_pld[2])};
  assign pkt_dat[2] = {tbs_pld[2], tbs_key[2], tbs_hdr[2]};

  assign tbs_hdr[3] = {7'b0000001, ^(tbs_key[3] ^ tbs_pld[3])};
  assign pkt_dat[3] = {tbs_pld[3], tbs_key[3], tbs_hdr[3]};

  assign tbs_hdr[4] = {7'b0000001, ^(tbs_key[4] ^ tbs_pld[4])};
  assign pkt_dat[4] = {tbs_pld[4], tbs_key[4], tbs_hdr[4]};

  assign tbs_hdr[5] = {7'b0000001, ^(tbs_key[5] ^ tbs_pld[5])};
  assign pkt_dat[5] = {tbs_pld[5], tbs_key[5], tbs_hdr[5]};

  assign tbs_hdr[6] = {7'b0000001, ^(tbs_key[6] ^ tbs_pld[6])};
  assign pkt_dat[6] = {tbs_pld[6], tbs_key[6], tbs_hdr[6]};

  assign tbs_hdr[7] = {7'b0000001, ^(tbs_key[7] ^ tbs_pld[7])};
  assign pkt_dat[7] = {tbs_pld[7], tbs_key[7], tbs_hdr[7]};

  always @ (posedge tbs_clk or posedge tbs_rst)
    if (tbs_rst)
    begin
      tbs_key[0] <= 32'h0000_0001;
      tbs_pld[0] <= 32'ha5a5_a5a5;
      pkt_vld[0] <= 1'b0;

      tbs_key[1] <= 32'h0000_0001;
      tbs_pld[1] <= 32'ha5a5_a5a5;
      pkt_vld[1] <= 1'b0;

      tbs_key[2] <= 32'h0000_0001;
      tbs_pld[2] <= 32'ha5a5_a5a5;
      pkt_vld[2] <= 1'b0;

      tbs_key[3] <= 32'h0000_0001;
      tbs_pld[3] <= 32'ha5a5_a5a5;
      pkt_vld[3] <= 1'b0;

      tbs_key[4] <= 32'h0000_0001;
      tbs_pld[4] <= 32'ha5a5_a5a5;
      pkt_vld[4] <= 1'b0;

      tbs_key[5] <= 32'h0000_0001;
      tbs_pld[5] <= 32'ha5a5_a5a5;
      pkt_vld[5] <= 1'b0;

      tbs_key[6] <= 32'h0000_0001;
      tbs_pld[6] <= 32'ha5a5_a5a5;
      pkt_vld[6] <= 1'b0;

      tbs_key[7] <= 32'h00111_0001;
      tbs_pld[7] <= 32'h5a5a_5a5a;
      pkt_vld[7] <= 1'b0;
    end
    else
    begin
      if (pkt_rdy[0]) tbs_key[0] <= tbs_key[0] + 1;
      pkt_vld[0] <= 1'b1;

      if (pkt_rdy[7]) tbs_key[7] <= tbs_key[7] + 1;
      pkt_vld[7] <= 1'b1;
    end
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // frame interface (tb receiver)
  //---------------------------------------------------------------
  // tbr frame word counter
  always @ (posedge tbr_clk or posedge tbr_rst)
    if (tbr_rst)
      tbr_fwc <= 0;
    else
//#      tbr_fwc <= tbr_fwc + 1;
      tbr_fwc <= 0;

  always @ (posedge tbr_clk or posedge tbr_rst)
    if (tbr_rst)
      frm_rdy <= 1'b0;
    else
      frm_rdy <= 1'b1;

  always @ (posedge tbr_clk)
    if (frm_vld && (frm_kch == `DATA_KBITS))
      tbr_seq <= frm_dat[`FRM_SEQ_RNG];

//#  always @ (posedge tbr_clk)
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // ack/nack interface (tb receiver)
  //---------------------------------------------------------------
  always @ (posedge tbr_clk or posedge tbr_rst)
    if (tbr_rst)
      ack_type <= `ACK_T;
    else
      if ((tbr_cnt > 75) && (tbr_cnt < 300))
        ack_type <= `NAK_T;
      else
        ack_type <= `ACK_T;

  always @ (posedge tbr_clk or posedge tbr_rst)
    if (tbr_rst)
      ack_colour <= 0;
    else
      if ((tbr_cnt > 75) && (tbr_cnt < 300))
        if (ooc_rts)
	  ack_colour <= ~ack_colour;
        else
	  ack_colour <= ack_colour;
      else
	ack_colour <= ack_colour;

  always @ (posedge tbr_clk or posedge tbr_rst)
    if (tbr_rst)
      ack_seq <= 0;
    else
      if (ooc_rts)
        if (ack_type == `NAK_T)
          ack_seq <= tbr_seq;
        else
          ack_seq <= tbr_seq + 1;

  always @ (posedge tbr_clk or posedge tbr_rst)
    if (tbr_rst)
      ack_vld <= 0;
    else
      if (ooc_rts)
        ack_vld <= 1;
//#        ack_vld <= 0;
      else
        ack_vld <= 0;
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // channel flow control interface (tb receiver)
  //---------------------------------------------------------------
  always @ (posedge tbr_clk or posedge tbr_rst)
    if (tbr_rst)
      cfc_rem = 8'b11111111;
    else
      if ((tbr_cnt > 75) && (tbr_cnt < 225))
        cfc_rem = 8'b11111110;
      else
        cfc_rem = 8'b11111111;
  //---------------------------------------------------------------
   

  //---------------------------------------------------------------
  // out-of-credit interface (tb receiver)
  //---------------------------------------------------------------
  //---------------------------------------------------------------

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //----------------- clocks, resets and counters -----------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // generate clocks
  //---------------------------------------------------------------
  initial
  begin
    tbs_clk = 1'b0;
    forever #6.66666667 tbs_clk = ~tbs_clk;
  end

  initial
  begin
    tbr_clk = 1'b0;
    forever #6.66666667 tbr_clk = ~tbr_clk;
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
    tbs_rst  = 1'b1;
    #50
    tbs_rst  = 1'b0;
  end

  initial
  begin
    tbr_rst  = 1'b1;
    #50
    tbr_rst  = 1'b0;
  end

  initial
  begin
    uut_rst  = 1'b1;
    #50
    uut_rst  = 1'b0;
  end
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // clock cycle counters
  //---------------------------------------------------------------
  always @ (posedge tbs_clk or posedge tbs_rst)
    if (tbs_rst)
      tbs_cnt <= 0;
    else
      tbs_cnt <= tbs_cnt + 1;

  always @ (posedge tbr_clk or posedge tbr_rst)
    if (tbr_rst)
      tbr_cnt <= 0;
    else
      tbr_cnt <= tbr_cnt + 1;
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
endmodule
