// -------------------------------------------------------------------------
// $Id: spiNNlink.v 2515 2013-08-19 07:50:12Z plana $
//  spiNNlink module (structral code)
//
// -------------------------------------------------------------------------
// AUTHOR
//  lap - luis.plana@manchester.ac.uk
//  Based on work by J Pepper (Date 08/08/2012)
//
// -------------------------------------------------------------------------
// DETAILS
//  Created on       : 28 Nov 2012
//  Version          : $Revision: 2515 $
//  Last modified on : $Date: 2013-08-19 08:50:12 +0100 (Mon, 19 Aug 2013) $
//  Last modified by : $Author: plana $
//  $HeadURL: https://solem.cs.man.ac.uk/svn/spiNNlink/testing/src/spiNNlink.v $
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
`include "spio_hss_multiplexer_common.h"
// ----------------------------------------------------------------

`timescale 1ns / 1ps
module spio_hss_multiplexer_spinnlink
(
  input  wire 			 clk,
  input  wire 			 rst,

  // register bank interface
  input  wire [`REGA_BITS - 1:0] reg_addr,
  output wire [`REGD_BITS - 1:0] reg_data,

  // packet inputs
  input  wire  [`PKT_BITS - 1:0] pkt_data0,
  input  wire 			 pkt_vld0,
  output wire 			 pkt_rdy0,

  input  wire  [`PKT_BITS - 1:0] pkt_data1,
  input  wire 			 pkt_vld1,
  output wire 			 pkt_rdy1,

  input  wire  [`PKT_BITS - 1:0] pkt_data2,
  input  wire 			 pkt_vld2,
  output wire 			 pkt_rdy2,

  input  wire  [`PKT_BITS - 1:0] pkt_data3,
  input  wire 			 pkt_vld3,
  output wire 			 pkt_rdy3,

  input  wire  [`PKT_BITS - 1:0] pkt_data4,
  input  wire 			 pkt_vld4,
  output wire 			 pkt_rdy4,

  input  wire  [`PKT_BITS - 1:0] pkt_data5,
  input  wire 			 pkt_vld5,
  output wire 			 pkt_rdy5,

  input  wire  [`PKT_BITS - 1:0] pkt_data6,
  input  wire 			 pkt_vld6,
  output wire 			 pkt_rdy6,

  input  wire  [`PKT_BITS - 1:0] pkt_data7,
  input  wire 			 pkt_vld7,
  output wire 			 pkt_rdy7,

  // high-speed link data output
  output wire  [`FRM_BITS - 1:0] hsl_data,
  output wire  [`KCH_BITS - 1:0] hsl_kchr,
  input  wire                    hsl_rdy,

  // high-speed link data input
  input  wire  [`FRM_BITS - 1:0] ihsl_data,
  input  wire  [`KCH_BITS - 1:0] ihsl_kchr,
  input  wire                    ihsl_vld,

  // packet outputs
  output wire  [`PKT_BITS - 1:0] opkt_data0,
  output wire 			 opkt_vld0,
  input  wire 			 opkt_rdy0,

  output wire  [`PKT_BITS - 1:0] opkt_data1,
  output wire 			 opkt_vld1,
  input  wire 			 opkt_rdy1,

  output wire  [`PKT_BITS - 1:0] opkt_data2,
  output wire 			 opkt_vld2,
  input  wire 			 opkt_rdy2,

  output wire  [`PKT_BITS - 1:0] opkt_data3,
  output wire 			 opkt_vld3,
  input  wire 			 opkt_rdy3,

  output wire  [`PKT_BITS - 1:0] opkt_data4,
  output wire 			 opkt_vld4,
  input  wire 			 opkt_rdy4,

  output wire  [`PKT_BITS - 1:0] opkt_data5,
  output wire 			 opkt_vld5,
  input  wire 			 opkt_rdy5,

  output wire  [`PKT_BITS - 1:0] opkt_data6,
  output wire 			 opkt_vld6,
  input  wire 			 opkt_rdy6,

  output wire  [`PKT_BITS - 1:0] opkt_data7,
  output wire 			 opkt_vld7,
  input  wire 			 opkt_rdy7
);

  //---------------------------------------------------------------
  // constants
  //---------------------------------------------------------------
   

  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  // register interface ({fa, ft, fd, pd} -> rb)
  wire [`CRDT_BITS - 1:0] reg_crdt;
  wire                    reg_crce;
  wire                    reg_frme;
  wire                    reg_rfrm;
  wire                    reg_dfrm;
  wire                    reg_tfrm;
  wire                    reg_sfrm;
  wire                    reg_busy;
  wire                    reg_nack;
  wire                    reg_lnak;
  wire                    reg_rnak;
  wire                    reg_lack;
  wire                    reg_rack;
  wire                    reg_looc;
  wire                    reg_rooc;
  wire [`NUM_CHANS - 1:0] reg_empt;
  wire [`NUM_CHANS - 1:0] reg_full;
   

  // frame interface (fa -> ft)
  wire  [`FRM_BITS - 1:0] frm_data;
  wire  [`KCH_BITS - 1:0] frm_kchr;
  wire                    frm_last;
  wire                    frm_vld;
  wire                    frm_rdy;

  // channel flow control interface (fa -> ft)
  wire                    cfc_vld;

  // out-of-credit interface (fa -> ft)
  wire                    ooc_colour;
  wire                    ooc_vld;

  // remote ack/nack interface (fd -> fa)
  wire                    rack_type;
  wire  [`CLR_BITS - 1:0] rack_colour;
  wire  [`SEQ_BITS - 1:0] rack_seq;
  wire                    rack_vld;

  // local ack/nack interface (pd -> ft)
  wire                    lack_type;
  wire  [`CLR_BITS - 1:0] lack_colour;
  wire  [`SEQ_BITS - 1:0] lack_seq;
  wire                    lack_vld;

  // remote channel flow control (fd -> fa)
  wire [`NUM_CHANS - 1:0] cfc_rem;
   
  // local channel flow control (pd -> fa)
  wire [`NUM_CHANS - 1:0] cfc_loc;
   
  // received packet outputs (fd -> pd) 
  wire  [`PKT_BITS - 1:0] ipkt_data0;
  wire                    ipkt_vld0;

  wire  [`PKT_BITS - 1:0] ipkt_data1;
  wire                    ipkt_vld1;

  wire  [`PKT_BITS - 1:0] ipkt_data2;
  wire                    ipkt_vld2;

  wire  [`PKT_BITS - 1:0] ipkt_data3;
  wire                    ipkt_vld3;

  wire  [`PKT_BITS - 1:0] ipkt_data4;
  wire                    ipkt_vld4;

  wire  [`PKT_BITS - 1:0] ipkt_data5;
  wire                    ipkt_vld5;

  wire  [`PKT_BITS - 1:0] ipkt_data6;
  wire                    ipkt_vld6;

  wire  [`PKT_BITS - 1:0] ipkt_data7;
  wire                    ipkt_vld7;

  // frame interface (fd -> pd)
  wire  [`CLR_BITS - 1:0] ifrm_colour;
  wire  [`SEQ_BITS - 1:0] ifrm_seq;
  wire                    ifrm_vld;

  // out-of-credit interface (fd -> pd)
  wire                    iooc_colour;
  wire                    iooc_vld;


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //--------------------------- structure -------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // instantiate the frame assembler
  //---------------------------------------------------------------
  spio_hss_multiplexer_frame_assembler fa
  (
    .clk        (clk),
    .rst        (rst),

    // register interface (to register bank)
    .reg_sfrm   (reg_sfrm),
    .reg_rnak   (reg_rnak),
    .reg_rack   (reg_rack),
    .reg_looc   (reg_looc),
    .reg_crdt   (reg_crdt),
    .reg_empt   (reg_empt),
    .reg_full   (reg_full),

    // packet interface
    .pkt_data0  (pkt_data0),
    .pkt_vld0   (pkt_vld0),
    .pkt_rdy0   (pkt_rdy0),

    .pkt_data1  (pkt_data1),
    .pkt_vld1   (pkt_vld1),
    .pkt_rdy1   (pkt_rdy1),

    .pkt_data2  (pkt_data2),
    .pkt_vld2   (pkt_vld2),
    .pkt_rdy2   (pkt_rdy2),

    .pkt_data3  (pkt_data3),
    .pkt_vld3   (pkt_vld3),
    .pkt_rdy3   (pkt_rdy3),

    .pkt_data4  (pkt_data4),
    .pkt_vld4   (pkt_vld4),
    .pkt_rdy4   (pkt_rdy4),

    .pkt_data5  (pkt_data5),
    .pkt_vld5   (pkt_vld5),
    .pkt_rdy5   (pkt_rdy5),

    .pkt_data6  (pkt_data6),
    .pkt_vld6   (pkt_vld6),
    .pkt_rdy6   (pkt_rdy6),

    .pkt_data7  (pkt_data7),
    .pkt_vld7   (pkt_vld7),
    .pkt_rdy7   (pkt_rdy7),

    // ack/nack interface
    .ack_type   (rack_type),
    .ack_colour (rack_colour),
    .ack_seq    (rack_seq),
    .ack_vld    (rack_vld),

    // channel flow control interface
    .cfc_rem    (cfc_rem),
    .cfc_loc    (cfc_loc),
    .cfc_vld    (cfc_vld),
 
    // frame interface
    .frm_data   (frm_data),
    .frm_kchr   (frm_kchr),
    .frm_last   (frm_last),
    .frm_vld    (frm_vld),
    .frm_rdy    (frm_rdy),

   // out-of-credit interface
    .ooc_colour (ooc_colour),
    .ooc_vld    (ooc_vld)
  );
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // instantiate the frame transmitter
  //---------------------------------------------------------------
  spio_hss_multiplexer_frame_tx ft
  (
    .clk (clk),
    .rst (rst),

    // register interface (to register bank)
    .reg_tfrm   (reg_tfrm),

    // frame interface
    .frm_data   (frm_data),
    .frm_kchr   (frm_kchr),
    .frm_last   (frm_last),
    .frm_vld    (frm_vld),
    .frm_rdy    (frm_rdy),

    // channel flow control interface
    .cfc_loc    (cfc_loc),
    .cfc_vld    (cfc_vld),
 
    // out-of-credit interface
    .ooc_colour (ooc_colour),
    .ooc_vld    (ooc_vld),

    // ack/nack interface
    .ack_type   (lack_type),
    .ack_colour (lack_colour),
    .ack_seq    (lack_seq),
    .ack_vld    (lack_vld),

    // high-speed link interface
    .hsl_data   (hsl_data),
    .hsl_kchr   (hsl_kchr),
    .hsl_rdy    (hsl_rdy)
  );
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // instantiate the frame dissasembler
  //---------------------------------------------------------------
  spio_hss_multiplexer_frame_disassembler fd
  (
    .clk (clk),
    .rst (rst),

    // register interface (to register bank)
    .reg_dfrm   (reg_dfrm),
    .reg_crce   (reg_crce),
    .reg_frme   (reg_frme),
    .reg_rooc   (reg_rooc),

    // high-speed link interface
    .hsl_data   (ihsl_data),
    .hsl_kchr   (ihsl_kchr),
    .hsl_vld    (ihsl_vld),

    // ack/nack interface
    .ack_type   (rack_type),
    .ack_colour (rack_colour),
    .ack_seq    (rack_seq),
    .ack_vld    (rack_vld),

    // channel flow control interface
    .cfc_rem    (cfc_rem),

    // packet interface (to packet dispatcher)
    .pkt_data0  (ipkt_data0),
    .pkt_vld0   (ipkt_vld0),

    .pkt_data1  (ipkt_data1),
    .pkt_vld1   (ipkt_vld1),

    .pkt_data2  (ipkt_data2),
    .pkt_vld2   (ipkt_vld2),

    .pkt_data3  (ipkt_data3),
    .pkt_vld3   (ipkt_vld3),

    .pkt_data4  (ipkt_data4),
    .pkt_vld4   (ipkt_vld4),

    .pkt_data5  (ipkt_data5),
    .pkt_vld5   (ipkt_vld5),

    .pkt_data6  (ipkt_data6),
    .pkt_vld6   (ipkt_vld6),

    .pkt_data7  (ipkt_data7),
    .pkt_vld7   (ipkt_vld7),

    // frame interface (to packet dispatcher)
    .frm_colour (ifrm_colour),
    .frm_seq    (ifrm_seq),
    .frm_vld    (ifrm_vld),

    // out-of-credit interface (to packet dispatcher)
    .ooc_colour (iooc_colour),
    .ooc_vld    (iooc_vld)
  );
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // instantiate the packet dispatcher
  //---------------------------------------------------------------
  spio_hss_multiplexer_packet_dispatcher pd
  (
    .clk (clk),
    .rst (rst),

    // register interface (to register bank)
    .reg_rfrm   (reg_rfrm),
    .reg_busy   (reg_busy),
    .reg_lnak   (reg_lnak),
    .reg_lack   (reg_lack),

    // packet interface (from frame disassembler)
    .ipkt_data0 (ipkt_data0),
    .ipkt_vld0  (ipkt_vld0),

    .ipkt_data1 (ipkt_data1),
    .ipkt_vld1  (ipkt_vld1),

    .ipkt_data2 (ipkt_data2),
    .ipkt_vld2  (ipkt_vld2),

    .ipkt_data3 (ipkt_data3),
    .ipkt_vld3  (ipkt_vld3),

    .ipkt_data4 (ipkt_data4),
    .ipkt_vld4  (ipkt_vld4),

    .ipkt_data5 (ipkt_data5),
    .ipkt_vld5  (ipkt_vld5),

    .ipkt_data6 (ipkt_data6),
    .ipkt_vld6  (ipkt_vld6),

    .ipkt_data7 (ipkt_data7),
    .ipkt_vld7  (ipkt_vld7),

    // frame interface (from frame disassembler)
    .frm_colour (ifrm_colour),
    .frm_seq    (ifrm_seq),
    .frm_vld    (ifrm_vld),

    // out-of-credit interface (from frame disassembler)
    .ooc_colour (iooc_colour),
    .ooc_vld    (iooc_vld),

    // local channel flow control interface
    .cfc_loc (cfc_loc),

    // ack/nack interface
    .ack_type   (lack_type),
    .ack_colour (lack_colour),
    .ack_seq    (lack_seq),
    .ack_vld    (lack_vld),

    // packet interface (to packet dispatcher)
    .pkt_data0 (opkt_data0),
    .pkt_vld0  (opkt_vld0),
    .pkt_rdy0  (opkt_rdy0),

    .pkt_data1 (opkt_data1),
    .pkt_vld1  (opkt_vld1),
    .pkt_rdy1  (opkt_rdy1),

    .pkt_data2 (opkt_data2),
    .pkt_vld2  (opkt_vld2),
    .pkt_rdy2  (opkt_rdy2),

    .pkt_data3 (opkt_data3),
    .pkt_vld3  (opkt_vld3),
    .pkt_rdy3  (opkt_rdy3),

    .pkt_data4 (opkt_data4),
    .pkt_vld4  (opkt_vld4),
    .pkt_rdy4  (opkt_rdy4),

    .pkt_data5 (opkt_data5),
    .pkt_vld5  (opkt_vld5),
    .pkt_rdy5  (opkt_rdy5),

    .pkt_data6 (opkt_data6),
    .pkt_vld6  (opkt_vld6),
    .pkt_rdy6  (opkt_rdy6),

    .pkt_data7 (opkt_data7),
    .pkt_vld7  (opkt_vld7),
    .pkt_rdy7  (opkt_rdy7)
  );
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // instantiate the register bank
  //---------------------------------------------------------------
  spio_hss_multiplexer_reg_bank rb
  (
    .clk      (clk),
    .rst      (rst),

    // frame assembler interface
    .reg_sfrm (reg_sfrm),
    .reg_rnak (reg_rnak),
    .reg_rack (reg_rack),
    .reg_looc (reg_looc),
    .reg_crdt (reg_crdt),
    .reg_empt (reg_empt),
    .reg_full (reg_full),


    // frame transmitter interface
    .reg_tfrm (reg_tfrm),

    // frame disassembler interface
    .reg_dfrm (reg_dfrm),
    .reg_crce (reg_crce),
    .reg_frme (reg_frme),
    .reg_rooc (reg_rooc),
    .reg_cfcr (cfc_rem),

    // packet dispatcher interface
    .reg_rfrm (reg_rfrm),
    .reg_busy (reg_busy),
    .reg_lnak (reg_lnak),
    .reg_lack (reg_lack),
    .reg_cfcl (cfc_loc),

    // register access interface
    .reg_addr (reg_addr),
    .reg_data (reg_data)
  );
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

endmodule
