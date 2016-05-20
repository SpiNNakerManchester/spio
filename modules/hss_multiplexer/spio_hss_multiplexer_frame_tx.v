// -------------------------------------------------------------------------
// $Id: frame_tx.v 2515 2013-08-19 07:50:12Z plana $
//  spiNNlink frame transmit module
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
//  $HeadURL: https://solem.cs.man.ac.uk/svn/spiNNlink/testing/src/frame_tx.v $
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


// ----------------------------------------------------------------
// include spiNNlink global constants and parameters
//
`include "spio_hss_multiplexer_common.h"
// ----------------------------------------------------------------

//-----------------------------------------------------------------
// useful macros (local to frame_tx)
//-----------------------------------------------------------------
`define NAK_FRM    {`KCH_NAK, ack_colour_i, ack_seq_i, `CRC_PAD}
`define ACK_FRM    {`KCH_ACK, ack_colour_i, ack_seq_i, `CRC_PAD}
`define OOC_FRM    {`KCH_OOC, ooc_colour, {(8 - `CLR_BITS) {1'b0}}, `CRC_PAD}
`define CFC_FRM    {`KCH_CFC, cfc_loc, `CRC_PAD}
`define LAST_FRM   {ack_colour_i, ack_seq_i, cfc_loc, `CRC_PAD}


`timescale 1ns / 1ps
module spio_hss_multiplexer_frame_tx
(
  input  wire                    clk,
  input  wire                    rst,

  // register interface (to register bank)
  output reg                     reg_tfrm,
  input wire                     reg_stop,

  // frame interface (from frame assembler)
  // assembled data frame
  input  wire  [`FRM_BITS - 1:0] frm_data,
  input  wire  [`KCH_BITS - 1:0] frm_kchr,
  input  wire                    frm_last,
  input  wire                    frm_vld,
  output reg                     frm_rdy,

  // channel flow control interface
  // from packet dispatcher
  // report channel flow state to remote side
  input wire  [`NUM_CHANS - 1:0] cfc_loc,
 
  // out-of-credit interface (from frame assembler)
  // out-of-credit report
  input  wire  [`CLR_BITS - 1:0] ooc_colour,
  input  wire 	                 ooc_rts,

  // ack/nack interface (from packet dispatcher)
  // send ack/nack back to remote side
  input  wire                    ack_type,
  input  wire  [`CLR_BITS - 1:0] ack_colour,
  input  wire  [`SEQ_BITS - 1:0] ack_seq,
  input  wire 	                 ack_rts,

  // high-speed link interface (to gtp)
  output reg   [`FRM_BITS - 1:0] hsl_data,
  output reg   [`KCH_BITS - 1:0] hsl_kchr,
  output reg 	                 hsl_vld,
  input  wire 	                 hsl_rdy
);

  //---------------------------------------------------------------
  // constants
  //---------------------------------------------------------------
  localparam STATE_BITS = 1;
  localparam CHFR_ST = 0;
  localparam DFRM_ST = CHFR_ST + 1;
   

  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  reg                     ack_type_l;
  reg   [`CLR_BITS - 1:0] ack_colour_l;
  reg   [`SEQ_BITS - 1:0] ack_seq_l;
  reg  			  ack_rts_l;
   
  reg                     ack_type_i;
  reg   [`CLR_BITS - 1:0] ack_colour_i;
  reg   [`SEQ_BITS - 1:0] ack_seq_i;
  reg  			  ack_rts_i;

  reg   [`FRM_BITS - 1:0] frm_data_l;
  reg   [`KCH_BITS - 1:0] frm_kchr_l;
  reg 	                  frm_last_l;
  reg 	                  frm_vld_l;

  reg   [`FRM_BITS - 1:0] frm_data_i;
  reg   [`KCH_BITS - 1:0] frm_kchr_i;
  reg	                  frm_last_i;
  reg	                  frm_vld_i;

  reg  [STATE_BITS - 1:0] state;
  
  reg  park_frm;

  reg  send_last;
  reg  send_nak;
  reg  send_ack;
  reg  send_ooc;
  reg  send_frm;
  reg  send_cfc;
  reg  send_idle;  // really means do not send anything!

  reg  rts_nak;
  reg  rts_ack;
  reg  rts_cfc;
  reg  rts_ooc;
  reg  rts_frm;



  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //--------------------------- datapath --------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // compute crc on the fly (combinatorial)
  //---------------------------------------------------------------
  reg    [`FRM_BITS - 1:0] crc_in;
  wire   [`FRM_BITS - 1:0] crc_out;
  reg                      crc_last;
   
  spio_hss_multiplexer_crc_gen crc_gen_0
  (
    .clk      (clk),
    .rst      (rst),
    .crc_go   (hsl_rdy),
    .crc_last (crc_last),
    .crc_in   (crc_in),
    .crc_out  (crc_out)
  );
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // crc input multiplexer (combinatorial)
  //---------------------------------------------------------------
  always @ (*)
    casex ({send_nak, send_ack, send_ooc, send_cfc, send_last})
      5'b1xxxx: crc_in = `NAK_FRM;
      5'bx1xxx: crc_in = `ACK_FRM;
      5'bxx1xx: crc_in = `OOC_FRM;
      5'bxxx1x: crc_in = `CFC_FRM;
      5'bxxxx1: crc_in = `LAST_FRM;
      default:  crc_in = frm_data_i;
    endcase

  always @ (*)
    if (send_frm)
      crc_last = 1'b0;
   else
      crc_last = 1'b1;  // initializes crc computation!
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // high-speed link interface
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      hsl_vld <= 1'b0;
    else
      if (hsl_rdy)
	if (!send_idle)
          hsl_vld <= 1'b1;
        else
          hsl_vld <= 1'b0;

  always @ (posedge clk)
    if (hsl_rdy && !send_idle)
        hsl_data <= crc_out;

  always @ (posedge clk)
    if (hsl_rdy && !send_idle)
      casex ({send_nak, send_ack, send_ooc, send_cfc, send_last})
        5'b1xxxx: hsl_kchr <= `NAK_KBITS;
        5'bx1xxx: hsl_kchr <= `ACK_KBITS;
        5'bxx1xx: hsl_kchr <= `OOC_KBITS;
        5'bxxx1x: hsl_kchr <= `CFC_KBITS;
        5'bxxxx1: hsl_kchr <= `ZERO_KBITS;
        default:  hsl_kchr <= frm_kchr_i;
      endcase
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // frame interface
  //---------------------------------------------------------------
  //---------------------------------------------------------------
  // frame ready signal
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      frm_rdy <= 1'b1;
    else
      if (frm_vld_i && !send_frm && !send_last)
        frm_rdy <= 1'b0;
      else
        frm_rdy <= 1'b1;
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // latch incoming data frame word if not ready to issue
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
    begin
      frm_data_l <= {`FRM_BITS {1'b0}}; // not really necessary!
      frm_kchr_l <= {`KCH_BITS {1'b0}}; // not really necessary!
      frm_last_l <= 1'b0;
      frm_vld_l  <= 1'b0;
    end
    else
      if (frm_vld_i && !send_frm && !send_last && !park_frm)
      begin
        frm_data_l <= frm_data;
        frm_kchr_l <= frm_kchr;
        frm_last_l <= frm_last;
        frm_vld_l  <= frm_vld;
      end
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // select latched data frame word, if appropriate (combinatorial)
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      park_frm <= 1'b0;
    else
      if (frm_vld_i && !send_frm && !send_last)
        park_frm <= 1'b1;
      else
        park_frm <= 1'b0;

  always @ (*)
    if (park_frm)
    begin
      frm_data_i = frm_data_l;
      frm_kchr_i = frm_kchr_l;
      frm_last_i = frm_last_l;
      frm_vld_i  = frm_vld_l;
    end
    else
    begin
      frm_data_i = frm_data;
      frm_kchr_i = frm_kchr;
      frm_last_i = frm_last;
      frm_vld_i  = frm_vld;
    end
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // ack/nak interface
  //---------------------------------------------------------------
  //---------------------------------------------------------------
  // latch incoming ack request if not ready to issue
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
    begin
      ack_type_l   <= `NAK_T;             // not really necessary!
      ack_colour_l <= {`CLR_BITS {1'b0}}; // not really necessary!
      ack_seq_l    <= `SEQ_BITS'd0;       // not really necessary!
      ack_rts_l    <= 1'b0;
    end
    else
      if (send_ack || send_nak)
        ack_rts_l <= 1'b0;
      else
        if (ack_rts)
        begin
          ack_type_l   <= ack_type;
          ack_colour_l <= ack_colour;
          ack_seq_l    <= ack_seq;
          ack_rts_l    <= ack_rts;
        end
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // select latched ack request, if appropriate (combinatorial)
  //---------------------------------------------------------------
  always @ (*)
    if (ack_rts)
    begin
      ack_type_i   = ack_type;
      ack_colour_i = ack_colour;
      ack_seq_i    = ack_seq;
      ack_rts_i    = ack_rts;
    end
    else
    begin
      ack_type_i   = ack_type_l;
      ack_colour_i = ack_colour_l;
      ack_seq_i    = ack_seq_l;
      ack_rts_i    = ack_rts_l;
    end
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // channel-flow control
  //---------------------------------------------------------------
  reg  [`NUM_CHANS - 1:0] prev_cfc;
   
  always @ (posedge clk or posedge rst)
    if (rst)
      prev_cfc <= {`NUM_CHANS {1'b1}};
    else
      if (send_cfc || send_last)
        prev_cfc <= cfc_loc;
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // register interface
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      reg_tfrm <= 1'b0;
    else
      reg_tfrm <= (state == CHFR_ST) && send_frm;
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	 
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------- control --------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // frame type selection control (combinatorial)
  //---------------------------------------------------------------
  always @ (*)
    rts_nak = ack_rts_i && (ack_type_i == `NAK_T);
   
  always @ (*)
    rts_ack = ack_rts_i && (ack_type_i == `ACK_T);
   
  always @ (*)
    rts_cfc = (cfc_loc != prev_cfc);
  
  always @ (*)
    rts_ooc = ooc_rts;
   
  always @ (*)
    rts_frm = frm_vld_i && !reg_stop;
  
  always @ (*)
    if (!hsl_rdy)  // output not ready, don't send
    begin
      send_nak  = 1'b0;
      send_ack  = 1'b0;
      send_ooc  = 1'b0;
      send_frm  = 1'b0;
      send_last = 1'b0;
      send_cfc  = 1'b0;
      send_idle = 1'b0;
    end
    // in the middle of a data frame, keep going
    else if (state == DFRM_ST)
      if (frm_last_i)  // send last word in frame (special)
      begin
        send_nak  = 1'b0;
        send_ack  = 1'b0;
        send_ooc  = 1'b0;
        send_frm  = 1'b0;
        send_last = 1'b1;
        send_cfc  = 1'b0;
        send_idle = 1'b0;
      end
      else             // send next word in frame
      begin
        send_nak  = 1'b0;
        send_ack  = 1'b0;
        send_ooc  = 1'b0;
        send_frm  = 1'b1;
        send_last = 1'b0;
        send_cfc  = 1'b0;
        send_idle = 1'b0;
      end
    else  // idle: choose frame to send
      casex ({rts_nak, rts_cfc, rts_ack, rts_ooc, rts_frm})
        // top priority
        5'b1xxxx:  // nak frame requested (m.e. with ack)
          begin
            send_nak  = 1'b1;
            send_ack  = 1'b0;
            send_ooc  = 1'b0;
            send_frm  = 1'b0;
            send_last = 1'b0;
            send_cfc  = 1'b0;
            send_idle = 1'b0;
          end
      
        // second priority
        5'b01xxx:  // cfc frame requested
          begin
            send_nak  = 1'b0;
            send_ack  = 1'b0;
            send_ooc  = 1'b0;
            send_frm  = 1'b0;
            send_last = 1'b0;
            send_cfc  = 1'b1;
            send_idle = 1'b0;
          end
      
        // third priority
        5'bx01xx:  // ack frame requested (m.e. with nack)
          begin
            send_nak  = 1'b0;
            send_ack  = 1'b1;
            send_ooc  = 1'b0;
            send_frm  = 1'b0;
            send_last = 1'b0;
            send_cfc  = 1'b0;
            send_idle = 1'b0;
          end
      
        // joint fourth priority
        5'b0001x:  // out-of-credit frame requested (m.e. with frm)
          begin
            send_nak  = 1'b0;
            send_ack  = 1'b0;
            send_ooc  = 1'b1;
            send_frm  = 1'b0;
            send_last = 1'b0;
            send_cfc  = 1'b0;
            send_idle = 1'b0;
          end
      
        // joint fourth priority
        5'b000x1:  // new data frame requested (m.e. with ooc)
          begin
            send_nak  = 1'b0;
            send_ack  = 1'b0;
            send_ooc  = 1'b0;
            send_frm  = 1'b1;
            send_last = 1'b0;
            send_cfc  = 1'b0;
            send_idle = 1'b0;
          end
      
        // if nothing requested go/stay idle
        default:
          begin
            send_nak  = 1'b0;
            send_ack  = 1'b0;
            send_ooc  = 1'b0;
            send_frm  = 1'b0;
            send_last = 1'b0;
            send_cfc  = 1'b0;
            send_idle = 1'b1;
          end
      endcase
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // next state
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      state <= CHFR_ST;
    else
      case (state)
	CHFR_ST: if (send_frm)
                   state <= DFRM_ST;

	DFRM_ST: if (send_last)
                   state <= CHFR_ST;
      endcase
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

endmodule
