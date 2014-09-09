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
module spio_hss_multiplexer_frame_tx
(
  input  wire                    clk,
  input  wire                    rst,

  // register interface (to register bank)
  output reg                     reg_tfrm,
  input  wire [`IDLE_BITS - 1:0] reg_idso,

  // frame interface (from frame assembler)
  // assembled data frame
  input  wire  [`FRM_BITS - 1:0] frm_data,
  input  wire  [`KCH_BITS - 1:0] frm_kchr,
  input  wire                    frm_last,
  input  wire                    frm_vld,
  output reg                     frm_rdy,

  // channel flow control interface
  // (from frame assembler & packet dispatcher)
  // report channel flow state to remote side
  input wire [`NUM_CHANS - 1:0] cfc_loc,
  input wire 			cfc_vld,
 
  // out-of-credit interface (from frame assembler)
  // out-of-credit report
  input  wire  [`CLR_BITS - 1:0] ooc_colour,
  input  wire 	                 ooc_vld,

  // ack/nack interface (from packet dispatcher)
  // send ack/nack back to remote side
  input  wire                    ack_type,
  input  wire  [`CLR_BITS - 1:0] ack_colour,
  input  wire  [`SEQ_BITS - 1:0] ack_seq,
  input  wire 	                 ack_vld,

  // high-speed link interface (to gtp)
  output reg   [`FRM_BITS - 1:0] hsl_data,
  output reg   [`KCH_BITS - 1:0] hsl_kchr,
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
  reg  			  ack_vld_l;
   
  reg                     ack_type_i;
  reg   [`CLR_BITS - 1:0] ack_colour_i;
  reg   [`SEQ_BITS - 1:0] ack_seq_i;
  reg  			  ack_vld_i;

  reg   [`FRM_BITS - 1:0] frm_data_l;
  reg   [`KCH_BITS - 1:0] frm_kchr_l;
  reg 	                  frm_last_l;
  reg 	                  frm_vld_l;
  reg	                  frm_sel_l;

  reg   [`FRM_BITS - 1:0] frm_data_i;
  reg   [`KCH_BITS - 1:0] frm_kchr_i;
  reg	                  frm_last_i;
  reg	                  frm_vld_i;

  reg  [STATE_BITS - 1:0] state;
  
  reg 			  send_idle;
  reg 			  send_clkc;
  reg 			  send_frm;
  reg 			  send_nak;
  reg 			  send_ack;
  reg 			  send_ooc;
  reg 			  send_cfc;

  reg  [`CLKC_BITS - 1:0] clkc_ctr;


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
    .crc_last (crc_last),
    .crc_in   (crc_in),
    .crc_out  (crc_out)
  );
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // crc input multiplexor (combinatorial)
  //---------------------------------------------------------------
  always @ (*)
    casex ({send_nak, send_ack, send_ooc, send_cfc})
      4'b1xxx:  crc_in = {`KCH_NAK, ack_colour_i, ack_seq_i, 
                           {(`FRM_BYTES - 2) {8'h00}}
                         };

      4'bx1xx:  crc_in = {`KCH_ACK, ack_colour_i, ack_seq_i,
                           {(`FRM_BYTES - 2) {8'h00}}
                         };

      4'b001x:  crc_in = {`KCH_OOC, ooc_colour, {(8 - `CLR_BITS) {1'b0}}, 
                           {(`FRM_BYTES - 2) {8'h00}}
                         };

      4'b00x1:  crc_in = {`KCH_CFC, cfc_loc, {(8 -`NUM_CHANS) {1'b0}},
                           {(`FRM_BYTES - 2) {8'h00}}
                         };

      default:  crc_in = frm_data_i;
    endcase

  always @ (*)
    if (send_frm)
      crc_last = frm_last_i;
   else
      crc_last = 1'b1;  // initializes crc computation!
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // high-speed link interface
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      hsl_data <= {`FRM_BITS {1'b0}}; // not really necessary!
    else
      casex ({send_clkc, send_idle})
        2'b1x:   hsl_data <= `CLKC_FRM;

        // When idle send a comma and an externally specified sentinel value
        2'bx1:   hsl_data <= {`KCH_COMMA, reg_idso};

        default: hsl_data <= crc_out;
      endcase

  always @ (posedge clk or posedge rst)
    if (rst)
      hsl_kchr <= {`KCH_BITS {1'b0}}; // not really necessary!
    else
      casex ({send_clkc, send_idle, send_nak, send_ack, send_ooc, send_cfc})
        6'b1xxxxx: hsl_kchr <= `CLKC_KBITS;
    
        6'bx1xxxx: hsl_kchr <= `IDLE_KBITS;
    
        6'bxx1xxx: hsl_kchr <= `NAK_KBITS;
    
        6'bxxx1xx: hsl_kchr <= `ACK_KBITS;
    
        6'bxxxx1x: hsl_kchr <= `OOC_KBITS;
    
        6'bxxxxx1: hsl_kchr <= `CFC_KBITS;
    
        default:   hsl_kchr <= frm_kchr_i;
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
      if (frm_vld_i && !send_frm)
        frm_rdy <= 1'b0;
      else
        frm_rdy <= 1'b1;
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // latch incoming data frame flit if not ready to issue
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
    begin
      frm_data_l <= {`FRM_BITS {1'b0}}; // not really necessary!
      frm_kchr_l <= {`KCH_BITS {1'b0}}; // not really necessary!
      frm_last_l <= 1'b0;               // not really necessary!
      frm_vld_l  <= 1'b0;               // not really necessary!
    end
    else
      if (frm_vld_i && !send_frm && !frm_sel_l)
      begin
        frm_data_l <= frm_data;
        frm_kchr_l <= frm_kchr;
        frm_last_l <= frm_last;
        frm_vld_l  <= frm_vld;
      end
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // select latched data frame flit, if appropriate (combinatorial)
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      frm_sel_l  <= 1'b0;
    else
      if (frm_vld_i && !send_frm)
        frm_sel_l  <= 1'b1;
      else
        frm_sel_l  <= 1'b0;

  always @ (*)
    if (frm_sel_l)
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
      ack_vld_l    <= 1'b0;
    end
    else
    begin
      if (send_ack || send_nak)
        ack_vld_l <= 1'b0;
      else
        if (ack_vld)
        begin
          ack_type_l   <= ack_type;
          ack_colour_l <= ack_colour;
          ack_seq_l    <= ack_seq;
          ack_vld_l    <= ack_vld;
        end
    end    
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // select latched ack request, if appropriate (combinatorial)
  //---------------------------------------------------------------
  always @ (*)
    if (ack_vld)
    begin
      ack_type_i   = ack_type;
      ack_colour_i = ack_colour;
      ack_seq_i    = ack_seq;
      ack_vld_i    = ack_vld;
    end
    else
    begin
      ack_type_i   = ack_type_l;
      ack_colour_i = ack_colour_l;
      ack_seq_i    = ack_seq_l;
      ack_vld_i    = ack_vld_l;
    end
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // clock correction counter
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      clkc_ctr <= `CLKC_CNT;
    else
      case (state)
	CHFR_ST: if (!clkc_ctr)
                   clkc_ctr <= `CLKC_CNT;
	         else
                   clkc_ctr <= clkc_ctr - 1;

	default: clkc_ctr <= clkc_ctr;
      endcase
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
  reg vld_ack;
  reg vld_nak;

  always @ (*)
    vld_ack = ack_vld_i && (ack_type_i == `ACK_T);
   
  always @ (*)
    vld_nak = ack_vld_i && (ack_type_i == `NAK_T);
   
  always @ (*)
    casex ({hsl_rdy, (state == DFRM_ST), (clkc_ctr == 0),
              vld_ack, vld_nak, ooc_vld, cfc_vld, frm_vld_i
           }
          )
      8'b0xxxxxxx:  // high-speed link not ready, don't send!
        begin
          send_clkc = 1'b0;
          send_ack  = 1'b0;
          send_nak  = 1'b0;
          send_ooc  = 1'b0;
          send_cfc  = 1'b0;
          send_frm  = 1'b0;
          send_idle = 1'b0;
        end

      // top priority
      8'b101xxxxx:  // time to send clock correction sequence
        begin
          send_clkc = 1'b1;
          send_ack  = 1'b0;
          send_nak  = 1'b0;
          send_ooc  = 1'b0;
          send_cfc  = 1'b0;
          send_frm  = 1'b0;
          send_idle = 1'b0;
        end

      // second priority
      8'b1001xxxx:  // ack frame requested
        begin
          send_clkc = 1'b0;
          send_ack  = 1'b1;
          send_nak  = 1'b0;
          send_ooc  = 1'b0;
          send_cfc  = 1'b0;
          send_frm  = 1'b0;
          send_idle = 1'b0;
        end

      // second priority
      8'b100x1xxx:  // nak frame requested
        begin
          send_clkc = 1'b0;
          send_ack  = 1'b0;
          send_nak  = 1'b1;
          send_ooc  = 1'b0;
          send_cfc  = 1'b0;
          send_frm  = 1'b0;
          send_idle = 1'b0;
        end

      // third priority
      8'b100001xx:  // out-of-credit frame requested
        begin
          send_clkc = 1'b0;
          send_ack  = 1'b0;
          send_nak  = 1'b0;
          send_ooc  = 1'b1;
          send_cfc  = 1'b0;
          send_frm  = 1'b0;
          send_idle = 1'b0;
        end

      // third priority
      8'b10000x1x:  // channel-flow control frame requested
        begin
          send_clkc = 1'b0;
          send_ack  = 1'b0;
          send_nak  = 1'b0;
          send_ooc  = 1'b0;
          send_cfc  = 1'b1;
          send_frm  = 1'b0;
          send_idle = 1'b0;
        end

      // third priority
      8'b11xxxxxx,  // in the middle of a data frame, keep going
      8'b10000xx1:  // new data frame requested
        begin
          send_clkc = 1'b0;
          send_ack  = 1'b0;
          send_nak  = 1'b0;
          send_ooc  = 1'b0;
          send_cfc  = 1'b0;
          send_frm  = 1'b1;
          send_idle = 1'b0;
        end

      // lowest priority
      default:  // if nothing requested, send sync frame
        begin
          send_clkc = 1'b0;
          send_ack  = 1'b0;
          send_nak  = 1'b0;
          send_ooc  = 1'b0;
          send_cfc  = 1'b0;
          send_frm  = 1'b0;
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

	DFRM_ST: if (frm_last)
                   state <= CHFR_ST;
      endcase
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

endmodule
