// -------------------------------------------------------------------------
// $Id: packet_dispatch.v 2515 2013-08-19 07:50:12Z plana $
//  spiNNlink packet dispatch module
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
//  $HeadURL: https://solem.cs.man.ac.uk/svn/spiNNlink/testing/src/packet_dispatch.v $
//
// -------------------------------------------------------------------------
// COPYRIGHT
//  Copyright (c) The University of Manchester, 2012. All rights reserved.
//  SpiNNaker Project
//  Advanced Processor Technologies Group
//  School of Computer Science
// -------------------------------------------------------------------------
// TODO
//  * fix individual channel flow control
// -------------------------------------------------------------------------


// ----------------------------------------------------------------
// include spiNNlink global constants and parameters
//
`include "spio_hss_multiplexer_common.h"
// ----------------------------------------------------------------

`timescale 1ns / 1ps
module spio_hss_multiplexer_packet_dispatcher
(
  input  wire 			 clk,
  input  wire 			 rst,

  // register interface (to register bank)
  output reg                     reg_rfrm,
  output reg                     reg_busy,
  output reg                     reg_lnak,
  output reg                     reg_lack,

  // incoming packet interface (from frame disassembler)
  input  wire  [`PKT_BITS - 1:0] ipkt_data0,
  input  wire 			 ipkt_vld0,

  input  wire  [`PKT_BITS - 1:0] ipkt_data1,
  input  wire 			 ipkt_vld1,

  input  wire  [`PKT_BITS - 1:0] ipkt_data2,
  input  wire 			 ipkt_vld2,

  input  wire  [`PKT_BITS - 1:0] ipkt_data3,
  input  wire 			 ipkt_vld3,

  input  wire  [`PKT_BITS - 1:0] ipkt_data4,
  input  wire 			 ipkt_vld4,

  input  wire  [`PKT_BITS - 1:0] ipkt_data5,
  input  wire 			 ipkt_vld5,

  input  wire  [`PKT_BITS - 1:0] ipkt_data6,
  input  wire 			 ipkt_vld6,

  input  wire  [`PKT_BITS - 1:0] ipkt_data7,
  input  wire 			 ipkt_vld7,

  // frame interface (from frame disassembler)
  input  wire  [`CLR_BITS - 1:0] frm_colour,
  input  wire  [`SEQ_BITS - 1:0] frm_seq,
  input  wire 			 frm_vld,

  // out-of-credit interface (from frame disassembler)
  input  wire  [`CLR_BITS - 1:0] ooc_colour,
  input  wire 			 ooc_vld,

  // channel flow control interface (to frame assembler)
  // send local cfc to remote side
  output reg  [`NUM_CHANS - 1:0] cfc_loc,
 
  // ack/nack interface (to frame transmitter)
  // send ack/nack back to remote side
  output reg                     ack_type,
  output reg   [`CLR_BITS - 1:0] ack_colour,
  output reg   [`SEQ_BITS - 1:0] ack_seq,
  output reg  			 ack_rts,

  // output packet interface
  output wire  [`PKT_BITS - 1:0] pkt_data0,
  output wire 			 pkt_vld0,
  input  wire 			 pkt_rdy0,

  output wire  [`PKT_BITS - 1:0] pkt_data1,
  output wire 			 pkt_vld1,
  input  wire 			 pkt_rdy1,

  output wire  [`PKT_BITS - 1:0] pkt_data2,
  output wire 			 pkt_vld2,
  input  wire 			 pkt_rdy2,

  output wire  [`PKT_BITS - 1:0] pkt_data3,
  output wire 			 pkt_vld3,
  input  wire 			 pkt_rdy3,

  output wire  [`PKT_BITS - 1:0] pkt_data4,
  output wire 			 pkt_vld4,
  input  wire 			 pkt_rdy4,

  output wire  [`PKT_BITS - 1:0] pkt_data5,
  output wire 			 pkt_vld5,
  input  wire 			 pkt_rdy5,

  output wire  [`PKT_BITS - 1:0] pkt_data6,
  output wire 			 pkt_vld6,
  input  wire 			 pkt_rdy6,

  output wire  [`PKT_BITS - 1:0] pkt_data7,
  output wire 			 pkt_vld7,
  input  wire 			 pkt_rdy7
);

  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  reg   [`CLR_BITS - 1:0] colour; 
  reg   [`SEQ_BITS - 1:0] seq_exp; 

  reg  [`A_CNT_BITS -1:0] ack_snd_ctr;
  reg  [`N_CNT_BITS -1:0] nak_rsn_ctr;
   
  reg                     ok_colour;
  reg                     ok_frm;

  reg                     go_frm;
  reg                     rjct_frm;

  wire [`NUM_CHANS - 1:0] busy;  // packet store cannot accept packet
  wire [`NUM_CHANS - 1:0] cfcf;  // individual channel flow control flag


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //--------------------------- structure -------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // output packet fifos
  //---------------------------------------------------------------
  spio_hss_multiplexer_pkt_fifo pst0
  (
    .clk       (clk),
    .rst       (rst),

    .go_frm    (go_frm),
    .busy      (busy[0]),
    .cfcf      (cfcf[0]),

    .ipkt_data (ipkt_data0),
    .ipkt_vld  (ipkt_vld0),

    .pkt_data  (pkt_data0),
    .pkt_vld   (pkt_vld0),
    .pkt_rdy   (pkt_rdy0)
  );

  spio_hss_multiplexer_pkt_fifo pst1
  (
    .clk       (clk),
    .rst       (rst),

    .go_frm    (go_frm),
    .busy      (busy[1]),
    .cfcf      (cfcf[1]),

    .ipkt_data (ipkt_data1),
    .ipkt_vld  (ipkt_vld1),

    .pkt_data  (pkt_data1),
    .pkt_vld   (pkt_vld1),
    .pkt_rdy   (pkt_rdy1)
  );

  spio_hss_multiplexer_pkt_fifo pst2
  (
    .clk       (clk),
    .rst       (rst),

    .go_frm    (go_frm),
    .busy      (busy[2]),
    .cfcf      (cfcf[2]),

    .ipkt_data (ipkt_data2),
    .ipkt_vld  (ipkt_vld2),

    .pkt_data  (pkt_data2),
    .pkt_vld   (pkt_vld2),
    .pkt_rdy   (pkt_rdy2)
  );

  spio_hss_multiplexer_pkt_fifo pst3
  (
    .clk       (clk),
    .rst       (rst),

    .go_frm    (go_frm),
    .busy      (busy[3]),
    .cfcf      (cfcf[3]),

    .ipkt_data (ipkt_data3),
    .ipkt_vld  (ipkt_vld3),

    .pkt_data  (pkt_data3),
    .pkt_vld   (pkt_vld3),
    .pkt_rdy   (pkt_rdy3)
  );

  spio_hss_multiplexer_pkt_fifo pst4
  (
    .clk       (clk),
    .rst       (rst),

    .go_frm    (go_frm),
    .busy      (busy[4]),
    .cfcf      (cfcf[4]),

    .ipkt_data (ipkt_data4),
    .ipkt_vld  (ipkt_vld4),

    .pkt_data  (pkt_data4),
    .pkt_vld   (pkt_vld4),
    .pkt_rdy   (pkt_rdy4)
  );

  spio_hss_multiplexer_pkt_fifo pst5
  (
    .clk       (clk),
    .rst       (rst),

    .go_frm    (go_frm),
    .busy      (busy[5]),
    .cfcf      (cfcf[5]),

    .ipkt_data (ipkt_data5),
    .ipkt_vld  (ipkt_vld5),

    .pkt_data  (pkt_data5),
    .pkt_vld   (pkt_vld5),
    .pkt_rdy   (pkt_rdy5)
  );

  spio_hss_multiplexer_pkt_fifo pst6
  (
    .clk       (clk),
    .rst       (rst),

    .go_frm    (go_frm),
    .busy      (busy[6]),
    .cfcf      (cfcf[6]),

    .ipkt_data (ipkt_data6),
    .ipkt_vld  (ipkt_vld6),

    .pkt_data  (pkt_data6),
    .pkt_vld   (pkt_vld6),
    .pkt_rdy   (pkt_rdy6)
  );

  spio_hss_multiplexer_pkt_fifo pst7
  (
    .clk       (clk),
    .rst       (rst),

    .go_frm    (go_frm),
    .busy      (busy[7]),
    .cfcf      (cfcf[7]),

    .ipkt_data (ipkt_data7),
    .ipkt_vld  (ipkt_vld7),

    .pkt_data  (pkt_data7),
    .pkt_vld   (pkt_vld7),
    .pkt_rdy   (pkt_rdy7)
  );
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //--------------------------- datapath --------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // channel flow control interface
  //---------------------------------------------------------------
  always @ (*)
    cfc_loc = cfcf;
//#    cfc_loc = {`NUM_CHANS {1'b1}};
//#  always @ (posedge clk or posedge rst)
//#    if (rst)
//#      cfc_loc <= {`NUM_CHANS {1'b1}};
//#    else
//#      if (ok_frm)
//#        cfc_loc <= ~busy;
//#      else
//#        cfc_loc <= cfc_loc;  // no change!
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // ack/nak interface
  //---------------------------------------------------------------
  // to simplify ack/nak treatment, ack_seq is used in a similar
  // in both: frame ack_seq is not ack'ed, all previous ones are.
  // Additionally, a nak also requests that frame ack_seq is re-sent.
  always @ (posedge clk or posedge rst)
    if (rst)
    begin
      ack_type   <= `NAK_T;        // not really necessary!
      ack_colour <= `CLR_BITS'd0;  // not really necessary!
      ack_seq    <= `SEQ_BITS'd0;  // not really necessary!
      ack_rts    <= 1'b0;
    end
    else
      casex ({ooc_vld, (ooc_colour == colour),
              frm_vld, (frm_colour == colour), (frm_seq == seq_exp),
              rjct_frm, (ack_snd_ctr == 0), (nak_rsn_ctr == 0)
             }
            )
        8'b11xxxxxx: begin  // ack expected seq number if out-of-credit
                      ack_type   <= `ACK_T;
                      ack_colour <= colour;
                      ack_seq    <= seq_exp;
                      ack_rts    <= 1'b1;
                    end
        8'b0x11101x: begin  // ack next seq number if frame is OK
                       ack_type   <= `ACK_T;
                       ack_colour <= colour;
                       ack_seq    <= seq_exp + 1;
                       ack_rts    <= 1'b1;
                     end
        8'b0x1111xx,        // nack seq number if busy or
        8'b0x110xxx: begin  // received seq not equal to expected
                       ack_type   <= `NAK_T;
                       ack_colour <= ~colour; // colour changes with nack! 
                       ack_seq    <= seq_exp;
                       ack_rts    <= 1'b1;
                     end
        8'b10xxxxx1,        // resend nack if out-of-credit in wrong colour
        8'b0x10xxx1: begin  // resend nack if valid frame in wrong colour
                       ack_type   <= `NAK_T;
                       ack_colour <= colour; // colour not affected by resends! 
                       ack_seq    <= seq_exp;
                       ack_rts    <= 1'b1;
                     end
        default:    begin  // no ack/nack
                       ack_type   <= ack_type;   // no change!
                       ack_colour <= ack_colour; // no change!
                       ack_seq    <= ack_seq;    // no change!
                       ack_rts    <= 1'b0;
                     end
      endcase
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // register bank interface
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      reg_rfrm <= 1'b0;
    else
      reg_rfrm <= go_frm;  // valid frame and not busy!

  always @ (posedge clk or posedge rst)
    if (rst)
      reg_busy <= 1'b0;
    else
      reg_busy <= ok_frm && rjct_frm;  // valid frame but busy!

  always @ (posedge clk or posedge rst)
    if (rst)
      reg_lnak <= 1'b0;
    else
      reg_lnak <= ack_rts && (ack_type == `NAK_T);  // nak sent!

  always @ (posedge clk or posedge rst)
    if (rst)
      reg_lack <= 1'b0;
    else
      reg_lack <= ack_rts && (ack_type == `ACK_T);  // ack sent!
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	 
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------- control --------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // colour
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      colour <= `CLR_BITS'd0;
    else
      if (ok_colour && (rjct_frm || (frm_seq != seq_exp)))
        colour <= ~colour;  // change colour for initial nack!
      else
        colour <= colour;   // no change!
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // sequence number
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      seq_exp <= {`SEQ_BITS {1'b0}};
    else
      if (go_frm)
        seq_exp <= seq_exp + 1;
      else
        seq_exp <= seq_exp;  // no change!
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // frame validity control (combinatorial)
  //---------------------------------------------------------------
  // reject frame if at least one busy packet store
  always @ (*)
    rjct_frm = (busy != 0);
//#    rjct_frm = (busy[0] || busy[1] || busy[2] || busy[3]
//#                 || busy[4] || busy[5] || busy[6] || busy[7]
//#               );
//#    rjct_frm = 1'b0;

  // valid frame with correct colour
  always @ (*)
    ok_colour = frm_vld && (frm_colour == colour);

  // valid frame with correct colour and correct sequence number
  always @ (*)
    ok_frm = ok_colour && (frm_seq == seq_exp);

  // valid frame that will go into packet stores
  always @ (*)
    go_frm = ok_frm && !rjct_frm;
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // ack send counter
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      ack_snd_ctr <= `ACK_CNT;
    else
      casex ({go_frm, (ack_snd_ctr == 0)})
        2'b11:   ack_snd_ctr <= `ACK_CNT;  // send ack: restart
        2'b10:   ack_snd_ctr <= ack_snd_ctr - 1;
        default: ack_snd_ctr <= ack_snd_ctr;  // no change!
      endcase
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // nack resend counter
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      nak_rsn_ctr <= `NAK_CNT;
    else
      casex ({ooc_vld, (ooc_colour == colour),
              frm_vld, (frm_colour == colour), (nak_rsn_ctr == 0)
             }
            )
        5'b11xxx,                              // valid ooc,
        5'bxx11x,                              // valid frame,
        5'b10xx1,                              // resend ooc nack,
        5'bxx101: nak_rsn_ctr <= `NAK_CNT;     // resend nack: restart

        5'b10xx0,
        5'bxx100: nak_rsn_ctr <= nak_rsn_ctr - 1;

        default:  nak_rsn_ctr <= nak_rsn_ctr;  // no change!
      endcase
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

endmodule
