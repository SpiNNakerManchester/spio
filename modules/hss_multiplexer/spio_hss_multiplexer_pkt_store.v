// -------------------------------------------------------------------------
// $Id$
//  spiNNlink packet store module used in frame assempler
//
// -------------------------------------------------------------------------
// AUTHOR
//  lap - luis.plana@manchester.ac.uk
//
// -------------------------------------------------------------------------
// DETAILS
//  Created on       : 28 Nov 2013
//  Version          : $Revision$
//  Last modified on : $Date$
//  Last modified by : $Author$
//  $HeadURL$
//
// -------------------------------------------------------------------------
// COPYRIGHT
//  Copyright (c) The University of Manchester, 2013. All rights reserved.
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
module spio_hss_multiplexer_pkt_store
(
  input  wire 			 clk,
  input  wire 			 rst,

  // status (for register interface)
  output reg                     empty,
  output reg                     full,

  // remote channel flow control interface
  input  wire                    cfc_rem,
 
  // ack/nack interface
  input  wire 			 vld_ack,
  input  wire 			 vld_nak,
  input  wire  [`SEQ_BITS - 1:0] ack_seq,
  input  wire                    rep_seq,

  // incoming packet interface
  input  wire  [`PKT_BITS - 1:0] pkt_data,
  input  wire 			 pkt_vld,
  output reg                     pkt_rdy,

  // outgoing packet interface
  input  wire  [`SEQ_BITS - 1:0] bpkt_seq,
  output reg   [`PKT_BITS - 1:0] bpkt_data,
  output reg                     bpkt_pres,
  input  wire                    bpkt_rq,
  output reg                     bpkt_gt
);

  //---------------------------------------------------------------
  // constants
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  reg   [`BUF_BITS - 1:0] ba;      // acknowledge pending pointer
  reg   [`BUF_BITS - 1:0] br;      // read pointer
  reg   [`BUF_BITS - 1:0] bo;      // output pointer
  reg   [`BUF_BITS - 1:0] bw;      // write pointer
  reg   [`BUF_BITS - 1:0] nxt_ba;
  reg   [`BUF_BITS - 1:0] nxt_br;
  reg   [`BUF_BITS - 1:0] nxt_bw;

  reg                     nxt_full;

  reg                     unread;      // unread data present!
  reg                     nxt_unread;

  reg                     reading;
  reg                     writing;


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //--------------------------- datapath --------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // internal packet buffers (for frame resend)
  //---------------------------------------------------------------
  reg   [`PKT_BITS - 1:0] pkt_dbuf [0 : `BUF_LEN - 1];

  //---------------------------------------------------------------
  // sequence <-> register mapping
  //---------------------------------------------------------------
  reg   [`BUF_BITS - 1:0] seq_map [0 : `BUF_LEN - 1];
  reg   [`BUF_BITS - 1:0] seq_to_reg;

  //---------------------------------------------------------------
  // buffer operations in progress
  //---------------------------------------------------------------
  always @ (*)
    reading = bpkt_rq && unread;  // read request will succeed
//# reading = bpkt_rq && unread && cfc_rem;  // read request will succeed

  always @ (*)
    writing = pkt_vld && !full;   // write request will succeed

  //---------------------------------------------------------------
  // buffer empty flag (used only for reporting)
  //---------------------------------------------------------------
  always @ (*)
    empty = !unread;

  //---------------------------------------------------------------
  // buffer unread flag
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      unread <= 1'b0;
    else
      unread <= nxt_unread;

  always @ (*)
    case ({vld_nak, vld_ack, reading, writing})
      4'b0001,  // writing
      4'b0011,  // reading and writing
      4'b0101,  // acked and writing
      4'b0111,  // acked, reading and writing
      4'b1001,  // nacked and writing
      4'b1011:  // nacked, reading and writing
               nxt_unread = 1;

      4'b0010,  // reading
      4'b0110:  // acked and reading
               nxt_unread = (nxt_br != bw);

      4'b1000,  // nacked
      4'b1010:  // nacked and reading
               nxt_unread = (nxt_br != bw) || unread;

      default: nxt_unread = unread;
    endcase
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // buffer full flag
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      full <= 1'b0;
    else
      full <= nxt_full;

  // reading does not free space in the buffer!
  // ack/nack can free space, if not a repeat!
  always @ (*)
    case ({vld_nak, vld_ack, writing})
      3'b001:  // writing
               nxt_full = (nxt_bw == ba);

      3'b010,  // acked
      3'b100:  // nacked
               nxt_full = full && rep_seq;

      3'b011,  // acked and writing
      3'b101:  // nacked and writing
               nxt_full = (nxt_bw == ba) && rep_seq;

      default: nxt_full = full;
    endcase
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // buffer write pointer
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      bw <= `BUF_BITS'd0;
    else
      bw <= nxt_bw;

  always @ (*)
    if (writing)
      nxt_bw = bw + 1;
    else
      nxt_bw = bw;     // no change!
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // buffer read pointer
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      br <= `BUF_BITS'd0;
    else
      br <= nxt_br;

  always @ (*)
    casex ({vld_nak, reading})
        2'b1x:   nxt_br = seq_to_reg;  // nack'd frame
        2'b01:   nxt_br = br + 1;      // next in sequence
        default: nxt_br = br;          // no change!
    endcase
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // buffer ack pending pointer
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      ba <= `BUF_BITS'd0;
    else
      ba <= nxt_ba;

  // to simplify ack/nak treatment, ack_seq is used in a similar
  // in both: frame ack_seq is not ack'ed, all previous ones are.
  // Additionally, a nak also requests that frame ack_seq is re-sent.
  always @ (*)
    casex ({vld_nak, vld_ack, (ack_seq == bpkt_seq)})
        3'b1xx,                        // nacks ack implicitly
        3'bx10:  nxt_ba = seq_to_reg;  // valid ack

        3'bx11:  nxt_ba = br;          // no pending acks

        default: nxt_ba = ba;          // no change!
    endcase
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // buffer output pointer
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      bo <= `BUF_BITS'd0;
    else
      if (reading)
        bo <= br;
      else
        bo <= bo;  // no change!
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // packet buffer writes
  //---------------------------------------------------------------
  always @ (posedge clk)
    if (writing)
      pkt_dbuf[bw] <= pkt_data;
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // sequence map writes
  //---------------------------------------------------------------
  always @ (posedge clk)
    if (bpkt_rq)
      seq_map[bpkt_seq[`BUF_BITS - 1:0]] <= br;  // even if not reading!
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // sequence map reads
  //---------------------------------------------------------------
  always @ (*)
     seq_to_reg = seq_map[ack_seq[`BUF_BITS - 1:0]];
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // buffered packet data to frame issue
  //---------------------------------------------------------------
  always @ (*)
    bpkt_data = pkt_dbuf[bo];
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // buffered packet data presence to frame issue
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      bpkt_pres <= 1'b0;
    else
      if (bpkt_rq)
        bpkt_pres <= reading;
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // buffer to frame issue handshake
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      bpkt_gt <= 1'b0;
    else
      bpkt_gt <= reading;
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // packet interface handshake
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      pkt_rdy <= 1'b0;
    else
      pkt_rdy <= !nxt_full;
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
endmodule
