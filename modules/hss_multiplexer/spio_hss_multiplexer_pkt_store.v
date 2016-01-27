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
  input wire 		       clk,
  input wire 		       rst,

  // status (for register interface)
  // empty means no unread data in buffer
  // there can be unacked data in buffer!
  output reg 		       empty,
  output reg 		       full,

  // Force the multiplexer to stop accepting new packets. The first packet to
  // arrive on or after the signal is asserted will still be accepted.
  input wire 		       stop,

  // remote channel flow control interface
  input wire 		       cfc_rem,
 
  // ack/nack interface
  input wire 		       vld_ack,
  input wire 		       vld_nak,
  input wire [`SEQ_BITS - 1:0] ack_seq,

  // incoming packet interface
  input wire [`PKT_BITS - 1:0] pkt_data,
  input wire 		       pkt_vld,
  output reg 		       pkt_rdy,

  // outgoing packet interface
  input wire [`SEQ_BITS - 1:0] bpkt_seq,
  output reg [`PKT_BITS - 1:0] bpkt_data,
  output reg 		       bpkt_pres,
  input wire 		       bpkt_rq,
  output reg 		       bpkt_gt
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
  // read request will succeed
  always @ (*)
    reading = bpkt_rq && unread && cfc_rem;

  // write request will succeed
  always @ (*)
    writing = pkt_vld && pkt_rdy && !full;

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
    nxt_unread = (nxt_br != nxt_bw);
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // buffer full flag
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      full <= 1'b0;
    else
      full <= nxt_full;

  // make sure that the comparison below has the right size operands!
  reg [`BUF_BITS - 1:0] inc_bw;
  always @ (*)
   inc_bw = nxt_bw + 1;

  always @ (*)
   nxt_full = (nxt_ba == inc_bw);
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

  // to simplify ack/nak treatment, ack_seq is used in the same way
  // in both: frame ack_seq is not ack'ed, all previous ones are.
  // Additionally, a nak requests that frame ack_seq is re-sent.
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
      if (stop)
        begin
          // If stopping has been requested we cannot change the ready state
          // except after a packet has been received (due to the rdy/vld
          // protocol).
          if (pkt_vld)
            pkt_rdy <= 1'b0;
          else
            pkt_rdy <= pkt_rdy;  // No change
        end
      else
        pkt_rdy <= !nxt_full;
      
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
endmodule
