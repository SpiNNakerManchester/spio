// -------------------------------------------------------------------------
//  $Id: rxc.v 2515 2013-08-19 07:50:12Z plana $
// spiNNlink receiver control module
//
// -------------------------------------------------------------------------
// AUTHOR
//  lap - luis.plana@manchester.ac.uk
//
// -------------------------------------------------------------------------
// DETAILS
//  Created on       : 28 Nov 2012
//  Version          : $Revision: 2515 $
//  Last modified on : $Date: 2013-08-19 08:50:12 +0100 (Mon, 19 Aug 2013) $
//  Last modified by : $Author: plana $
//  $HeadURL: https://solem.cs.man.ac.uk/svn/spiNNlink/testing/src/rxc.v $
//
// -------------------------------------------------------------------------
// COPYRIGHT
//  Copyright (c) The University of Manchester, 2012. All rights reserved.
//  SpiNNaker Project
//  Advanced Processor Technologies Group
//  School of Computer Science
// -------------------------------------------------------------------------
//
// TODO
// -------------------------------------------------------------------------


// ----------------------------------------------------------------
// include spiNNlink global constants and parameters
//
`include "spio_hss_multiplexer_common.h"
// ----------------------------------------------------------------

`timescale 1ns / 1ps
module rxc(
    input  wire        clk,
    input  wire        rst,

    input  wire [31:0] gtp_data,
    input  wire  [3:0] gtp_kchr,
    input  wire  [3:0] gtp_is_comma,
    input  wire        gtp_byte_is_aligned,
    input  wire  [1:0] gtp_los,
    output reg         gtp_align_comma,

    output reg   [1:0] rx_state,

    output reg  [31:0] rx_out_data,
    output reg   [3:0] rx_out_kchr,
    output reg         rx_out_vld
    );

  //---------------------------------------------------------------
  // constants
  //---------------------------------------------------------------
  localparam BYTE0 = 4'b0001;
  localparam BYTE1 = 4'b0010;
  localparam BYTE2 = 4'b0100;
  localparam BYTE3 = 4'b1000;

  localparam STATE_BITS = 2;

  localparam SYNC_BITS = 7;
  localparam SYNC_CNT  = 100;


  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  reg               [3:0] alignment;
  reg              [31:8] data_buffer;  // no need to store least-sig. byte
  reg               [3:1] kchr_buffer;
   
  reg              [31:0] rx_aligned_data;
  reg               [3:0] rx_aligned_kchr;
  reg                     rx_aligned_vld;

  reg                     gtp_vld;
  reg                     is_sync;
  reg                     is_clkc;
  reg                     is_ok;

  reg  [STATE_BITS - 1:0] state;
  reg   [SYNC_BITS - 1:0] cnt_00;
  reg   [SYNC_BITS - 1:0] cnt_01;
  reg   [SYNC_BITS - 1:0] cnt_10;
  reg   [SYNC_BITS - 1:0] cnt_11;


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //--------------------------- datapath --------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  always @ (posedge clk or posedge rst)
    if (rst)
      gtp_align_comma <= 1'b1;
    else
      if (state == 2'b11)
        gtp_align_comma <= 1'b0;


  always @ (posedge clk or posedge rst)
    if (rst)
      alignment <= 4'b0000;
    else
      if (gtp_vld)
        if (gtp_is_comma != 4'b0000)
          alignment <= gtp_is_comma;  // new comma position
        else
          alignment <= alignment;     // no change!
      else
        alignment <= 4'b0000;         // lost synchronisation


  always @ (posedge clk or posedge rst)
    if (rst)
      data_buffer <= 24'd0; // not really necessary!
    else
      if (gtp_vld)
        data_buffer <= gtp_data[31:8];


  always @ (posedge clk or posedge rst)
    if (rst)
      kchr_buffer <= 3'd0; // not really necessary!
    else
      if (gtp_vld)
        kchr_buffer <= gtp_kchr[3:1];


  always @ (*)
    case (alignment)
      BYTE0:   rx_aligned_data = {gtp_data[7:0],  data_buffer[31:8]};
      BYTE1:   rx_aligned_data = {gtp_data[15:0], data_buffer[31:16]};
      BYTE2:   rx_aligned_data = {gtp_data[23:0], data_buffer[31:24]};
      BYTE3:   rx_aligned_data =  gtp_data;
      default: rx_aligned_data =  gtp_data;
    endcase


  always @ (*)
    case (alignment)
      BYTE0:   rx_aligned_kchr = {gtp_kchr[0],   kchr_buffer[3:1]};
      BYTE1:   rx_aligned_kchr = {gtp_kchr[1:0], kchr_buffer[3:2]};
      BYTE2:   rx_aligned_kchr = {gtp_kchr[2:0], kchr_buffer[3]};
      BYTE3:   rx_aligned_kchr =  gtp_kchr;
      default: rx_aligned_kchr =  gtp_kchr;
    endcase


  always @ (*)
      rx_aligned_vld = gtp_vld && (alignment != 4'b0000);


  always @ (posedge clk or posedge rst)
    if (rst)
      rx_out_data <= 32'd0; // not really necessary!
    else
      if (is_ok)
        rx_out_data <= rx_aligned_data;


  always @ (posedge clk or posedge rst)
    if (rst)
      rx_out_kchr <= 4'b0000; // not really necessary!
    else
      if (is_ok)
        rx_out_kchr <= rx_aligned_kchr;


  always @ (posedge clk or posedge rst)
    if (rst)
      rx_out_vld <= 1'b0;
    else
      rx_out_vld <= is_ok;


  always @ (posedge clk or posedge rst)
    if (rst)
      rx_state <= 2'b00;
    else
      rx_state <= state;


  always @ (posedge clk or posedge rst)
    if (rst)
      state <= 2'b00;
    else
       case (state)
	2'b00:
          if ((cnt_00 >= SYNC_CNT) || (cnt_01 > SYNC_CNT))
            state <= 2'b01;
          else
            state <= state;  // no change!

        2'b01:
          if ((cnt_01 >= SYNC_CNT) || (cnt_10 > SYNC_CNT))
            state <= 2'b10;
          else
            state <= state;  // no change!

        2'b10:
          if ((cnt_10 >= SYNC_CNT) || (cnt_11 > SYNC_CNT))
            state <= 2'b11;
          else
            state <= state;  // no change!

        2'b11:
            state <= 2'b11;  // no change!

        default:
            state <= 2'b00;
      endcase


  always @ (posedge clk or posedge rst)
    if (rst)
      cnt_00 <= 0;
    else
      if (is_sync && (rx_aligned_data[23:16] == 8'h00))
       cnt_00 <= cnt_00 + 1;


  always @ (posedge clk or posedge rst)
    if (rst)
      cnt_01 <= 0;
    else
      if (is_sync && (rx_aligned_data[23:16] == 8'h01))
       cnt_01 <= cnt_01 + 1;


  always @ (posedge clk or posedge rst)
    if (rst)
      cnt_10 <= 0;
    else
      if (is_sync && (rx_aligned_data[23:16] == 8'h02))
       cnt_10 <= cnt_10 + 1;


  always @ (posedge clk or posedge rst)
    if (rst)
      cnt_11 <= 0;
    else
      if (is_sync && (rx_aligned_data[23:16] == 8'h03))
       cnt_11 <= cnt_11 + 1;


  always @ (*)
      gtp_vld = ((gtp_byte_is_aligned == 1'b1) || (state == 2'b11))  //#
                  && (gtp_los[1] == 1'b0);

  always @ (*)
      is_ok = rx_aligned_vld && !is_sync && !is_clkc && (state == 2'b11);

  always @ (*)
      is_sync = (rx_aligned_kchr == 4'b1010)
                  && (rx_aligned_data[31:24] == `COMMA)
                  && (rx_aligned_data[15:8] == `SYNC);

  always @ (*)
      is_clkc = (rx_aligned_kchr == 4'b0001)
                  && (rx_aligned_data[7:0] == `CLKC);
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

endmodule
