// -------------------------------------------------------------------------
// $Id: packet_receiver.v 2517 2013-08-19 09:33:30Z plana $
//  spiNNaker link receiver module
//
// -------------------------------------------------------------------------
// AUTHOR
//  lap - luis.plana@manchester.ac.uk
//  Based on work by J Pepper (Date 08/08/2012)
//
// -------------------------------------------------------------------------
// Taken from:
// https://solem.cs.man.ac.uk/svn/spiNNlink/testing/src/packet_receiver.v
// Revision 2517 (Last-modified date: Date: 2013-08-19 10:33:30 +0100)
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
`include "spio_spinnaker_link.h"
// ----------------------------------------------------------------


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//----------------------- internal modules ----------------------
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`timescale 1ns / 1ps
module pkt_reg
(
  input  wire 			 CLK_IN,
  input  wire 			 RESET_IN,

  // status
  output reg                     busy,

  // incoming packet
  input  wire  [`PKT_BITS - 1:0] ipkt_data,
  input  wire 			 ipkt_vld,

  // output packet
  output reg   [`PKT_BITS - 1:0] PKT_DATA_OUT,
  output reg  			 PKT_VLD_OUT,
  input  wire 			 PKT_RDY_IN
);

  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  reg full;


  //---------------------------------------------------------------
  // output packet interface
  //---------------------------------------------------------------
  always @ (posedge CLK_IN)
    if (ipkt_vld && !busy)
      PKT_DATA_OUT <= ipkt_data;
    else
      PKT_DATA_OUT <= PKT_DATA_OUT;  // no change!

  //---------------------------------------------------------------
  // output packet valid if not empty (combinatorial)
  //---------------------------------------------------------------
  always @ (*)
    PKT_VLD_OUT = full;
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // status
  //---------------------------------------------------------------
  always @ (posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      full <= 1'b0;
    else
      casex ({ipkt_vld, PKT_RDY_IN})
        2'b1x:   full <= 1;
        2'b01:   full <= 0;
        default: full <= full;  // no change!
      endcase

  //---------------------------------------------------------------
  // cannot accept a new packet (combinatorial)
  //---------------------------------------------------------------
  always @ (*)
    busy = full && !PKT_RDY_IN;
  //---------------------------------------------------------------
endmodule
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`timescale 1ns / 1ps
module spio_spinnaker_link_receiver
(
  input                         CLK_IN,
  input                         RESET_IN,

  // SpiNNaker link asynchronous interface
  input                   [6:0] SL_DATA_2OF7_IN,
  output reg                    SL_ACK_OUT,

  // synchronous interface
  output wire [`PKT_BITS - 1:0] PKT_DATA_OUT,
  output wire                   PKT_VLD_OUT,
  input                         PKT_RDY_IN
);

  //---------------------------------------------------------------
  // constants
  //---------------------------------------------------------------
  //# Xilinx recommends one-hot state encoding
  localparam STATE_BITS = 2;
  localparam REST_ST    = 0;
  localparam IDLE_ST    = REST_ST + 1;
  localparam TRAN_ST    = IDLE_ST + 1;
  localparam ERR_ST     = TRAN_ST + 1;


  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
(* clock_signal = "no" *)  reg  [STATE_BITS - 1:0] state;
(* clock_signal = "no" *)  reg                     nsb;  // new symbol arrived
(* clock_signal = "no" *)  reg                     dat;  // data symbol
(* clock_signal = "no" *)  reg                     eop;  // end-of-packet symbol
(* clock_signal = "no" *)  reg                     oob;  // out-of-band symbol (error)

(* clock_signal = "no" *)  reg               [6:0] old_data_2of7;
(* clock_signal = "no" *)  reg                     tgl_ack;

(* clock_signal = "no" *)  reg               [3:0] symbol;
(* clock_signal = "no" *)  reg                     long_pkt;
(* clock_signal = "no" *)  reg               [4:0] symbol_cnt;
(* clock_signal = "no" *)  reg                     exp_eop;

(* clock_signal = "no" *)  reg   [`PKT_BITS - 1:0] pkt_buf;
(* clock_signal = "no" *)  reg   [`PKT_BITS - 1:0] ipkt_data;
(* clock_signal = "no" *)  reg 	                  ipkt_vld;
(* clock_signal = "no" *)  wire                    busy;


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //--------------------------- structure -------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // output packet interface module
  //---------------------------------------------------------------
  pkt_reg prg
  (
    .CLK_IN       (CLK_IN),
    .RESET_IN     (RESET_IN),

    .busy         (busy),

    .ipkt_data    (ipkt_data),
    .ipkt_vld     (ipkt_vld),

    .PKT_DATA_OUT (PKT_DATA_OUT),
    .PKT_VLD_OUT  (PKT_VLD_OUT),
    .PKT_RDY_IN   (PKT_RDY_IN)
  );
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //--------------------------- functions -------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // NRZ 2-of-7 decoder
  //---------------------------------------------------------------
  function [3:0] decode_2of7 ;
    input [6:0] data;
    input [6:0] old_data;

    case (data ^ old_data)
      7'b0010001: decode_2of7 = 0;    // 0
      7'b0010010: decode_2of7 = 1;    // 1
      7'b0010100: decode_2of7 = 2;    // 2
      7'b0011000: decode_2of7 = 3;    // 3
      7'b0100001: decode_2of7 = 4;    // 4
      7'b0100010: decode_2of7 = 5;    // 5
      7'b0100100: decode_2of7 = 6;    // 6
      7'b0101000: decode_2of7 = 7;    // 7
      7'b1000001: decode_2of7 = 8;    // 8
      7'b1000010: decode_2of7 = 9;    // 9
      7'b1000100: decode_2of7 = 10;   // 10
      7'b1001000: decode_2of7 = 11;   // 11
      7'b0000011: decode_2of7 = 12;   // 12
      7'b0000110: decode_2of7 = 13;   // 13
      7'b0001100: decode_2of7 = 14;   // 14
      7'b0001001: decode_2of7 = 15;   // 15
      default:    decode_2of7 = 4'hx; // eop, incomplete, oob
    endcase
  endfunction
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //--------------------------- datapath --------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // SpiNNaker link interface
  //---------------------------------------------------------------
  always @(posedge tgl_ack or posedge RESET_IN)
    if (RESET_IN)
      SL_ACK_OUT <= 1'b0;
    else
      SL_ACK_OUT <= ~SL_ACK_OUT;

  always @(*)
    if (RESET_IN)
      tgl_ack = 1'b0;
    else
      case (state)
	REST_ST:   tgl_ack = 1'b1;   // mimic SpiNNaker behaviour

	TRAN_ST: casex ({dat, oob, eop, exp_eop, busy})
	           5'b1xxxx,  // data symbol: ack and wait for next symbol
		   5'bx1xxx,  // oob symbol: ack and go to error state
	           5'bxx10x,  // unexpected eop: ack and start new packet
	           5'bxx110:  // expected eop and not busy: ack and send packet 
                       tgl_ack = 1'b1;       // ack new symbol

		   // 5'bxx111,  // expected eop but busy: don't ack yet
		   // 5'b000xx,  // no symbol arrived: don't ack
		   default:
	               tgl_ack = 1'b0;       // no change!
                 endcase

        IDLE_ST,
        ERR_ST:  if (nsb)  
	           tgl_ack = 1'b1;           // ack new symbol
                 else
                   tgl_ack = 1'b0;           // no change!

        default:   tgl_ack = 1'b0;           // no change!
      endcase 

  always @(posedge CLK_IN)
    case (state)
	REST_ST:   old_data_2of7 <= SL_DATA_2OF7_IN;  // remember 2of7 data

	TRAN_ST: casex ({dat, oob, eop, exp_eop, busy})
	           5'b1xxxx,  // data symbol: lacth and wait for next symbol
		   5'bx1xxx,  // oob symbol: latch and go to error state
	           5'bxx10x,  // unexpected eop: latch and start new packet
	           5'bxx110:  // expected eop and not busy: latch and send packet 
                     old_data_2of7 <= SL_DATA_2OF7_IN;  // remember 2of7 data

		   // 5'bxx111,  // expected eop but busy: wait
		   // 5'b000xx,  // no symbol arrived: wait
		   default:
                     old_data_2of7 <= old_data_2of7;  // no change!
               endcase

      IDLE_ST,
      ERR_ST:  if (nsb)  
                 old_data_2of7 <= SL_DATA_2OF7_IN;    // remember 2of7 data
               else
                 old_data_2of7 <= old_data_2of7;  // no change!

      default:   old_data_2of7 <= old_data_2of7;  // no change!
    endcase 
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // packet assembly
  //---------------------------------------------------------------
  // shift in new symbol to assemble packet
  always @(posedge CLK_IN)
    if (dat)
      pkt_buf <= {symbol, pkt_buf[`PKT_BITS - 1:4]};

  // keep track of number of symbols received to check for frame errors
  always @(posedge CLK_IN)
    case (state)
	IDLE_ST: if (dat)
                 symbol_cnt <= 5'd1;  // count first symbol in packet
               else
                 symbol_cnt <= 5'd0;  // init count

	TRAN_ST: if (dat)
		   symbol_cnt <= symbol_cnt + 1;  // count new symbol
               else
		   symbol_cnt <= symbol_cnt;      // no change!

	default:   symbol_cnt <= 5'd0;  // init count
    endcase 

  // remember expected packet size
  always @(posedge CLK_IN)
    case (state)
	IDLE_ST: if (dat)
                 long_pkt <= symbol[1];

      default:   long_pkt <= long_pkt;   // no change!
    endcase 
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // decoded 2-of-7 symbol (combinatorial)
  //---------------------------------------------------------------
  always @(*)
    symbol = decode_2of7(SL_DATA_2OF7_IN, old_data_2of7);
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // packet register interface
  //---------------------------------------------------------------
  // handshake on packet register interface (combinatorial)
  always @(*)
      ipkt_vld = ((state == TRAN_ST) && eop && exp_eop);

  // select appropriate buffer data (combinatorial)
  always @(*)
    if (long_pkt)
      ipkt_data = pkt_buf;
    else
      ipkt_data = {{(`PKT_BITS - 40) {1'b0}},  // padding!
                    pkt_buf[(`PKT_BITS - 40) +: 40]
                  }; //#
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------- control --------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // new symbol arrived (combinatorial)
  //---------------------------------------------------------------
  always @(*)
    case (SL_DATA_2OF7_IN ^ old_data_2of7)
      0, 1, 2, 4,
      8, 16, 32,
      64:         nsb = 0;  // incomplete (single-bit change)
      default:    nsb = 1;  // correct data, eop or oob 
    endcase
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // data symbol arrived (combinatorial)
  //---------------------------------------------------------------
//#  always @(*)
//#    dat = nsb && !eop && !oob;

  always @(*)
    case (SL_DATA_2OF7_IN ^ old_data_2of7)
      7'b0010001: dat = 1;  // 0
      7'b0010010: dat = 1;  // 1
      7'b0010100: dat = 1;  // 2
      7'b0011000: dat = 1;  // 3
      7'b0100001: dat = 1;  // 4
      7'b0100010: dat = 1;  // 5
      7'b0100100: dat = 1;  // 6
      7'b0101000: dat = 1;  // 7
      7'b1000001: dat = 1;  // 8
      7'b1000010: dat = 1;  // 9
      7'b1000100: dat = 1;  // 10
      7'b1001000: dat = 1;  // 11
      7'b0000011: dat = 1;  // 12
      7'b0000110: dat = 1;  // 13
      7'b0001100: dat = 1;  // 14
      7'b0001001: dat = 1;  // 15
      default:    dat = 0;  // anything else is not correct data
    endcase
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // out-of-band symbol arrived (combinatorial)
  //---------------------------------------------------------------
  always @(*)
    case (SL_DATA_2OF7_IN ^ old_data_2of7)
      7'b0010001: oob = 0;  // 0
      7'b0010010: oob = 0;  // 1
      7'b0010100: oob = 0;  // 2
      7'b0011000: oob = 0;  // 3
      7'b0100001: oob = 0;  // 4
      7'b0100010: oob = 0;  // 5
      7'b0100100: oob = 0;  // 6
      7'b0101000: oob = 0;  // 7
      7'b1000001: oob = 0;  // 8
      7'b1000010: oob = 0;  // 9
      7'b1000100: oob = 0;  // 10
      7'b1001000: oob = 0;  // 11
      7'b0000011: oob = 0;  // 12
      7'b0000110: oob = 0;  // 13
      7'b0001100: oob = 0;  // 14
      7'b0001001: oob = 0;  // 15
      7'b1100000: oob = 0;  // eop
      0, 1, 2, 4,
      8, 16, 32,
      64:         oob = 0;  // incomplete (single-bit change)
      default:    oob = 1;  // anything else is oob
    endcase
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // end-of-packet symbol arrived (combinatorial)
  //---------------------------------------------------------------
  always @(*)
    case (SL_DATA_2OF7_IN ^ old_data_2of7)
      7'b1100000: eop = 1;
      default:    eop = 0;
    endcase
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // expect an end-of-packet symbol (combinatorial)
  //---------------------------------------------------------------
  always @ (*)
    exp_eop = (!long_pkt && (symbol_cnt == 10)) || (symbol_cnt == 18);
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // state
  //---------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      state <= REST_ST;
    else
      case (state)
	REST_ST: state <= IDLE_ST;

	IDLE_ST: casex ({dat, oob})
                   2'b1x:   state <= TRAN_ST;  // first symbol in new packet
                   2'bx1:   state <= ERR_ST;   // error, drop packet
                   default: state <= IDLE_ST;  // no change!
                 endcase

	TRAN_ST: casex ({dat, oob, eop, exp_eop, busy})
		   5'bx1xxx:  // oob symbol: drop and go to error
                     state <= ERR_ST;

	           5'bxx10x,  // unexpected eop: drop packet
	           5'bxx110:  // expected eop and not busy: ack and send packet 
                     state <= IDLE_ST;

	           // 5'b1xxxx,  // data symbol: ack and wait for next symbol
		   // 5'bxx111,  // expected eop but busy: wait
                   default:   // no symbol
                     state <= TRAN_ST;
                 endcase

        ERR_ST:  if (eop)
                   state <= IDLE_ST;
                 else
                   state <= ERR_ST;   // no change!

        default: state <= state;  // no change!
      endcase
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

endmodule

