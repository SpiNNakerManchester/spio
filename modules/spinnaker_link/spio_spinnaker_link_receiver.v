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
`include "spio_spinnaker_link_definitions.v"
// ----------------------------------------------------------------


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//----------------------- internal modules ----------------------
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`timescale 1ns / 1ps
module pkt_reg
(
  input  wire 			 clk,
  input  wire 			 rst,

  // packet counter interface
  output reg                     ctr_pkt,

  // status
  output reg                     busy,

  // incoming packet
  input  wire  [`PKT_BITS - 1:0] ipkt_data,
  input  wire 			 ipkt_vld,

  // output packet
  output reg   [`PKT_BITS - 1:0] pkt_data,
  output reg  			 pkt_vld,
  input  wire 			 pkt_rdy
);

  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  reg full;


  //---------------------------------------------------------------
  // output packet interface
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      pkt_data <= {`PKT_BITS {1'b0}}; // not really necessary!
    else
      if (ipkt_vld && !busy)
        pkt_data <= ipkt_data;
      else
        pkt_data <= pkt_data;  // no change!

  //---------------------------------------------------------------
  // output packet valid if not empty (combinatorial)
  //---------------------------------------------------------------
  always @ (*)
    pkt_vld = full;
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // status
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      full <= 1'b0;
    else
      casex ({ipkt_vld, pkt_rdy})
        2'b1x:   full <= 1;
        2'b01:   full <= 0;
        default: full <= full;  // no change!
      endcase

  //---------------------------------------------------------------
  // cannot accept a new packet (combinatorial)
  //---------------------------------------------------------------
  always @ (*)
    busy = full && !pkt_rdy;
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // packet counter interface
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      ctr_pkt <= 1'b0;
    else
      ctr_pkt <= pkt_vld && pkt_rdy;
  //---------------------------------------------------------------
endmodule
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`timescale 1ns / 1ps
module packet_receiver
(
  input                         clk,
  input                         rst,

  // packet counter interface
  output wire                   ctr_pkt,

  // SpiNNaker link asynchronous interface
  input                   [6:0] data_2of7,
  output reg                    ack,

  // synchronous interface
  output wire [`PKT_BITS - 1:0] pkt_data,
  output wire                   pkt_vld,
  input                         pkt_rdy
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
  reg  [STATE_BITS - 1:0] state;
  reg                     nsb;  // new symbol arrived
  reg                     dat;  // data symbol
  reg                     eop;  // end-of-packet symbol
  reg                     oob;  // out-of-band symbol (error)

  reg               [6:0] old_data_2of7;

  reg               [3:0] symbol;
  reg                     long_pkt;
  reg               [4:0] symbol_cnt;
  reg                     exp_eop;

  reg   [`PKT_BITS - 1:0] pkt_buf;
  reg   [`PKT_BITS - 1:0] ipkt_data;
  reg 	                  ipkt_vld;
  wire                    busy;


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //--------------------------- structure -------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // output packet interface module
  //---------------------------------------------------------------
  pkt_reg prg
  (
    .clk       (clk),
    .rst       (rst),

    .ctr_pkt   (ctr_pkt),

    .busy      (busy),

    .ipkt_data (ipkt_data),
    .ipkt_vld  (ipkt_vld),

    .pkt_data  (pkt_data),
    .pkt_vld   (pkt_vld),
    .pkt_rdy   (pkt_rdy)
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
  always @(posedge clk or posedge rst)
    if (rst)
    begin
      old_data_2of7 <= 7'd0;  // not really necessary!
      ack           <= 1'b0;
    end
    else
      case (state)
	REST_ST: begin
                   old_data_2of7 <= data_2of7;  // remember 2of7 data
	           ack           <= 1'b1;       // mimic SpiNNaker behaviour
                 end

	TRAN_ST: casex ({dat, oob, eop, exp_eop, busy})
	           5'b1xxxx,  // data symbol: ack and wait for next symbol
		   5'bx1xxx,  // oob symbol: ack and go to error state
	           5'bxx10x,  // unexpected eop: ack and start new packet
	           5'bxx110:  // expected eop and not busy: ack and send packet 
                     begin
                       old_data_2of7 <= data_2of7;  // remember 2of7 data
                       ack           <= ~ack;       // ack new symbol
                     end

		   // 5'bxx111,  // expected eop but busy: don't ack yet
		   // 5'b000xx,  // no symbol arrived: don't ack
		   default:
                     begin
                       old_data_2of7 <= old_data_2of7;  // no change!
	               ack           <= ack;            // no change!
	             end
                 endcase

        IDLE_ST,
        ERR_ST:  if (nsb)  
                 begin
                   old_data_2of7 <= data_2of7;      // remember 2of7 data
	           ack           <= ~ack;           // ack new symbol
	         end
                 else
                 begin
                   old_data_2of7 <= old_data_2of7;  // no change!
                   ack           <= ack;            // no change!
                 end

        default: begin
                   old_data_2of7 <= old_data_2of7;  // no change!
                   ack           <= ack;            // no change!
                 end
      endcase 
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // packet assembly
  //---------------------------------------------------------------
  // shift in new symbol to assemble packet
  always @(posedge clk or posedge rst)
    if (rst)
      pkt_buf <= `PKT_BITS'b0;  // not really necessary!
    else
      if (dat)
        pkt_buf <= {symbol, pkt_buf[`PKT_BITS - 1:4]};

  // keep track of number of symbols received to check for frame errors
  always @(posedge clk or posedge rst)
    if (rst)
      symbol_cnt <= 5'd0;  // not really necessary!
    else
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
  always @(posedge clk or posedge rst)
    if (rst)
      long_pkt <= 1'b0;  // not really necessary!
    else
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
    symbol = decode_2of7(data_2of7, old_data_2of7);
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
    case (data_2of7 ^ old_data_2of7)
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
    case (data_2of7 ^ old_data_2of7)
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
    case (data_2of7 ^ old_data_2of7)
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
    case (data_2of7 ^ old_data_2of7)
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
  always @(posedge clk or posedge rst)
    if (rst)
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

