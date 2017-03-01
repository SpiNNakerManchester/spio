// -------------------------------------------------------------------------
// $Id: spinn_receiver.v 2644 2013-10-24 15:18:41Z plana $
// -------------------------------------------------------------------------
// COPYRIGHT
// Copyright (c) The University of Manchester, 2012. All rights reserved.
// SpiNNaker Project
// Advanced Processor Technologies Group
// School of Computer Science
// -------------------------------------------------------------------------
// Project            : bidirectional SpiNNaker link to AER device interface
// Module             : spiNNaker link receiver module
// Author             : lap/Jeff Pepper/Simon Davidson
// Status             : Review pending
// $HeadURL: https://solem.cs.man.ac.uk/svn/spinn_aer2_if/spinn_receiver.v $
// Last modified on   : $Date: 2013-10-24 16:18:41 +0100 (Thu, 24 Oct 2013) $
// Last modified by   : $Author: plana $
// Version            : $Revision: 2644 $
// -------------------------------------------------------------------------


`define PKT_BITS         72


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//------------------------ spinn_receiver -----------------------
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`timescale 1ns / 1ps
module spinn_receiver
(
  input                         clk,
  input                         rst,

  // status
  output reg                     err,

  // SpiNNaker link asynchronous interface
  input                   [6:0] data_2of7,
  output reg                    ack,

  // synchronous interface
  output reg  [`PKT_BITS - 1:0] pkt_data,
  output reg                    pkt_vld,
  input                         pkt_rdy
);

  //---------------------------------------------------------------
  // constants
  //---------------------------------------------------------------
  //# Xilinx recommends one-hot state encoding
  localparam STATE_BITS = 3;
  localparam REST_ST    = 0;
  localparam IDLE_ST    = REST_ST + 1;
  localparam TRAN_ST    = IDLE_ST + 1;
  localparam WTRD_ST    = TRAN_ST + 1;
  localparam ERR0_ST    = WTRD_ST + 1;
  localparam EOP0_ST    = ERR0_ST + 1;


  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  reg  [STATE_BITS - 1:0] state;
  reg                     busy;  // output not ready for next packet
  reg                     nsb;   // new symbol arrived
  reg                     dat;   // data symbol
  reg                     eop;   // end-of-packet symbol
  reg                     bad;   // incorrect symbol (error)

  reg               [6:0] old_data_2of7;

  reg               [3:0] symbol;
  reg                     long_pkt;
  reg               [4:0] symbol_cnt;
  reg                     exp_eop;

  reg               [4:0] err_cnt;

  reg   [`PKT_BITS - 1:0] pkt_buf;
  reg   [`PKT_BITS - 1:0] nxt_data;
  reg                     nxt_vld;


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
      default:    decode_2of7 = 4'hx; // eop, incomplete, bad
    endcase
  endfunction
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //------------------- SpiNNaker link interface ------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

        IDLE_ST: if (nsb)  
                 begin
                   old_data_2of7 <= data_2of7;      // remember 2of7 data
	           ack           <= ~ack;           // ack new symbol
	         end
                 else
                 begin
                   old_data_2of7 <= old_data_2of7;  // no change!
                   ack           <= ack;            // no change!
                 end

	TRAN_ST: if (dat || bad)
                 begin
                   old_data_2of7 <= data_2of7;  // remember 2of7 data
                   ack           <= ~ack;       // ack new symbol
                 end
                 else
                 begin
                   old_data_2of7 <= old_data_2of7;  // no change!
                   ack           <= ack;            // no change!
	         end

	EOP0_ST: if (!busy)  
                 begin
                   old_data_2of7 <= data_2of7;      // remember 2of7 data
	           ack           <= ~ack;           // ack new symbol
	         end
                 else
                 begin
                   old_data_2of7 <= old_data_2of7;  // no change!
                   ack           <= ack;            // no change!
                 end

        WTRD_ST: if (!busy)  
                 begin
                   old_data_2of7 <= data_2of7;      // remember 2of7 data
	           ack           <= ~ack;           // ack new symbol
	         end
                 else
                 begin
                   old_data_2of7 <= old_data_2of7;  // no change!
                   ack           <= ack;            // no change!
                 end

        ERR0_ST: if (nsb || (err_cnt == 0))  
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
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //------------------- output packet interface -------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // packet valid handshake
  //---------------------------------------------------------------
  always @(posedge clk or posedge rst)
    if (rst)
      pkt_vld <= 1'b0;  // not really necessary!
    else
      if (nxt_vld || busy)
        pkt_vld <= 1'b1;
      else
        pkt_vld <= 1'b0;
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // packet data
  //---------------------------------------------------------------
  always @(posedge clk or posedge rst)
    if (rst)
      pkt_data <= `PKT_BITS'd0;  // not really necessary!
    else
      case (state)
	EOP0_ST: if (exp_eop && !busy)
		   pkt_data <= nxt_data;
                 else
		   pkt_data <= pkt_data;  // no change!

	WTRD_ST: if (!busy)
		   pkt_data <= nxt_data;
                 else
		   pkt_data <= pkt_data;  // no change!

	default:   pkt_data <= pkt_data;  // no change!
      endcase 
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // select appropriate buffer data (combinatorial)
  //---------------------------------------------------------------
  always @(*)
    if (long_pkt)
      nxt_data[40 +: 32] = pkt_buf[40 +: 32];  //#
    else
      nxt_data[40 +: 32] = 32'd0;              //#
    

  always @(*)
    if (long_pkt)
      nxt_data[0 +: 40] = pkt_buf[0 +: 40];
    else
      nxt_data[0 +: 40] = pkt_buf[32 +: 40];  //#
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // new packet valid -- can send
  //---------------------------------------------------------------
  always @(*)
    case (state)
    	EOP0_ST: if (exp_eop && !busy)
                   nxt_vld <= 1'b1;
                 else
                   nxt_vld <= 1'b0;

    	WTRD_ST: if (!busy)
    	           nxt_vld <= 1'b1;
                 else
    	           nxt_vld <= 1'b0;

    	default:   nxt_vld <= 1'b0;
    endcase 
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //------------------------ packet assembly ----------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // 
  //---------------------------------------------------------------
  // shift in new symbol to assemble packet
  always @(posedge clk or posedge rst)
    if (rst)
      pkt_buf <= `PKT_BITS'd0;  // not really necessary!
    else
      if (((state == IDLE_ST) || (state == TRAN_ST)) && dat)
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

	default:   symbol_cnt <= symbol_cnt;      // no change!
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
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------- control --------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // busy (output packet interface not ready to accept)
  //---------------------------------------------------------------
  always @(*)
    busy = pkt_vld && !pkt_rdy;
  //---------------------------------------------------------------

/*  //---------------------------------------------------------------
  // new symbol arrived (combinatorial)
  //---------------------------------------------------------------
  always @(*)
    case (data_2of7 ^ old_data_2of7)
      0, 1, 2, 4,
      8, 16, 32,
      64:         nsb = 0;  // incomplete (single-bit/no change)
      default:    nsb = 1;  // correct data, eop or bad 
    endcase
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // data symbol arrived (combinatorial)
  //---------------------------------------------------------------
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
  // bad symbol arrived (combinatorial)
  //---------------------------------------------------------------
  always @(*)
    case (data_2of7 ^ old_data_2of7)
      7'b0010001: bad = 0;  // 0
      7'b0010010: bad = 0;  // 1
      7'b0010100: bad = 0;  // 2
      7'b0011000: bad = 0;  // 3
      7'b0100001: bad = 0;  // 4
      7'b0100010: bad = 0;  // 5
      7'b0100100: bad = 0;  // 6
      7'b0101000: bad = 0;  // 7
      7'b1000001: bad = 0;  // 8
      7'b1000010: bad = 0;  // 9
      7'b1000100: bad = 0;  // 10
      7'b1001000: bad = 0;  // 11
      7'b0000011: bad = 0;  // 12
      7'b0000110: bad = 0;  // 13
      7'b0001100: bad = 0;  // 14
      7'b0001001: bad = 0;  // 15
      7'b1100000: bad = 0;  // eop
      0, 1, 2, 4,
      8, 16, 32,
      64:         bad = 0;  // incomplete (single-bit/no change)
      default:    bad = 1;  // anything else is bad
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

*/  //---------------------------------------------------------------
  // bad symbol arrived (combinatorial)
  //---------------------------------------------------------------
  always @(*)
    case (data_2of7 ^ old_data_2of7)
      7'b0010001: begin dat = 1; eop = 0; nsb = 1; bad = 0; end  // 0
      7'b0010010: begin dat = 1; eop = 0; nsb = 1; bad = 0; end  // 1
      7'b0010100: begin dat = 1; eop = 0; nsb = 1; bad = 0; end  // 2
      7'b0011000: begin dat = 1; eop = 0; nsb = 1; bad = 0; end  // 3
      7'b0100001: begin dat = 1; eop = 0; nsb = 1; bad = 0; end  // 4
      7'b0100010: begin dat = 1; eop = 0; nsb = 1; bad = 0; end  // 5
      7'b0100100: begin dat = 1; eop = 0; nsb = 1; bad = 0; end  // 6
      7'b0101000: begin dat = 1; eop = 0; nsb = 1; bad = 0; end  // 7
      7'b1000001: begin dat = 1; eop = 0; nsb = 1; bad = 0; end  // 8
      7'b1000010: begin dat = 1; eop = 0; nsb = 1; bad = 0; end  // 9
      7'b1000100: begin dat = 1; eop = 0; nsb = 1; bad = 0; end  // 10
      7'b1001000: begin dat = 1; eop = 0; nsb = 1; bad = 0; end  // 11
      7'b0000011: begin dat = 1; eop = 0; nsb = 1; bad = 0; end  // 12
      7'b0000110: begin dat = 1; eop = 0; nsb = 1; bad = 0; end  // 13
      7'b0001100: begin dat = 1; eop = 0; nsb = 1; bad = 0; end  // 14
      7'b0001001: begin dat = 1; eop = 0; nsb = 1; bad = 0; end  // 15
      7'b1100000: begin dat = 0; eop = 1; nsb = 1; bad = 0; end  // eop
      // incomplete (single-bit/no change)
      0, 1, 2, 4,	  	   	  	  
      8, 16, 32,                                     
      64:         begin dat = 0; eop = 0; nsb = 0; bad = 0; end
      // anything else is a bad symbol
      default:    begin dat = 0; eop = 0; nsb = 1; bad = 1; end
    endcase
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // expect an end-of-packet symbol (combinatorial)
  //---------------------------------------------------------------
  always @ (*)
    exp_eop = (!long_pkt && (symbol_cnt == 10)) || (symbol_cnt == 18);
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // error status
  //---------------------------------------------------------------
  always @(posedge clk or posedge rst)
    if (rst)
      err <= 1'b0;
    else
      if (state == ERR0_ST)
        err <= 1'b1;
      else
        err <= err;  // no change!
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // keep track of number of symbols received to check for frame errors
  //---------------------------------------------------------------
  always @(posedge clk or posedge rst)
    if (rst)
      err_cnt <= 5'd31;
    else
      case (state)
	ERR0_ST: if (err_cnt != 0)
		   err_cnt <= err_cnt - 1;
                 else
		   err_cnt <= 5'd31;

	default:   err_cnt <= 5'd31;    // no change!
      endcase 
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

	IDLE_ST: casex ({dat, bad})
                   2'b1x:   state <= TRAN_ST;  // first symbol in new packet
                   2'bx1:   state <= ERR0_ST;  // error, drop packet
                   default: state <= IDLE_ST;  // no change!
                 endcase

	TRAN_ST: casex ({bad, eop})
		   2'b1x:  // bad symbol: drop and go to error
                     state <= ERR0_ST;

	           2'bx1:  // eop: check if correct packet and forward
                     state <= EOP0_ST;

                   default:  // data or no symbol arrived: keep going
                     state <= TRAN_ST;
                 endcase

	EOP0_ST: casex ({exp_eop, busy})
		   2'b01:  // unexpected and busy: wait here
                     state <= EOP0_ST;

	           2'b11:  // expected eop and busy: wait for ready
                     state <= WTRD_ST;

		   //#2'b00,  // unexpected eop: drop
	           //#2'b10,  // expected eop: forward packet
                   default:  // data or no symbol arrived: keep going
                     state <= IDLE_ST;
                 endcase

        WTRD_ST:  if (!busy)
                   state <= IDLE_ST;  // forward packet and start again
                 else
                   state <= WTRD_ST;  // no change!

        ERR0_ST:  if (eop)
                   state <= IDLE_ST;
                 else
                   state <= ERR0_ST;  // no change!

        default: state <= state;  // no change!
      endcase
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

endmodule
