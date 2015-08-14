// -------------------------------------------------------------------------
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
`timescale 1ns / 1ps
module spio_spinnaker_link_receiver
(
  input                         CLK_IN,
  input                         RESET_IN,

  // SpiNNaker link interface
  input                   [6:0] SL_DATA_2OF7_IN,
  (* IOB = "FORCE" *)
  output wire                   SL_ACK_OUT,

  // spiNNlink interface
  output wire [`PKT_BITS - 1:0] PKT_DATA_OUT,
  output wire                   PKT_VLD_OUT,
  input                         PKT_RDY_IN
);

  //-------------------------------------------------------------
  // constants
  //-------------------------------------------------------------


  //-------------------------------------------------------------
  // internal signals
  //-------------------------------------------------------------
  wire [6:0] synced_sl_data_2of7;  // synchronized 2of7-encoded data input

  wire [6:0] flt_data_2of7;  // 2of7-encoded data -- no need for vld signal
  wire       flt_rdy;


  spio_spinnaker_link_sync #(.SIZE(7)) sync
  ( .CLK_IN (CLK_IN),
    .IN     (SL_DATA_2OF7_IN),
    .OUT    (synced_sl_data_2of7)
  );
		
  flit_input_if fi
  (
    .CLK_IN          (CLK_IN),
    .RESET_IN        (RESET_IN),
    .SL_DATA_2OF7_IN (synced_sl_data_2of7),
    .SL_ACK_OUT      (SL_ACK_OUT),
    .flt_data_2of7   (flt_data_2of7),
    .flt_rdy         (flt_rdy)
  );

  pkt_deserializer pd
  (
    .CLK_IN          (CLK_IN),
    .RESET_IN        (RESET_IN),
    .flt_data_2of7   (flt_data_2of7),
    .flt_rdy         (flt_rdy),
    .PKT_DATA_OUT    (PKT_DATA_OUT),
    .PKT_VLD_OUT     (PKT_VLD_OUT),
    .PKT_RDY_IN      (PKT_RDY_IN)
  );
endmodule
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`timescale 1ns / 1ps
module flit_input_if
(
  input                         CLK_IN,
  input                         RESET_IN,

  // SpiNNaker link interface
  input                   [6:0] SL_DATA_2OF7_IN,
  output reg                    SL_ACK_OUT,

  // packet deserializer interface
  output reg              [6:0] flt_data_2of7,
  input                         flt_rdy
);

  //-------------------------------------------------------------
  // constants
  //-------------------------------------------------------------
  localparam STATE_BITS = 1;
  localparam STRT_ST    = 0;
  localparam IDLE_ST    = STRT_ST + 1;


  //-------------------------------------------------------------
  // internal signals
  //-------------------------------------------------------------
  reg flt_vld_i;  // keep track of flit data validity
  reg flt_busy;   // flit interface busy

  reg new_flit;  // new flit arrived

  (* KEEP = "TRUE" *)
  reg ack_int;   // internal copy of SL_ACK_OUT

  reg next_ack;  // next value of SL_ACK_OUT

  (* KEEP = "TRUE" *)
  reg [STATE_BITS - 1:0] state;  // current state


  //-------------------------------------------------------------
  // SpiNNaker link interface: generate SL_ACK_OUT
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      SL_ACK_OUT <= 1'b0;
    else
      SL_ACK_OUT <= next_ack;


  //-------------------------------------------------------------
  // internal copy of SL_ACK_OUT
  // NOTE: allows the SL_ACK_OUT FF to be packed with the I/O block
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      ack_int <= 1'b0;
    else
      ack_int <= next_ack;


  //-------------------------------------------------------------
  // next value of SL_ACK_OUT
  //-------------------------------------------------------------
  always @(*)
    case (state)
      STRT_ST:   next_ack = 1'b1;      // mimic SpiNNaker: ack on reset

      default: if (new_flit && !flt_busy)
                 next_ack = ~ack_int;  //  ack new flit when ready
               else
                 next_ack = ack_int;   //  no ack!
    endcase 


  //-------------------------------------------------------------
  // packet deserializer interface: generate flt_data_2of7
  //-------------------------------------------------------------
  always @(posedge CLK_IN)
    case (state)
      STRT_ST:
          flt_data_2of7 <= SL_DATA_2OF7_IN;  // remember initial data

      default:
        if (new_flit && !flt_busy)
          flt_data_2of7 <= SL_DATA_2OF7_IN;  // remember incoming data
    endcase 


  //-------------------------------------------------------------
  // detect the arrival of a new flit (2 or more transitions)
  //-------------------------------------------------------------
  always @(*)
    case (SL_DATA_2OF7_IN ^ flt_data_2of7)
      0, 1, 2, 4,
      8, 16, 32,
      64:         new_flit = 0;  // incomplete (no/single-bit change)
      default:    new_flit = 1;  // correct data, eop or error
    endcase


  //-------------------------------------------------------------
  // keep track of flit data validity
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      flt_vld_i = 1'b0;
    else
      if (!flt_busy)
        if (new_flit)
          flt_vld_i = 1'b1;
        else
          flt_vld_i = 1'b0;

  //-------------------------------------------------------------
  // flit interface busy
  //-------------------------------------------------------------
  always @(*)
    flt_busy = flt_vld_i && !flt_rdy;


  //-------------------------------------------------------------
  // state machine
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      state <= STRT_ST;  // need to send an ack on reset exit!
    else
      state <= IDLE_ST;
endmodule
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`timescale 1ns / 1ps
module pkt_deserializer
(
  input                         CLK_IN,
  input                         RESET_IN,

  // flit interface
  input                   [6:0] flt_data_2of7,
  output reg                    flt_rdy,

  // spiNNlink interface
  output reg  [`PKT_BITS - 1:0] PKT_DATA_OUT,
  output reg                    PKT_VLD_OUT,
  input                         PKT_RDY_IN
);

  //---------------------------------------------------------------
  // constants
  //---------------------------------------------------------------
  //# Xilinx recommends one-hot state encoding
  localparam STATE_BITS = 2;
  localparam STRT_ST    = 0;
  localparam IDLE_ST    = STRT_ST + 1;
  localparam TRAN_ST    = IDLE_ST + 1;
  localparam WAIT_ST    = TRAN_ST + 1;
  localparam FERR_ST    = WAIT_ST + 1;


  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  reg [6:0] old_data;  // remember previous 2of7 data for decoding

  reg       new_flit;  // new flit arrived
  reg       dat_flit;  // new flit is correct data
  reg       bad_flit;  // new flit is an error
  reg       eop_flit;  // new flit is an end-of-packet
  reg       exp_eop;   // is the new flit expected to be an eop?
  reg [4:0] flit_cnt;  // keep track of number of received flits

  reg [3:0] new_data;  // new data flit decoded

  reg       long_pkt;  // remember length of packet

  reg   [`PKT_BITS - 1:0] pkt_buf;   // buffer used to assemble pkt

  reg                     pkt_busy;  // pkt interface busy
  reg                     pkt_wait;  // wait for free pkt interface
  reg                     pkt_send;  // send pkt out

  reg  [STATE_BITS - 1:0] state;     // current state


  //-------------------------------------------------------------
  // functions
  //-------------------------------------------------------------

  //-------------------------------------------------------------
  // NRZ 2-of-7 decoder
  //-------------------------------------------------------------
  function [3:0] decode_nrz_2of7 ;
    input [6:0] data;
    input [6:0] old_data;

    case (data ^ old_data)
      7'b0010001: decode_nrz_2of7 = 0;    // 0
      7'b0010010: decode_nrz_2of7 = 1;    // 1
      7'b0010100: decode_nrz_2of7 = 2;    // 2
      7'b0011000: decode_nrz_2of7 = 3;    // 3
      7'b0100001: decode_nrz_2of7 = 4;    // 4
      7'b0100010: decode_nrz_2of7 = 5;    // 5
      7'b0100100: decode_nrz_2of7 = 6;    // 6
      7'b0101000: decode_nrz_2of7 = 7;    // 7
      7'b1000001: decode_nrz_2of7 = 8;    // 8
      7'b1000010: decode_nrz_2of7 = 9;    // 9
      7'b1000100: decode_nrz_2of7 = 10;   // 10
      7'b1001000: decode_nrz_2of7 = 11;   // 11
      7'b0000011: decode_nrz_2of7 = 12;   // 12
      7'b0000110: decode_nrz_2of7 = 13;   // 13
      7'b0001100: decode_nrz_2of7 = 14;   // 14
      7'b0001001: decode_nrz_2of7 = 15;   // 15
      default:    decode_nrz_2of7 = 4'hx; // eop, incomplete, bad
    endcase
  endfunction


  //-------------------------------------------------------------
  // flit interface: generate flt_rdy
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      flt_rdy <= 1'b0;
    else
      case (state)
	TRAN_ST: if (pkt_wait)
                   flt_rdy <= 1'b0;

	WAIT_ST: if (!pkt_busy)
                   flt_rdy <= 1'b1;

        default:   flt_rdy <= 1'b1;
      endcase 



  //-------------------------------------------------------------
  // remember previous 2of7 data for decoding
  //-------------------------------------------------------------
  always @(posedge CLK_IN)
    case (state)
      STRT_ST:   old_data <= flt_data_2of7;  // remember initial 2of7 data

      default: if (new_flit && flt_rdy)
                 old_data <= flt_data_2of7;  // remember new 2of7 data
    endcase


  //---------------------------------------------------------------
  // detect the arrival of a new flit (2 or more transitions)
  //---------------------------------------------------------------
  always @(*)
    case (flt_data_2of7 ^ old_data)
      0, 1, 2, 4,
      8, 16, 32,
      64:         new_flit = 0;  // incomplete (single-bit change)
      default:    new_flit = 1;  // correct data, eop or bad 
    endcase
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // new flit is correct data
  //---------------------------------------------------------------
  always @(*)
    case (flt_data_2of7 ^ old_data)
      7'b0010001: dat_flit = 1;  // 0
      7'b0010010: dat_flit = 1;  // 1
      7'b0010100: dat_flit = 1;  // 2
      7'b0011000: dat_flit = 1;  // 3
      7'b0100001: dat_flit = 1;  // 4
      7'b0100010: dat_flit = 1;  // 5
      7'b0100100: dat_flit = 1;  // 6
      7'b0101000: dat_flit = 1;  // 7
      7'b1000001: dat_flit = 1;  // 8
      7'b1000010: dat_flit = 1;  // 9
      7'b1000100: dat_flit = 1;  // 10
      7'b1001000: dat_flit = 1;  // 11
      7'b0000011: dat_flit = 1;  // 12
      7'b0000110: dat_flit = 1;  // 13
      7'b0001100: dat_flit = 1;  // 14
      7'b0001001: dat_flit = 1;  // 15
      default:    dat_flit = 0;  // anything else is not correct data
    endcase


  //---------------------------------------------------------------
  // new flit is an error
  //---------------------------------------------------------------
  always @(*)
    case (flt_data_2of7 ^ old_data)
      7'b0010001: bad_flit = 0;  // 0
      7'b0010010: bad_flit = 0;  // 1
      7'b0010100: bad_flit = 0;  // 2
      7'b0011000: bad_flit = 0;  // 3
      7'b0100001: bad_flit = 0;  // 4
      7'b0100010: bad_flit = 0;  // 5
      7'b0100100: bad_flit = 0;  // 6
      7'b0101000: bad_flit = 0;  // 7
      7'b1000001: bad_flit = 0;  // 8
      7'b1000010: bad_flit = 0;  // 9
      7'b1000100: bad_flit = 0;  // 10
      7'b1001000: bad_flit = 0;  // 11
      7'b0000011: bad_flit = 0;  // 12
      7'b0000110: bad_flit = 0;  // 13
      7'b0001100: bad_flit = 0;  // 14
      7'b0001001: bad_flit = 0;  // 15
      7'b1100000: bad_flit = 0;  // eop
      0, 1, 2, 4,
      8, 16, 32,
      64:         bad_flit = 0;  // incomplete (single-bit change)
      default:    bad_flit = 1;  // anything else is an error
    endcase


  //---------------------------------------------------------------
  // new flit is an end-of-packet
  //---------------------------------------------------------------
  always @(*)
    case (flt_data_2of7 ^ old_data)
      7'b1100000: eop_flit = 1;
      default:    eop_flit = 0;
    endcase


  //---------------------------------------------------------------
  // is the new flit expected to be an eop?
  //---------------------------------------------------------------
  always @ (*)
    exp_eop = (!long_pkt && (flit_cnt == 10)) || (flit_cnt == 18);
  //---------------------------------------------------------------


  //-------------------------------------------------------------
  // keep track of how many flits have been received
  //-------------------------------------------------------------
  always @(posedge CLK_IN)
    case (state)
      TRAN_ST: if (dat_flit)
                 flit_cnt <= flit_cnt + 1;  // count new flit

      default:   flit_cnt <= 5'd1;          // init count
    endcase 


  //---------------------------------------------------------------
  // decode new flit
  //---------------------------------------------------------------
  always @(*)
    new_data = decode_nrz_2of7 (flt_data_2of7, old_data);


  //-------------------------------------------------------------
  // packet interface: generate PKT_DATA_OUT
  //-------------------------------------------------------------
  always @(posedge CLK_IN)
    if (pkt_send && !pkt_busy)
      if (long_pkt)
        PKT_DATA_OUT <= pkt_buf;
      else
        PKT_DATA_OUT <= pkt_buf >> 32;  // no payload: complete buffer shift!


  //-------------------------------------------------------------
  // packet interface: generate PKT_VLD_OUT
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      PKT_VLD_OUT <= 1'b0;
    else
      if (!pkt_busy)
        PKT_VLD_OUT <= pkt_send;


  //-------------------------------------------------------------
  // shift in new data to assemble packet
  //-------------------------------------------------------------
  always @(posedge CLK_IN)
    if (dat_flit && flt_rdy)
      pkt_buf <= {new_data, pkt_buf[`PKT_BITS - 1:4]};


  //-------------------------------------------------------------
  // packet interface busy
  //-------------------------------------------------------------
  always @(*)
    pkt_busy = PKT_VLD_OUT && !PKT_RDY_IN;


  //-------------------------------------------------------------
  // must wait for packet interface to free
  //-------------------------------------------------------------
  always @(*)
    pkt_wait = eop_flit && exp_eop && pkt_busy;


  //-------------------------------------------------------------
  // time to send packet
  //-------------------------------------------------------------
  always @(*)
    case (state)
      TRAN_ST: if (eop_flit && exp_eop)
                 pkt_send = 1;  // try to send newly assembled pkt
               else
                 pkt_send = 0;  // no pkt ready to send

      WAIT_ST:   pkt_send = 1;  // waiting trying to send ready pkt

      default:   pkt_send = 0;  // no pkt ready to send
    endcase 


  //-------------------------------------------------------------
  // remember length of packet
  //-------------------------------------------------------------
  always @(posedge CLK_IN)
    if ((state == IDLE_ST) && dat_flit)
      long_pkt <= new_data[1];


  //-------------------------------------------------------------
  // state machine
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      state <= STRT_ST;
    else
      case (state)
	IDLE_ST: casex ({dat_flit, bad_flit})
                   2'b1x:   state <= TRAN_ST;  // first flit in new packet
                   2'bx1:   state <= FERR_ST;  // error, drop packet
                   default: state <= IDLE_ST;  // keep idle
                 endcase

	TRAN_ST: casex ({bad_flit, eop_flit, exp_eop, pkt_busy})
		   4'b1xxx:  // error flit: drop packet
                     state <= FERR_ST;

	           4'bx10x,  // unexpected eop: drop packet
	           4'bx110:  // expected eop and not busy: send and go idle 
                     state <= IDLE_ST;

		   4'bx111:  // expected eop but busy: wait
                     state <= WAIT_ST;

                   default:   // data or no flit: stay in transfer state
                     state <= TRAN_ST;
                 endcase

        WAIT_ST: if (!pkt_busy)
                   state <= IDLE_ST;  // wait for pkt interface free

        FERR_ST: if (eop_flit)
                   state <= IDLE_ST;  // wait for eop to exit error state

	default: state <= IDLE_ST;
      endcase
endmodule
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
