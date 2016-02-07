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

  // link error reporting
  output wire                   FLT_ERR_OUT,
  output wire                   FRM_ERR_OUT,

  // SpiNNaker link interface
  input                   [6:0] SL_DATA_2OF7_IN,
  output wire                   SL_ACK_OUT,

  // spiNNlink interface
  output wire [`PKT_BITS - 1:0] PKT_DATA_OUT,
  output wire                   PKT_VLD_OUT,
  input                         PKT_RDY_IN
);

  //-------------------------------------------------------------
  // internal signals
  //-------------------------------------------------------------
  wire [6:0] synced_sl_data_2of7;  // synchronized 2of7-encoded data input

  wire [6:0] flt_data_2of7;  // 2of7-encoded data -- no need for vld signal
  wire       flt_rdy;
  wire [4:0] flt_tot;        // number of flits in this packet


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
    .flt_rdy         (flt_rdy),
    .flt_tot         (flt_tot)
  );

  pkt_deserializer pd
  (
    .CLK_IN          (CLK_IN),
    .RESET_IN        (RESET_IN),
    .FLT_ERR_OUT     (FLT_ERROR_OUT),
    .FRM_ERR_OUT     (FRM_ERR_OUT),
    .flt_data_2of7   (flt_data_2of7),
    .flt_rdy         (flt_rdy),
    .flt_tot         (flt_tot),
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
  input                         flt_rdy,
  input                   [4:0] flt_tot
);

  //-------------------------------------------------------------
  // constants
  //-------------------------------------------------------------
  localparam STATE_BITS = 2;
  localparam STRT_ST    = 0;   // need to send an ack on reset exit!
  localparam IDLE_ST    = STRT_ST + 1;
  localparam RECV_ST    = IDLE_ST + 1;
  localparam EOPW_ST    = RECV_ST + 1;

  localparam INTER_ACK_DELAY = 1;


  //-------------------------------------------------------------
  // internal signals
  //-------------------------------------------------------------
  reg send_ack;  // send ack to SpiNNaker (toggle SL_ACK_OUT)

  reg new_flit;  // new flit arrived
  reg eop_flit;  // end-of-packet flit arrived
 
  reg [4:0] ack_cnt;   // number of acks already sent

  reg [6:0] rtz_data;  // data translated from nrz to rtz

  reg [2:0] dly_cnt;   // keep track of inter ack clock cycles

  reg [STATE_BITS - 1:0] state;  // current state


  //-------------------------------------------------------------
  // 2-of-7 symbol detector (correct data, eop or error)
  //-------------------------------------------------------------
  function detect_nrz_2of7 ;
    input [6:0] data;

    case (data)
      0, 1, 2, 4,
      8, 16, 32,
      64:         detect_nrz_2of7 = 0;  // incomplete (no/single-bit change)
      default:    detect_nrz_2of7 = 1;  // correct data, eop or error
    endcase
  endfunction


  //-------------------------------------------------------------
  // 2-of-7 end-of-packet detector
  //-------------------------------------------------------------
  function eop_2of7 ;
    input [6:0] data;

    case (data)
      7'b1100000: eop_2of7 = 1;
      default:    eop_2of7 = 0;  // anything else is not an end-of-packet
    endcase
  endfunction


  //-------------------------------------------------------------
  // SpiNNaker link interface: generate SL_ACK_OUT
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      SL_ACK_OUT <= 1'b0;
    else
      if (send_ack)
        SL_ACK_OUT <= ~SL_ACK_OUT;


  //-------------------------------------------------------------
  // send next value of SL_ACK_OUT (toggle SL_ACK_OUT)
  //-------------------------------------------------------------
  always @(*)
    case (state)
      STRT_ST:   send_ack = 1'b1;  // mimic SpiNNaker: ack on reset exit

      IDLE_ST: if (new_flit && flt_rdy)
                 send_ack = 1'b1;  //  ack new flit when ready
               else
                 send_ack = 1'b0;   //  no ack!

      RECV_ST: if ((dly_cnt == 0) && (ack_cnt < flt_tot))
                 send_ack = 1'b1;  //  ack new flit when ready
               else
                 send_ack = 1'b0;   //  no ack!

      EOPW_ST:   send_ack = 1'b0;   //  no ack!
    endcase 


  //---------------------------------------------------------------
  // translate data from nrz to rtz
  //---------------------------------------------------------------
  always @(*)
    rtz_data = SL_DATA_2OF7_IN ^ flt_data_2of7;


  //-------------------------------------------------------------
  // detect the arrival of a new nrz flit (2 or more transitions)
  //-------------------------------------------------------------
  always @(*)
    new_flit = detect_nrz_2of7 (rtz_data);


  //-------------------------------------------------------------
  // detect the arrival of an end-of-packet flit
  //-------------------------------------------------------------
  always @(*)
    eop_flit = eop_2of7 (rtz_data);


  //-------------------------------------------------------------
  // packet deserializer interface: generate flt_data_2of7
  //-------------------------------------------------------------
  always @(posedge CLK_IN)
    case (state)
      STRT_ST:
          flt_data_2of7 <= SL_DATA_2OF7_IN;  // send & remember initial data

      IDLE_ST:
        if (new_flit && flt_rdy)
          flt_data_2of7 <= SL_DATA_2OF7_IN;  // send & remember incoming data

      RECV_ST:
        if (new_flit)
          flt_data_2of7 <= SL_DATA_2OF7_IN;  // send & remember incoming data
    endcase 


  //-------------------------------------------------------------
  // keep track of inter ack clock cycles
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      dly_cnt <= INTER_ACK_DELAY;
    else
      case (state)
        IDLE_ST:   dly_cnt <= INTER_ACK_DELAY;

        RECV_ST: if (dly_cnt == 0)
                   dly_cnt <= INTER_ACK_DELAY;  // ack sent
	         else
                   dly_cnt <= dly_cnt - 1;      // wait before sending ack

        default:   dly_cnt <= INTER_ACK_DELAY;
      endcase 


  //-------------------------------------------------------------
  // keep track of number of acks already sent
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      ack_cnt <= 0;
    else
      case (state)
        STRT_ST,
        EOPW_ST:   ack_cnt <= 0;

        default: if (send_ack)
                   ack_cnt <= ack_cnt + 1;
      endcase 


  //-------------------------------------------------------------
  // state machine
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      state <= STRT_ST;  // need to send an ack on reset exit!
    else
      case (state)
        STRT_ST:   state <= IDLE_ST;

        IDLE_ST: if (new_flit && flt_rdy)
                   state <= RECV_ST;

        RECV_ST: if (eop_flit)
                   state <= EOPW_ST;

        EOPW_ST:   state <= IDLE_ST;
      endcase 
endmodule
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`timescale 1ns / 1ps
module pkt_deserializer
(
  input                         CLK_IN,
  input                         RESET_IN,

  // link error reporting
  output reg                    FLT_ERR_OUT,
  output reg                    FRM_ERR_OUT,

  // flit interface
  input                   [6:0] flt_data_2of7,
  output reg                    flt_rdy,
  output reg              [4:0] flt_tot,

  // spiNNlink interface
  output reg  [`PKT_BITS - 1:0] PKT_DATA_OUT,
  output reg                    PKT_VLD_OUT,
  input                         PKT_RDY_IN
);

  //---------------------------------------------------------------
  // constants
  //---------------------------------------------------------------
  //# Xilinx recommends one-hot state encoding
  localparam STATE_BITS = 3;
  localparam STRT_ST    = 0;
  localparam IDLE_ST    = STRT_ST + 1;
  localparam TRAN_ST    = IDLE_ST + 1;
  localparam WAIT_ST    = TRAN_ST + 1;
  localparam FERR_ST    = WAIT_ST + 1;


  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  reg [6:0] old_data;  // remember previous nrz 2of7 data for rtz translation

  reg       new_flit;  // new flit arrived
  reg       dat_flit;  // is the new flit correct data?
  reg       bad_flit;  // is the new flit an error?
  reg       eop_flit;  // is the new flit an end-of-packet?
  reg       exp_eop;   // is the new flit expected to be an eop?
  reg [4:0] flit_cnt;  // keep track of number of received flits

  reg [3:0] new_data;  // new data flit decoded

  reg       long_pkt;  // remember length of packet

  reg   [`PKT_BITS - 1:0] pkt_buf;   // buffer used to assemble pkt

  reg                     pkt_busy;  // pkt interface busy

  reg                     send_pkt;  // send pkt out

  reg  [STATE_BITS - 1:0] state;     // current state


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
  // NRZ 2-of-7 symbol detector (correct data, eop or error)
  //-------------------------------------------------------------
  function detect_nrz_2of7 ;
    input [6:0] data;
    input [6:0] old_data;

    case (data ^ old_data)
      0, 1, 2, 4,
      8, 16, 32,
      64:         detect_nrz_2of7 = 0;  // incomplete (no/single-bit change)
      default:    detect_nrz_2of7 = 1;  // correct data, eop or error
    endcase
  endfunction


  //-------------------------------------------------------------
  // NRZ 2-of-7 data detector
  //-------------------------------------------------------------
  function data_nrz_2of7 ;
    input [6:0] data;
    input [6:0] old_data;

    case (data ^ old_data)
      7'b0010001: data_nrz_2of7 = 1;  // 0
      7'b0010010: data_nrz_2of7 = 1;  // 1
      7'b0010100: data_nrz_2of7 = 1;  // 2
      7'b0011000: data_nrz_2of7 = 1;  // 3
      7'b0100001: data_nrz_2of7 = 1;  // 4
      7'b0100010: data_nrz_2of7 = 1;  // 5
      7'b0100100: data_nrz_2of7 = 1;  // 6
      7'b0101000: data_nrz_2of7 = 1;  // 7
      7'b1000001: data_nrz_2of7 = 1;  // 8
      7'b1000010: data_nrz_2of7 = 1;  // 9
      7'b1000100: data_nrz_2of7 = 1;  // 10
      7'b1001000: data_nrz_2of7 = 1;  // 11
      7'b0000011: data_nrz_2of7 = 1;  // 12
      7'b0000110: data_nrz_2of7 = 1;  // 13
      7'b0001100: data_nrz_2of7 = 1;  // 14
      7'b0001001: data_nrz_2of7 = 1;  // 15
      default:    data_nrz_2of7 = 0;  // anything else is not correct data
    endcase
  endfunction


  //-------------------------------------------------------------
  // NRZ 2-of-7 end-of-packet detector
  //-------------------------------------------------------------
  function eop_nrz_2of7 ;
    input [6:0] data;
    input [6:0] old_data;

    case (data ^ old_data)
      7'b1100000: eop_nrz_2of7 = 1;
      default:    eop_nrz_2of7 = 0;  // anything else is not an end-of-packet
    endcase
  endfunction


  //-------------------------------------------------------------
  // NRZ 2-of-7 error detector
  //-------------------------------------------------------------
  function error_nrz_2of7 ;
    input [6:0] data;
    input [6:0] old_data;

    case (data ^ old_data)
      7'b0010001: error_nrz_2of7 = 0;  // 0
      7'b0010010: error_nrz_2of7 = 0;  // 1
      7'b0010100: error_nrz_2of7 = 0;  // 2
      7'b0011000: error_nrz_2of7 = 0;  // 3
      7'b0100001: error_nrz_2of7 = 0;  // 4
      7'b0100010: error_nrz_2of7 = 0;  // 5
      7'b0100100: error_nrz_2of7 = 0;  // 6
      7'b0101000: error_nrz_2of7 = 0;  // 7
      7'b1000001: error_nrz_2of7 = 0;  // 8
      7'b1000010: error_nrz_2of7 = 0;  // 9
      7'b1000100: error_nrz_2of7 = 0;  // 10
      7'b1001000: error_nrz_2of7 = 0;  // 11
      7'b0000011: error_nrz_2of7 = 0;  // 12
      7'b0000110: error_nrz_2of7 = 0;  // 13
      7'b0001100: error_nrz_2of7 = 0;  // 14
      7'b0001001: error_nrz_2of7 = 0;  // 15
      7'b1100000: error_nrz_2of7 = 0;  // eop
      0, 1, 2, 4,
      8, 16, 32,
      64:         error_nrz_2of7 = 0;  // incomplete (single-bit change)
      default:    error_nrz_2of7 = 1;  // anything else is an error
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
	TRAN_ST: if (eop_flit && exp_eop && pkt_busy)
                   flt_rdy <= 1'b0;  // waiting for pkt interface!

	WAIT_ST: if (!pkt_busy)
                   flt_rdy <= 1'b1;

        default:   flt_rdy <= 1'b1;
      endcase 

  always @(posedge CLK_IN)
    if ((state == IDLE_ST) && dat_flit)
      if (new_data[1])
	flt_tot <= 19;
      else
	flt_tot <= 11;


  //-------------------------------------------------------------
  // remember previous nrz 2of7 data for translation to rtz
  //-------------------------------------------------------------
  always @(posedge CLK_IN)
    case (state)
      STRT_ST:   old_data <= flt_data_2of7;  // remember initial 2of7 data

      default: if (new_flit && flt_rdy)
                 old_data <= flt_data_2of7;  // remember new 2of7 data
    endcase


  //---------------------------------------------------------------
  // detect the arrival of a new nrz flit (2 or more transitions)
  //---------------------------------------------------------------
  always @(*)
    new_flit = detect_nrz_2of7 (flt_data_2of7, old_data);
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // is the new nrz flit correct data?
  //---------------------------------------------------------------
  always @(*)
    dat_flit = data_nrz_2of7 (flt_data_2of7, old_data);


  //---------------------------------------------------------------
  // is the new nrz flit an end-of-packet?
  //---------------------------------------------------------------
  always @(*)
    eop_flit = eop_nrz_2of7 (flt_data_2of7, old_data);


  //---------------------------------------------------------------
  // is the new nrz flit an error?
  //---------------------------------------------------------------
  always @(*)
    bad_flit = error_nrz_2of7 (flt_data_2of7, old_data);


  //---------------------------------------------------------------
  // is the new nrz flit expected to be an eop?
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


  //-------------------------------------------------------------
  // packet interface: generate PKT_DATA_OUT
  //-------------------------------------------------------------
  always @(posedge CLK_IN)
    if (send_pkt && !pkt_busy)
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
        if (send_pkt)
          PKT_VLD_OUT <= 1'b1;
        else
          PKT_VLD_OUT <= 1'b0;


  //-------------------------------------------------------------
  // packet interface busy
  //-------------------------------------------------------------
  always @(*)
    pkt_busy = PKT_VLD_OUT && !PKT_RDY_IN;


  //---------------------------------------------------------------
  // decode new data flit
  //---------------------------------------------------------------
  always @(*)
    new_data = decode_nrz_2of7 (flt_data_2of7, old_data);


  //-------------------------------------------------------------
  // shift in new data to assemble packet
  //-------------------------------------------------------------
  always @(posedge CLK_IN)
    if (dat_flit && flt_rdy)
      pkt_buf <= {new_data, pkt_buf[`PKT_BITS - 1:4]};


  //-------------------------------------------------------------
  // time to send packet
  //-------------------------------------------------------------
  always @(*)
    case (state)
      TRAN_ST: if (eop_flit && exp_eop)
                 send_pkt = 1;  // try to send newly assembled pkt
               else
                 send_pkt = 0;  // no pkt ready to send

      WAIT_ST:   send_pkt = 1;  // waiting and trying to send pkt

      default:   send_pkt = 0;  // no pkt ready to send
    endcase 


  //-------------------------------------------------------------
  // remember length of packet
  //-------------------------------------------------------------
  always @(posedge CLK_IN)
    if ((state == IDLE_ST) && dat_flit)
      long_pkt <= new_data[1];


  //---------------------------------------------------------------
  // error reporting
  //---------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      FLT_ERR_OUT <= 1'b0;
    else
      case (state)
        IDLE_ST,
        TRAN_ST:
          if (bad_flit)
            FLT_ERR_OUT <= 1'b1;
          else
            FLT_ERR_OUT <= 1'b0;

        default: FLT_ERR_OUT <= 0;
      endcase

  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      FRM_ERR_OUT <= 1'b0;
    else
      case (state)
        IDLE_ST,
        TRAN_ST:
          if (eop_flit && !exp_eop)
            FRM_ERR_OUT <= 1'b1;
          else
            FRM_ERR_OUT <= 1'b0;

        default: FRM_ERR_OUT <= 0;
      endcase
  //---------------------------------------------------------------


  //-------------------------------------------------------------
  // state machine
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      state <= STRT_ST;
    else
      case (state)
	STRT_ST: state <= IDLE_ST;

	IDLE_ST: casex ({dat_flit, bad_flit})
                   2'b1x:   state <= TRAN_ST;  // first flit in new packet
                   2'bx1:   state <= FERR_ST;  // error, drop packet
                   default: state <= IDLE_ST;  // stay idle
                 endcase

	TRAN_ST: casex ({bad_flit, eop_flit, exp_eop, pkt_busy})
		   4'b1xxx:  // error flit: drop packet
                     state <= FERR_ST;

	           4'bx10x,  // unexpected eop: drop packet and go idle
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

	default: state <= FERR_ST;  // should never happen!
      endcase
endmodule
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
