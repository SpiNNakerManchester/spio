// -------------------------------------------------------------------------
//  SpiNNaker link transmitter module
//
// -------------------------------------------------------------------------
// AUTHOR
//  lap - luis.plana@manchester.ac.uk
//  Based on work by J Pepper (Date 08/08/2012)
//
// -------------------------------------------------------------------------
// Taken from:
// https://solem.cs.man.ac.uk/svn/spiNNlink/testing/src/packet_sender.v
// Revision 2517 (Last-modified date: 2013-08-19 10:33:30 +0100)
//
// -------------------------------------------------------------------------
// COPYRIGHT
//  Copyright (c) The University of Manchester, 2012. All rights reserved.
//  SpiNNaker Project
//  Advanced Processor Technologies Group
//  School of Computer Science
// -------------------------------------------------------------------------
// TODO
//  * decide what to do with "default" cases: non-valid states.
//  * could save one clock cycle by adding RDY_ST (not straightforward!).
// -------------------------------------------------------------------------


// ----------------------------------------------------------------
// include spiNNlink global constants and parameters
//
`include "spio_spinnaker_link.h"
// ----------------------------------------------------------------


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`timescale 1ns / 1ps
module spio_spinnaker_link_sender
(
  input                        CLK_IN,
  input                        RESET_IN,

  // synchronous packet interface
  input      [`PKT_BITS - 1:0] PKT_DATA_IN,
  input                        PKT_VLD_IN,
  output                       PKT_RDY_OUT,

  // SpiNNaker link asynchronous interface
  (* IOB = "FORCE" *)
  output                 [6:0] SL_DATA_2OF7_OUT,
  input                        SL_ACK_IN
);

  //-------------------------------------------------------------
  // constants
  //-------------------------------------------------------------


  //-------------------------------------------------------------
  // internal signals
  //-------------------------------------------------------------
  wire       synced_sl_ack;  // synchronized acknowledge input

  wire [6:0] flt_data;
  wire       flt_vld;
  wire       flt_rdy;


  spio_spinnaker_link_sync #(.SIZE(1)) sync
  ( .CLK_IN (CLK_IN),
    .IN     (SL_ACK_IN),
    .OUT    (synced_sl_ack)
  );
		
  pkt_serializer ps
  (
    .CLK_IN           (CLK_IN),
    .RESET_IN         (RESET_IN),
    .PKT_DATA_IN      (PKT_DATA_IN),
    .PKT_VLD_IN       (PKT_VLD_IN),
    .PKT_RDY_OUT      (PKT_RDY_OUT),
    .flt_data         (flt_data),
    .flt_vld          (flt_vld),
    .flt_rdy          (flt_rdy)
  );

  flit_output_if fo
  (
    .CLK_IN           (CLK_IN),
    .RESET_IN         (RESET_IN),
    .flt_data         (flt_data),
    .flt_vld          (flt_vld),
    .flt_rdy          (flt_rdy),
    .SL_DATA_2OF7_OUT (SL_DATA_2OF7_OUT),
    .SL_ACK_IN        (synced_sl_ack)
  );
endmodule
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`timescale 1ns / 1ps
module pkt_serializer
(
  input                         CLK_IN,
  input                         RESET_IN,

  // packet interface
  input       [`PKT_BITS - 1:0] PKT_DATA_IN,
  input                         PKT_VLD_IN,
  output reg                    PKT_RDY_OUT,

  // flit interface
  output reg              [6:0] flt_data,
  output reg                    flt_vld,
  input                         flt_rdy
);

  //---------------------------------------------------------------
  // constants
  //---------------------------------------------------------------
  //# Xilinx recommends one-hot state encoding
  localparam STATE_BITS = 2;
  localparam IDLE_ST    = 0;
  localparam TRAN_ST    = IDLE_ST + 1;
  localparam PARK_ST    = TRAN_ST + 1;


  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  reg   [`PKT_BITS - 1:0] pkt_buf;   // buffer incoming pkt
  reg                     long_pkt;  // remember pkt length

  reg                     flt_busy;  // flit interface busy
  reg               [4:0] flt_cnt;   // count sent flits

  reg                     eop;       // time to send end-of-packet
  reg  [STATE_BITS - 1:0] state;     // current state


  //-------------------------------------------------------------
  // functions
  //-------------------------------------------------------------

  //-------------------------------------------------------------
  // NRZ 2-of-7 encoder
  //-------------------------------------------------------------
  function [6:0] encode_nrz_2of7 ;
    input [4:0] din;
    input [6:0] old_din;

    casex (din)
      5'b00000 : encode_nrz_2of7 = old_din ^ 7'b0010001; // 0
      5'b00001 : encode_nrz_2of7 = old_din ^ 7'b0010010; // 1
      5'b00010 : encode_nrz_2of7 = old_din ^ 7'b0010100; // 2
      5'b00011 : encode_nrz_2of7 = old_din ^ 7'b0011000; // 3
      5'b00100 : encode_nrz_2of7 = old_din ^ 7'b0100001; // 4
      5'b00101 : encode_nrz_2of7 = old_din ^ 7'b0100010; // 5
      5'b00110 : encode_nrz_2of7 = old_din ^ 7'b0100100; // 6
      5'b00111 : encode_nrz_2of7 = old_din ^ 7'b0101000; // 7
      5'b01000 : encode_nrz_2of7 = old_din ^ 7'b1000001; // 8
      5'b01001 : encode_nrz_2of7 = old_din ^ 7'b1000010; // 9
      5'b01010 : encode_nrz_2of7 = old_din ^ 7'b1000100; // 10
      5'b01011 : encode_nrz_2of7 = old_din ^ 7'b1001000; // 11
      5'b01100 : encode_nrz_2of7 = old_din ^ 7'b0000011; // 12
      5'b01101 : encode_nrz_2of7 = old_din ^ 7'b0000110; // 13
      5'b01110 : encode_nrz_2of7 = old_din ^ 7'b0001100; // 14
      5'b01111 : encode_nrz_2of7 = old_din ^ 7'b0001001; // 15
      5'b1xxxx : encode_nrz_2of7 = old_din ^ 7'b1100000; // EOP
      default  : encode_nrz_2of7 = 7'bxxxxxxx;
    endcase
  endfunction


  //-------------------------------------------------------------
  // packet interface: generate PKT_RDY_OUT
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      PKT_RDY_OUT <= 1'b1;
    else
      case (state)
        IDLE_ST: if (PKT_VLD_IN)
                   PKT_RDY_OUT <= 1'b0;  // starting, not ready for next pkt
                 else
                   PKT_RDY_OUT <= 1'b1;  // waiting for next pkt

        TRAN_ST: if (eop && !flt_busy)
                   PKT_RDY_OUT <= 1'b1;  // finished, ready for next pkt
                 else
                   PKT_RDY_OUT <= 1'b0;  // not ready yet for next pkt

        default:   PKT_RDY_OUT <= 1'b0;  // not ready yet for next pkt
      endcase 


  //-------------------------------------------------------------
  // buffer packet (or part of it) to serialize it
  //-------------------------------------------------------------
  always @(posedge CLK_IN)
    case (state)
      IDLE_ST:
        case ({PKT_VLD_IN, flt_busy})
          2'b10:   pkt_buf <= PKT_DATA_IN >> 4;  // first nibble gone
          2'b11:   pkt_buf <= PKT_DATA_IN;       // park new packet
          default: pkt_buf <= pkt_buf;           // no change!
        endcase

      default: if (!flt_busy)
                   pkt_buf <= pkt_buf >> 4;      // prepare for next nibble
    endcase 


  //-------------------------------------------------------------
  // remember length of packet
  //-------------------------------------------------------------
  always @(posedge CLK_IN)
    if ((state == IDLE_ST) && PKT_VLD_IN)
      long_pkt <= PKT_DATA_IN[1];


  //-------------------------------------------------------------
  // flit interface: generate flt_data
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      flt_data <= 7'd0;
    else
      case (state)
        IDLE_ST: if (PKT_VLD_IN && !flt_busy)
                   flt_data <= encode_nrz_2of7 ({1'b0, PKT_DATA_IN[3:0]},
                                                 flt_data
                                               );  // first nibble

        default: if (!flt_busy)
                   flt_data <= encode_nrz_2of7 ({eop, pkt_buf[3:0]},
                                                 flt_data
                                               );  // next nibble or eop
	                                           // first if parked
      endcase 


  //-------------------------------------------------------------
  // flit interface: generate flt_vld
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      flt_vld <= 1'b0;
    else
      case (state)
        IDLE_ST: if (!PKT_VLD_IN && !flt_busy)
                   flt_vld <= 1'b0;  // no new flit to send
                 else
                   flt_vld <= 1'b1;  // last one still busy or first flit

        default:   flt_vld <= 1'b1;  // next flit always available
      endcase 


  //-------------------------------------------------------------
  // flit interface busy
  //-------------------------------------------------------------
  always @ (*)
    flt_busy = flt_vld && !flt_rdy;


  //-------------------------------------------------------------
  // keep track of how many flits have been sent
  //-------------------------------------------------------------
  always @(posedge CLK_IN)
    case (state)
      TRAN_ST: if (!flt_busy)
                 flt_cnt <= flt_cnt + 1;  // one more flit gone

      default:   flt_cnt <= 1;
    endcase 


  //-------------------------------------------------------------
  // time to send end-of-packet
  //-------------------------------------------------------------
  always @ (*)
    eop = (!long_pkt && (flt_cnt == 10)) || (flt_cnt == 18);


  //-------------------------------------------------------------
  // state machine
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      state <= IDLE_ST;
    else
      case (state)
        IDLE_ST:
          case ({PKT_VLD_IN, flt_busy})
            2'b10:   state <= TRAN_ST;  // start new packet
	    2'b11:   state <= PARK_ST;  // park new packet
            default: state <= IDLE_ST;  // wait for new packet
          endcase

        PARK_ST: if (!flt_busy)
                     state <= TRAN_ST;   

        default: if (eop && !flt_busy)
                     state <= IDLE_ST;  // done with packet
      endcase 
endmodule
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`timescale 1ns / 1ps
module flit_output_if
(
  input            CLK_IN,
  input            RESET_IN,

  // packet serializer interface
  input      [6:0] flt_data,
  input            flt_vld,
  output reg       flt_rdy,

  // SpiNNaker link interface
  output reg [6:0] SL_DATA_2OF7_OUT,
  input            SL_ACK_IN
);

  //-------------------------------------------------------------
  // constants
  //-------------------------------------------------------------
  localparam STATE_BITS = 1;
  localparam IDLE_ST    = 0;
  localparam TRAN_ST    = IDLE_ST + 1;


  //-------------------------------------------------------------
  // internal signals
  //-------------------------------------------------------------
  reg old_ack;  // remember previous value of SL_ACK_IN
  reg acked;    // detect a transition in SL_ACK_IN

  reg [STATE_BITS - 1:0] state;  // current state


  //-------------------------------------------------------------
  // packet serializer interface: generate flt_rdy
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      flt_rdy <= 1'b1;
    else
      case (state)
        IDLE_ST: if (flt_vld)
                   flt_rdy <= 1'b0;  // new flit, not ready for next

        default: if (acked)
                   flt_rdy <= 1'b1;  // flit acked, ready for next
	         else
                   flt_rdy <= 1'b0;
      endcase 


  //-------------------------------------------------------------
  // SpiNNaker link interface: generate SL_DATA_2OF7_OUT
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      SL_DATA_2OF7_OUT <= 7'd0;
    else
      case (state)
        IDLE_ST: if (flt_vld)
                   SL_DATA_2OF7_OUT <= flt_data;  // send new flit

        default: if (acked && flt_vld)
                   SL_DATA_2OF7_OUT <= flt_data;  // send new flit
      endcase 


  //-------------------------------------------------------------
  // remember last value of SL_ACK_IN
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      old_ack <= 1'b0;
    else
      old_ack <= SL_ACK_IN;


  //-------------------------------------------------------------
  // detect transition in SL_ACK_IN
  //-------------------------------------------------------------
  always @ (*)
    acked = (old_ack != SL_ACK_IN);


  //-------------------------------------------------------------
  // state machine
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      state <= IDLE_ST;
    else
      case (state)
        TRAN_ST: if (acked && !flt_vld)
                   state <= IDLE_ST;  // no new flit to send

        default: if (flt_vld)
                   state <= TRAN_ST;  // send new flit
      endcase 
endmodule
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
