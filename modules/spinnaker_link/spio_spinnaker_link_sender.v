// -------------------------------------------------------------------------
//  SpiNNaker link transmitter module
//
// -------------------------------------------------------------------------
// AUTHOR
//  lap - luis.plana@manchester.ac.uk
//  Based on work by J Pepper (Date 08/08/2012)
//
//  This module uses the STAC (synchronous timing/asynchronous control)
//  scheme to interface to SpiNNaker asynchronous channels.
//  This scheme is based on the predictive handshake scheme introduced
//  by Amir Yousefzadeh et al. from Instituto de Microelectrónica
//  de Sevilla (IMSE-CNM), CSIC and Universidad de Sevilla, Spain
//  (e-mail: reza@imse-cnm.csic.es) in: Amir Yousefzadeh, Luis A. Plana,
//  Steve Temple, Teresa Serrano-Gotarredona, Steven Furber and Bernabe
//  Linares-Barranco, "Fast Predictive Handshaking in Synchronous
//  FPGAs for Fully Asynchronous Multi-Symbol Chip Links. Application
//  to SpiNNaker 2-of-7 Links", IEEE Transactions on Circuits and Systems II:
//  Express Briefs, 2016. DOI: 10.1109/TCSII.2016.2531092
//
//  STAC differs from the predictive scheme in two ways:
//
//  1. It is aware of the possibility of back pressure
//  in asynchronous communication and combines predictive handshakes with
//  appropriate asynchronous handshakes for a correct response to back
//  pressure. This avoids potential breaches of communication protocol,
//  packet loss or corruption, and the need to resend packets.
//
//  2. There is no training step. This module works at a fixed frequency
//  based on SpiNNaker channel operation rates. This results in a simpler
//  implementation that can be used in both directions of communication
//  (which is not possible with the trained implementation), possibly
//  with lower throughput.
//
// -------------------------------------------------------------------------
// Taken from:
// https://solem.cs.man.ac.uk/svn/spiNNlink/testing/src/packet_sender.v
// Revision 2517 (Last-modified date: 2013-08-19 10:33:30 +0100)
//
// -------------------------------------------------------------------------
// COPYRIGHT
//  Copyright (c) The University of Manchester, 2012-2016.
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
module spio_spinnaker_link_sender
(
  input                        CLK_IN,
  input                        RESET_IN,

  // link error reporting
  output wire                  ACK_ERR_OUT,
  output wire                  TMO_ERR_OUT,

  // back-pressure point interface
  input                  [3:0] BPP_IN,
  input                  [4:0] BSF_LONG_IN,
  input                  [2:0] BAF_LONG_IN,

  // synchronous packet interface
  input      [`PKT_BITS - 1:0] PKT_DATA_IN,
  input                        PKT_VLD_IN,
  output                       PKT_RDY_OUT,

  // SpiNNaker link asynchronous interface
  output                 [6:0] SL_DATA_2OF7_OUT,
  input                        SL_ACK_IN
);

  //-------------------------------------------------------------
  // internal signals
  //-------------------------------------------------------------
  wire       synced_sl_ack;  // synchronized acknowledge input

  wire [6:0] flt_data;
  wire       flt_psz;
  wire       flt_vld;
  wire       flt_rdy;

		
  pkt_serializer ps
  (
    .CLK_IN           (CLK_IN),
    .RESET_IN         (RESET_IN),
    .PKT_DATA_IN      (PKT_DATA_IN),
    .PKT_VLD_IN       (PKT_VLD_IN),
    .PKT_RDY_OUT      (PKT_RDY_OUT),
    .flt_data         (flt_data),
    .flt_psz          (flt_psz),
    .flt_vld          (flt_vld),
    .flt_rdy          (flt_rdy)
  );

  flit_output_if fo
  (
    .CLK_IN           (CLK_IN),
    .RESET_IN         (RESET_IN),
    .BPP_IN           (BPP_IN),
    .BSF_LONG_IN      (BSF_LONG_IN),
    .BAF_LONG_IN      (BAF_LONG_IN),
    .ACK_ERR_OUT      (ACK_ERR_OUT),
    .TMO_ERR_OUT      (TMO_ERR_OUT),
    .flt_data         (flt_data),
    .flt_psz          (flt_psz),
    .flt_vld          (flt_vld),
    .flt_rdy          (flt_rdy),
    .SL_DATA_2OF7_OUT (SL_DATA_2OF7_OUT),
    .SL_ACK_IN        (synced_sl_ack)
  );

  spio_spinnaker_link_sync #(.SIZE(1)) sync
  ( .CLK_IN (CLK_IN),
    .IN     (SL_ACK_IN),
    .OUT    (synced_sl_ack)
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
  output reg                    flt_psz,
  output reg                    flt_vld,
  input                         flt_rdy
);

  //---------------------------------------------------------------
  // constants
  //---------------------------------------------------------------
  //# Xilinx recommends one-hot state encoding
  localparam STATE_BITS = 2;
  localparam IDLE_ST    = 0;
  localparam PARK_ST    = IDLE_ST + 1;
  localparam TRAN_ST    = PARK_ST + 1;


  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  reg                     pkt_acpt;  // accept a new input packet
  reg   [`PKT_BITS - 1:0] pkt_buf;   // buffer for input pkt
  reg                     pkt_long;  // remember input pkt length

  reg                     flt_busy;  // flit interface busy
  reg               [4:0] flt_cnt;   // count sent flits

  reg                     eop;       // time to send end-of-packet

  reg  [STATE_BITS - 1:0] state;     // current state


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
      PKT_RDY_OUT <= 1'b0;
    else
      case (state)
        IDLE_ST: if (pkt_acpt)
                   PKT_RDY_OUT <= 1'b0;  // start new pkt, not ready for next
                 else
                   PKT_RDY_OUT <= 1'b1;  // waiting for next pkt

        PARK_ST:   PKT_RDY_OUT <= 1'b0;  // not ready yet for next pkt

        default: if (eop && !flt_busy)
                   PKT_RDY_OUT <= 1'b1;  // finished, ready for next pkt
                 else
                   PKT_RDY_OUT <= 1'b0;  // not ready yet for next pkt
      endcase 


  //---------------------------------------------------------------
  // accept a new input packet
  //---------------------------------------------------------------
  always @ (*)
    pkt_acpt = (PKT_VLD_IN && PKT_RDY_OUT);
  //---------------------------------------------------------------

  //-------------------------------------------------------------
  // buffer packet (or part of it) to serialize it
  //-------------------------------------------------------------
  always @(posedge CLK_IN)
    case (state)
      IDLE_ST:
        case ({pkt_acpt, flt_busy})
          2'b10:   pkt_buf <= PKT_DATA_IN >> 4;  // first nibble gone
          2'b11:   pkt_buf <= PKT_DATA_IN;       // park new packet
        endcase

      default: if (!flt_busy)
                   pkt_buf <= pkt_buf >> 4;      // prepare for next nibble
    endcase 


  //-------------------------------------------------------------
  // remember length of packet
  //-------------------------------------------------------------
  always @(posedge CLK_IN)
    if ((state == IDLE_ST) && pkt_acpt)
      pkt_long <= PKT_DATA_IN[1];


  //-------------------------------------------------------------
  // flit interface: generate flt_data
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      flt_data <= 7'd0;
    else
      case (state)
        IDLE_ST: if (pkt_acpt && !flt_busy)
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
  // flit interface: generate flit packet size flag
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      flt_psz <= 1'b0;
    else
    if ((state == IDLE_ST) && pkt_acpt)
      flt_psz <= PKT_DATA_IN[1];


  //-------------------------------------------------------------
  // flit interface: generate flt_vld
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      flt_vld <= 1'b0;
    else
      case (state)
        IDLE_ST: if (!flt_busy)
                   if (pkt_acpt)
                     flt_vld <= 1'b1;  // first flit in pkt
                   else
                     flt_vld <= 1'b0;  // no new flit to send

        default: flt_vld <= 1'b1;  // next flit always available
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
      IDLE_ST,
      PARK_ST:   flt_cnt <= 1;

      default: if (!flt_busy)
                 flt_cnt <= flt_cnt + 1;  // one more flit gone
    endcase 


  //-------------------------------------------------------------
  // time to send end-of-packet
  //-------------------------------------------------------------
  always @ (*)
    eop = (!pkt_long && (flt_cnt == 10)) || (flt_cnt == 18);


  //-------------------------------------------------------------
  // state machine
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      state <= IDLE_ST;
    else
      case (state)
        IDLE_ST:
          case ({pkt_acpt, flt_busy})
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

  // back-pressure point interface
  input      [3:0] BPP_IN,
  input      [4:0] BSF_LONG_IN,
  input      [2:0] BAF_LONG_IN,

  // link error reporting
  output reg       ACK_ERR_OUT,
  output reg       TMO_ERR_OUT,

  // packet serializer interface
  input      [6:0] flt_data,
  input            flt_psz,
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
  localparam SYNC_ST    = 0;
  localparam ASYN_ST    = SYNC_ST + 1;

  localparam BSF_SHRT = 11;

  localparam INTER_FLT_DELAY = 1;


  //-------------------------------------------------------------
  // internal signals
  //-------------------------------------------------------------
  reg old_ack;  // remember previous value of SL_ACK_IN
  reg oap;      // remember previous value of SL_ACK_IN
  reg acked;    // detect a transition in SL_ACK_IN
  reg akp;      // detect a transition in SL_ACK_IN

  reg snd_asyn;  // can send flit

  reg pps;      // remember the size of the previous packet

  reg send_flit;  // send new flit to SpiNNaker

  reg [5:0] dat_cnt;  // keep track of data flits sent
  reg [5:0] ack_cnt;  // keep track of ack flits received
  reg       dat_dne;  // all data flits sent
  reg       ack_dne;  // all ack flits received

  reg [7:0] dly_cnt;  // keep track of inter flit clock cycles

  reg [STATE_BITS - 1:0] state;  // current state


  //-------------------------------------------------------------
  // packet serializer interface: generate flt_rdy
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      flt_rdy <= 1'b1;
    else
      case (state)
        SYNC_ST:
          casex ({send_flit, ack_dne, pps, dat_dne, (dly_cnt == 1)})
            5'b1xxxx: flt_rdy <= 1'b0;  // just sent psync flit
            5'bx11xx: flt_rdy <= 1'b0;  // not ready for next flit
            5'bx10xx: flt_rdy <= 1'b1;  // ready for next burst
	    5'bx0x01: flt_rdy <= 1'b1;  // ready for next flit
          endcase

        ASYN_ST:
          casex ({send_flit, ack_dne})
            2'b1x: flt_rdy <= 1'b1;  // just sent async flit (delayed!)
	    2'bx1: flt_rdy <= 1'b1;  // ready for next burst
	    2'b00: flt_rdy <= 1'b0;  // not ready yet for next flit
          endcase
      endcase 


  //-------------------------------------------------------------
  // remember the size of the previous packet
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      pps <= 1'b0;  // start assuming a short packet
    else
      case (state)
        SYNC_ST:
          if (ack_dne && (pps == 1'b0))
            pps <= flt_psz;

        ASYN_ST:
          if (ack_dne)
            pps <= flt_psz;
      endcase 


  //-------------------------------------------------------------
  // SpiNNaker link interface: generate SL_DATA_2OF7_OUT
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      SL_DATA_2OF7_OUT <= 7'd0;
    else
      if (send_flit)
        SL_DATA_2OF7_OUT <= flt_data;  // send new flit


  //-------------------------------------------------------------
  // send a new flit to SpiNNaker
  //-------------------------------------------------------------
  always @(*)
    if (RESET_IN)
      send_flit = 1'b0;
    else
      case (state)
        SYNC_ST:
          if (flt_vld && flt_rdy)
            send_flit = 1'b1;  // send new flit
          else
            send_flit = 1'b0;  // not ready for new flit

        ASYN_ST:
          if (flt_vld && (acked || snd_asyn) && !dat_dne)
            send_flit = 1'b1;  // send new flit
          else
            send_flit = 1'b0;  // not ready for new flit
      endcase 


  //-------------------------------------------------------------
  // remember last value of SL_ACK_IN
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      old_ack <= 1'b0;
    else
      if (send_flit)
        old_ack <= SL_ACK_IN;

  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      oap <= 1'b0;
    else
      oap <= SL_ACK_IN;


  //-------------------------------------------------------------
  // detect transition in SL_ACK_IN
  //-------------------------------------------------------------
  always @ (*)
    acked = (old_ack != SL_ACK_IN);

  always @ (*)
    akp = (oap != SL_ACK_IN);

  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      snd_asyn <= 1'b0;
    else
      case (state)
        SYNC_ST:
          if (ack_dne && pps)
	    snd_asyn <= 1'b1;
          else
            snd_asyn <= 1'b0;

        ASYN_ST:
            snd_asyn <= 1'b0;
      endcase 


  //-------------------------------------------------------------
  // keep track of data flits sent
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      dat_cnt <= BPP_IN;
    else
      case (state)
        SYNC_ST:
          casex ({send_flit, ack_dne, pps, flt_psz})
            4'b1xxx: dat_cnt <= dat_cnt - 1;
	    4'bx100: dat_cnt <= BSF_SHRT;
	    4'bx101: dat_cnt <= BSF_LONG_IN;
	    4'bx11x: dat_cnt <= BAF_LONG_IN;
          endcase

        ASYN_ST:
          casex ({send_flit, ack_dne, flt_psz})
            3'b1xx: dat_cnt <= dat_cnt - 1;
	    3'bx10: dat_cnt <= BSF_SHRT;
	    3'bx11: dat_cnt <= BSF_LONG_IN;
          endcase
      endcase 

  always @(*)
    dat_dne <= (dat_cnt == 0);


  //-------------------------------------------------------------
  // keep track of ack flits received
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      ack_cnt <= BPP_IN + 1;  // SpiNNaker acks on reset!
    else
      case (state)
        SYNC_ST:
          casex ({akp, ack_dne, pps, flt_psz})
            4'b10xx: ack_cnt <= ack_cnt - 1;
	    4'bx100: ack_cnt <= BSF_SHRT;
	    4'bx101: ack_cnt <= BSF_LONG_IN;
	    4'bx11x: ack_cnt <= BAF_LONG_IN;
          endcase

        ASYN_ST:
          casex ({akp, ack_dne, flt_psz})
            3'b10x: ack_cnt <= ack_cnt - 1;
	    3'bx10: ack_cnt <= BSF_SHRT;
	    3'bx11: ack_cnt <= BSF_LONG_IN;
          endcase
      endcase 

  always @(*)
    ack_dne <= (ack_cnt == 0) || ((ack_cnt == 1) && akp);


  //-------------------------------------------------------------
  // keep track of inter flit clock cycles
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      dly_cnt <= 0;
    else
      casex ({send_flit, (dly_cnt == 0)})
        2'b1x: dly_cnt <= INTER_FLT_DELAY;   // flit sent
        2'b00: dly_cnt <= dly_cnt - 1;       // wait before sending flit
      endcase


  //---------------------------------------------------------------
  // error reporting (2 clock cycles -- counter runs on slower clock)
  //---------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      ACK_ERR_OUT <= 1'b0;
    else
      ACK_ERR_OUT <= 1'b0;


  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      TMO_ERR_OUT <= 1'b0;
    else
      TMO_ERR_OUT <= 1'b0;
  //---------------------------------------------------------------


  //-------------------------------------------------------------
  // state machine
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      state <= SYNC_ST;
    else
      case (state)
        SYNC_ST: if ((ack_dne) && (pps == 1'b1))
                   state <= ASYN_ST;

        ASYN_ST: if (ack_dne)
                   state <= SYNC_ST;
      endcase 
endmodule
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
