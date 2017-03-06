// -------------------------------------------------------------------------
//  SpiNNaker <-> AER interface controller
//
// -------------------------------------------------------------------------
// AUTHOR
//  lap - luis.plana@manchester.ac.uk
//  Based on work by J Pepper (Date 08/08/2012)
//
// -------------------------------------------------------------------------
// Adapted from:
// https://solem.cs.man.ac.uk/svn/spinn_aer2_if/in_mapper.v
// Revision 2615 (Last-modified date: 2013-10-02 11:39:58 +0100)
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


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`timescale 1ns / 1ps
module raggedstone_spinn_aer_if_dump
(
  input  wire                   rst,
  input  wire                   clk,

  // control and status interface
  input  wire                   go,
  output reg                    dump_mode,

  // AER bus interface
  input  wire            [15:0] iaer_data,
  input  wire                   iaer_req,
  output reg                    iaer_ack,

  // AER mapper interface
  output reg             [15:0] maer_data,
  output reg                    maer_req,
  input  wire                   maer_ack
);

  //---------------------------------------------------------------
  // constants
  //---------------------------------------------------------------
  // dump incoming events after 128 cycles without mapper response
  localparam DUMP_CNT   = 128;

  localparam STATE_BITS = 1;
  localparam IDLE_ST    = 0;
  localparam DUMP_ST    = IDLE_ST + 1;
  localparam WAIT_ST    = DUMP_ST + 1;


  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  reg [STATE_BITS - 1:0] state;

  reg                    busy;  // output not ready for next packet
  reg              [7:0] dump_ctr;


  //---------------------------------------------------------------
  // generate bus ack signal
  //---------------------------------------------------------------
  always @(*)
    if (rst)
      iaer_ack = 1'b1;  // active LOW!
    else
      case (state)
        IDLE_ST:
          if (go)
            iaer_ack = maer_ack;  // pass mapper ack
          else
            iaer_ack = iaer_req;  // complete handshake

	default:
            iaer_ack = iaer_req;  // complete handshake
      endcase
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // generate mapper data and req signals
  //---------------------------------------------------------------
  always @(*)
    maer_data = iaer_data;

  always @(*)
    if (rst)
      maer_req = 1'b1;  // active LOW!
    else
      case (state)
        IDLE_ST:
          if (go)
            maer_req = iaer_req;
          else
            maer_req = 1'b1;  // no requests sent

	DUMP_ST:
            maer_req = 1'b0;  // keep un-acked request

	default:
            maer_req = 1'b1;  // complete request handshake
      endcase
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // mapper busy, i.e., has not accepted requested event
  //---------------------------------------------------------------
  always @(*)
    busy = !maer_req && maer_ack;
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // dump events from AER bus if mapper not responding!
  //---------------------------------------------------------------
  always @(posedge clk or posedge rst)
    if (rst)
      dump_ctr <= DUMP_CNT;
    else
      if (!busy)
        dump_ctr <= DUMP_CNT;  // mapper ready resets counter
      else if (dump_ctr != 0)
        dump_ctr <= dump_ctr - 1;
 
  always @(posedge clk or posedge rst)
    if (rst)
      dump_mode <= 1'b0;
    else
      if ((state == DUMP_ST) || (state == WAIT_ST))
        dump_mode <= 1'b1;
      else
        dump_mode <= 1'b0;
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // controller state machine
  //---------------------------------------------------------------
  always @(posedge clk or posedge rst)
    if (rst)
      state <= IDLE_ST;
    else
      case (state)
        IDLE_ST:
          if (dump_ctr == 0)
            state <= DUMP_ST;

	DUMP_ST:
          if (!busy)
            state <= WAIT_ST;

	WAIT_ST:
          if (iaer_req && maer_ack)
            state <= IDLE_ST;

        default:
	    state <= IDLE_ST;
      endcase
endmodule
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
