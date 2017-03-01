// -------------------------------------------------------------------------
// $Id: in_mapper.v 2615 2013-10-02 10:39:58Z plana $
// -------------------------------------------------------------------------
// COPYRIGHT
// Copyright (c) The University of Manchester, 2012. All rights reserved.
// SpiNNaker Project
// Advanced Processor Technologies Group
// School of Computer Science
// -------------------------------------------------------------------------
// Project            : bidirectional SpiNNaker link to AER device interface
// Module             : AER event to SpiNNaker packet mapper
// Author             : lap/Jeff Pepper/Simon Davidson
// Status             : Review pending
// $HeadURL: https://solem.cs.man.ac.uk/svn/spinn_aer2_if/in_mapper.v $
// Last modified on   : $Date: 2013-10-02 11:39:58 +0100 (Wed, 02 Oct 2013) $
// Last modified by   : $Author: plana $
// Version            : $Revision: 2615 $
// -------------------------------------------------------------------------


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//--------------------------- in_mapper -------------------------
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`timescale 1ns / 1ps
module in_mapper
#(
  parameter MODE_BITS = 4
)
(
  input  wire                   rst,
  input  wire                   clk,

  // control and status interface
  input  wire [MODE_BITS - 1:0] mode,
  output reg                    dump_mode,

  // input AER device interface
  input  wire            [15:0] iaer_data,
  input  wire                   iaer_req,
  output reg                    iaer_ack,

  // SpiNNaker packet interface
  output reg             [71:0] ipkt_data,
  output reg                    ipkt_vld,
  input  wire                   ipkt_rdy
);
  //---------------------------------------------------------------
  // constants
  //---------------------------------------------------------------
  localparam RET_128_DEF = 0;
  localparam RET_64_DEF  = RET_128_DEF + 1;
  localparam RET_32_DEF  = RET_64_DEF  + 1;
  localparam RET_16_DEF  = RET_32_DEF  + 1;
  localparam COCHLEA_DEF = RET_16_DEF  + 1;
  localparam DIRECT_DEF  = COCHLEA_DEF + 1;
  localparam RET_128_ALT = DIRECT_DEF  + 1;
  localparam RET_64_ALT  = RET_128_ALT + 1;
  localparam RET_32_ALT  = RET_64_ALT  + 1;
  localparam RET_16_ALT  = RET_32_ALT  + 1;
  localparam COCHLEA_ALT = RET_16_ALT  + 1;
  localparam DIRECT_ALT  = COCHLEA_ALT + 1;
  localparam LAST_VALUE  = DIRECT_ALT;

  // alternative chip coordinates
  localparam CHIP_ADDR_DEF = 16'h0200;
  localparam CHIP_ADDR_ALT = 16'hfefe;

  localparam STATE_BITS = 2;
  localparam IDLE_ST    = 0;
  localparam WTRQ_ST    = IDLE_ST + 1;
  localparam DUMP_ST    = WTRQ_ST + 1;


  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  reg [STATE_BITS - 1:0] state;

  reg             [15:0] chip_addr;

  reg             [14:0] coords;
  wire             [6:0] new_x, new_y;
  wire                   sign_bit;

  reg                    busy;  // output not ready for next packet
  reg              [7:0] dump_ctr;


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //------------------------ aer interface ------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // generate aer ack signal
  //---------------------------------------------------------------
  always @(posedge clk or posedge rst)
    if (rst)
      iaer_ack <= 1'b1;  // active LOW!
    else
      case (state)
        IDLE_ST:
          if (!iaer_req && !busy)
            iaer_ack <= 1'b0;
          else
            iaer_ack <= 1'b1;  // no change!

	WTRQ_ST:
          if (iaer_req)
            iaer_ack <= 1'b1;
          else
            iaer_ack <= 1'b0;  // no change!
	
	DUMP_ST:
            iaer_ack <= iaer_req;  // simply complete handshake!
	
	default:  
            iaer_ack <= iaer_ack;  // no change!
      endcase
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //----------------------------- map -----------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // mode selection 
  //---------------------------------------------------------------
  always @(*)
    case (mode)
      RET_64_DEF,  // retina 64x64 mode
      RET_64_ALT:  coords = {sign_bit, 2'b00, new_y[6:1], new_x[6:1]};

      RET_32_DEF,  // retina 32x32 mode
      RET_32_ALT:  coords = {sign_bit, 4'b0000, new_y[6:2], new_x[6:2]};

      RET_16_DEF,  // retina 16x16 mode
      RET_16_ALT:  coords = {sign_bit, 6'b000000, new_y[6:3], new_x[6:3]};

      COCHLEA_DEF, // cochlea mode
      COCHLEA_ALT: coords = {3'b000, iaer_data[1], 3'b000,
                              iaer_data[7:2],iaer_data[9:8]
                            };

      DIRECT_DEF,  // straight-through mode
      DIRECT_ALT:  coords = iaer_data[14:0];

      default:     // make the retina 128x128 mode the default
                   coords = {sign_bit, new_y, new_x};
    endcase
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // new on 04/05/2012. 
  // retina mapper rotates image by 90 degrees clockwise
  //---------------------------------------------------------------
  assign new_x = 7'b1111111 - iaer_data[14:8]; // new_X = 127 - old_Y
  assign new_y = 7'b1111111 - iaer_data[7:1];  // new_Y = 127 - old_X
  assign sign_bit = iaer_data[0];
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // virtual chip address selection
  //---------------------------------------------------------------
  always @(*)
    case (mode)
      RET_128_ALT,
      RET_64_ALT,
      RET_32_ALT,
      RET_16_ALT,
      COCHLEA_ALT,
      DIRECT_ALT:  chip_addr = CHIP_ADDR_ALT;

                   // DEF is the default address!
      default:     chip_addr = CHIP_ADDR_DEF;
    endcase
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------- packet interface -----------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  wire [38:0]  pkt_bits;
  wire         parity;
  reg          nxt_vld;
   
  assign pkt_bits = {chip_addr, iaer_data[15], coords, 7'd0};
  assign parity   = ~(^pkt_bits);

  always @(posedge clk or posedge rst)
    if (rst)
      ipkt_data <= 72'd0;  // not really necessary!
    else
      case (state)
        IDLE_ST:
          if (!iaer_req && !busy)
            ipkt_data <= {32'd0, pkt_bits, parity};  // no payload!
          else
            ipkt_data <= ipkt_data;  // no change!

	default:  
            ipkt_data <= ipkt_data;  // no change!
      endcase

  always @(posedge clk or posedge rst)
    if (rst)
      ipkt_vld <= 1'b0;
    else
      if (nxt_vld || busy)
        ipkt_vld <= 1'b1;
      else
        ipkt_vld <= 1'b0;
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // new packet valid -- can send
  //---------------------------------------------------------------
  always @(*)
    case (state)
    	IDLE_ST: if (!iaer_req && !busy)
                   nxt_vld <= 1'b1;
                 else
                   nxt_vld <= 1'b0;

    	default:   nxt_vld <= 1'b0;
    endcase 
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // busy (output packet interface not ready to accept new one)
  //---------------------------------------------------------------
  always @(*)
    busy = ipkt_vld && !ipkt_rdy;
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //--------------------------- control ---------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // dump events from AER device if SpiNNaker not responding!
  // dump after 128 cycles without SpiNNaker response
  //---------------------------------------------------------------
  always @(posedge clk or posedge rst)
    if (rst)
      dump_ctr <= 8'd128;
    else
      if (ipkt_rdy)
        dump_ctr <= 8'd128;  // spinn_driver ready resets counter
      else if (dump_ctr != 5'd0)
        dump_ctr <= dump_ctr - 1;
      else
        dump_ctr <= dump_ctr;  // no change!
 
  always @(posedge clk or posedge rst)
    if (rst)
      dump_mode <= 1'b0;
    else
      if (state == DUMP_ST)
        dump_mode <= 1'b1;
      else
        dump_mode <= 1'b0;
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // in_mapper state machine
  //---------------------------------------------------------------
  always @(posedge clk or posedge rst)
    if (rst)
      state <= IDLE_ST;
    else
      case (state)
        IDLE_ST:
          casex ({(dump_ctr == 0), iaer_req, busy})
            3'b1xx:  state <= DUMP_ST;
            3'b000:  state <= WTRQ_ST;
            default: state <= IDLE_ST;  // no change!
	  endcase

	WTRQ_ST:
          if (iaer_req)
            state <= IDLE_ST;
          else
            state <= WTRQ_ST;  // no change!

	DUMP_ST:
          case ({ipkt_rdy, iaer_req})
            2'b10:   state <= WTRQ_ST;
            2'b11:   state <= IDLE_ST;
            default: state <= DUMP_ST;  // no change!
	  endcase
	
	default:  
            state <= state;  // no change!
      endcase
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
endmodule
