// -------------------------------------------------------------------------
// $Id: user_int.v 2615 2013-10-02 10:39:58Z plana $
// -------------------------------------------------------------------------
// COPYRIGHT
// Copyright (c) The University of Manchester, 2012. All rights reserved.
// SpiNNaker Project
// Advanced Processor Technologies Group
// School of Computer Science
// -------------------------------------------------------------------------
// Project            : bidirectional SpiNNaker link to AER device interface
// Module             : user interface module
// Author             : lap/Jeff Pepper/Simon Davidson
// Status             : Review pending
// $HeadURL: https://solem.cs.man.ac.uk/svn/spinn_aer2_if/user_int.v $
// Last modified on   : $Date: 2013-10-02 11:39:58 +0100 (Wed, 02 Oct 2013) $
// Last modified by   : $Author: plana $
// Version            : $Revision: 2615 $
// -------------------------------------------------------------------------


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//------------------------ user_interface -----------------------
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`timescale 1ns / 1ps
module raggedstone_spinn_aer_if_user_int
#(
  // debouncer constant (can be adjusted for simulation!)
  parameter DBNCER_CONST = 20'hfffff,
  parameter MODE_BITS    = 4
)
(
  input  wire                   rst,
  input  wire                   clk,

  // control and status interface
  output reg  [MODE_BITS - 1:0] mode,

  // display interface (7-segment and leds)
  input  wire                   mode_sel,
  output reg              [7:0] o_7seg,
  output reg              [3:0] o_strobe,
  output wire                   o_led2
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

  localparam PRESCALE_BITS = 14;


  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  // signals for 7-segment display driver
  // ---------------------------------------------------------
  reg              [3:0] digit [0:3];
  reg              [3:0] point;
  reg              [1:0] curr_digit;

  reg                    display_clk;
  reg                    prescale_out;
  reg  [PRESCALE_BITS:0] prescale_cnt;


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //----------------------------- tasks ---------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // BCD to 7-segment converter
  // seven segment encoding:
  // bcd2sevenSeg[6:0] = abcdefg
  //---------------------------------------------------------------
  function [6:0] bcd2sevenSeg;
    input [3:0] bcd;
    case (bcd)
            0: bcd2sevenSeg = 7'b000_0001;  // 0
            1: bcd2sevenSeg = 7'b100_1111;  // 1
            2: bcd2sevenSeg = 7'b001_0010;  // 2
            3: bcd2sevenSeg = 7'b000_0110;  // 3
            4: bcd2sevenSeg = 7'b100_1100;  // 4
            5: bcd2sevenSeg = 7'b010_0100;  // 5
            6: bcd2sevenSeg = 7'b110_0000;  // 6
            7: bcd2sevenSeg = 7'b000_1111;  // 7
            8: bcd2sevenSeg = 7'b000_0000;  // 8
            9: bcd2sevenSeg = 7'b000_1100;  // 9
           10: bcd2sevenSeg = 7'b111_1111;  // space
           11: bcd2sevenSeg = 7'b111_0010;  // c
           12: bcd2sevenSeg = 7'b110_0010;  // o
           13: bcd2sevenSeg = 7'b110_1000;  // h
      default: bcd2sevenSeg = 7'b111_1111;  // all segments off
    endcase
  endfunction
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //----------------------- leds and buttons ----------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // Flash LED2 (activity indicator)
  //---------------------------------------------------------------
  reg [23:0] lstate;

  assign o_led2 = lstate[23];

  always @(posedge clk or posedge rst)
    if (rst)
      lstate <= 0;
    else
      if (lstate == 24'hffffff)
        lstate <= 0;
      else
        lstate <= lstate + 1;
  //---------------------------------------------------------------
  
  // ---------------------------------------------------------
  // debounce mode select pushbutton
  // ---------------------------------------------------------
  reg [19:0] mode_debounce_state;
  reg  [2:0] mode_bounce;
  reg        mode_sel_debounced;

  always @(posedge clk)
    begin
      mode_bounce[0] <= mode_sel;  
      mode_bounce[1] <= mode_bounce[0];
      mode_bounce[2] <= mode_bounce[1];
    end

  always @(posedge clk)
    if (mode_bounce[2] != mode_bounce[1]) 
      mode_debounce_state <= DBNCER_CONST;
    else
      if (mode_debounce_state != 0)
        mode_debounce_state <= mode_debounce_state - 1;
      else
        mode_debounce_state <= mode_debounce_state;  // no change!

  always @(posedge clk or posedge rst)
    if (rst)
      mode_sel_debounced <= 1'b1;
    else
      if ((mode_bounce[2] == mode_bounce[1])
           && (mode_debounce_state == 0)
         )
        mode_sel_debounced <= mode_bounce[2];
      else
        mode_sel_debounced <= mode_sel_debounced;  // no change!
   // ---------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //------------------------ mode selection -----------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // ---------------------------------------------------------
  // NOTE: only one mode change per button press
  // ---------------------------------------------------------
  reg sel_state;

  always @(posedge clk or posedge rst)
    if (rst)
      sel_state <= 0;
    else
      if (mode_sel_debounced == 1'b0)
        sel_state <= 1;
      else
        sel_state <= 0;
  // ---------------------------------------------------------

  // ---------------------------------------------------------
  // sequence through modes
  // ---------------------------------------------------------
  always @(posedge clk or posedge rst)
    if (rst)
      mode <= 3'b000;
    else
      if ((sel_state == 0) && (mode_sel_debounced == 1'b0))
      begin
        if (mode == LAST_VALUE)
          mode <= 0;
        else
          mode <= mode + 1;
      end
      else
        mode <= mode;  // no change!
  // ---------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //------------------------ display driver -----------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // ---------------------------------------------------------
  // display current mode in 7-segment displays
  // ---------------------------------------------------------
  always @(posedge clk)
    case (mode)
      RET_128_DEF,
      RET_128_ALT:
        begin
          digit[0] <= 4'd10;
          digit[1] <= 4'd1;
          digit[2] <= 4'd2;
          digit[3] <= 4'd8;
        end

      RET_64_DEF,
      RET_64_ALT:
        begin
          digit[0] <= 4'd10;
          digit[1] <= 4'd10;
          digit[2] <= 4'd6;
          digit[3] <= 4'd4;
        end

      RET_32_DEF,
      RET_32_ALT:
        begin
          digit[0] <= 4'd10;
          digit[1] <= 4'd10;
          digit[2] <= 4'd3;
          digit[3] <= 4'd2;
        end

      RET_16_DEF,
      RET_16_ALT:
        begin
          digit[0] <= 4'd10;
          digit[1] <= 4'd10;
          digit[2] <= 4'd1;
          digit[3] <= 4'd6;
        end

      COCHLEA_DEF,
      COCHLEA_ALT:
        begin
          digit[0] <= 4'd11;
          digit[1] <= 4'd12;
          digit[2] <= 4'd11;
          digit[3] <= 4'd13;
        end

      DIRECT_DEF,
      DIRECT_ALT:
        begin
          digit[0] <= 4'd0;
          digit[1] <= 4'd0;
          digit[2] <= 4'd0;
          digit[3] <= 4'd0;
        end
    endcase
  // ---------------------------------------------------------

  // ---------------------------------------------------------
  // 7-segment display driver
  // ---------------------------------------------------------
  //---------------------------------------------------------------
  // use decimal point to signal alternate chip_address
  //---------------------------------------------------------------
  always @(posedge clk)
    case (mode)
      RET_128_ALT,
      RET_64_ALT,
      RET_32_ALT,
      RET_16_ALT,
      COCHLEA_ALT,
      DIRECT_ALT:  point <= 4'b1110;

                   // no decimal point is the default
      default:     point <= 4'b1111;
    endcase
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // current digit selection
  //---------------------------------------------------------------
  always @(posedge display_clk or posedge rst)
    if (rst)
      curr_digit <= 0;
    else
      curr_digit <= curr_digit + 1;

  always @(posedge display_clk or posedge rst)
    if (rst)
      o_strobe <= 4'b0000;
    else
      case (curr_digit)
        0: o_strobe <= 4'b0001;
        1: o_strobe <= 4'b0010;
        2: o_strobe <= 4'b0100;
        3: o_strobe <= 4'b1000;
      endcase

  always @(posedge display_clk or posedge rst)
    if (rst)
      o_7seg <= 8'b1111_1111;
    else
      o_7seg <= {point[curr_digit], bcd2sevenSeg(digit[curr_digit])};
  // ---------------------------------------------------------

  // ---------------------------------------------------------
  // display clock generator (external clock scaled down)
  // ---------------------------------------------------------
  always @(posedge clk or posedge rst)
    if (rst)
      prescale_cnt <= 0;
    else
      prescale_cnt <= prescale_cnt + 1;

  always @(posedge clk or posedge rst)
    if (rst)
      prescale_out <= 0;
    else
      prescale_out <= (prescale_cnt == 0);

  always @(posedge prescale_out)
    display_clk <= ~display_clk;
  // ---------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
endmodule
