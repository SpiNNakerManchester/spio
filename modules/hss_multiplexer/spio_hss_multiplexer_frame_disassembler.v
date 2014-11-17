// -------------------------------------------------------------------------
// $Id: frame_disassembler.v 2515 2013-08-19 07:50:12Z plana $
//  spiNNlink frame receive/disassemble module
//
// -------------------------------------------------------------------------
// AUTHOR
//  lap - luis.plana@manchester.ac.uk
//  Based on work by J Pepper (Date 08/08/2012)
//
// -------------------------------------------------------------------------
// DETAILS
//  Created on       : 28 Nov 2012
//  Version          : $Revision: 2515 $
//  Last modified on : $Date: 2013-08-19 08:50:12 +0100 (Mon, 19 Aug 2013) $
//  Last modified by : $Author: plana $
//  $HeadURL: https://solem.cs.man.ac.uk/svn/spiNNlink/testing/src/frame_disassembler.v $
//
// -------------------------------------------------------------------------
// COPYRIGHT
//  Copyright (c) The University of Manchester, 2012. All rights reserved.
//  SpiNNaker Project
//  Advanced Processor Technologies Group
//  School of Computer Science
// -------------------------------------------------------------------------
// TODO
//  * fix error detection and reporting
// -------------------------------------------------------------------------


// ----------------------------------------------------------------
// include spiNNlink global constants and parameters
//
`include "spio_hss_multiplexer_common.h"
// ----------------------------------------------------------------

`timescale 1ns / 1ps
module spio_hss_multiplexer_frame_disassembler
(
  input  wire 			 clk,
  input  wire 			 rst,

  // register interface (to register bank)
  output reg                     reg_crce,
  output reg                     reg_frme,
  output reg                     reg_dfrm,
  output reg                     reg_rooc,
  output reg  [`IDLE_BITS - 1:0] reg_idsi,

  // high-speed link interface (from gtp)
  input  wire  [`FRM_BITS - 1:0] hsl_data,
  input  wire  [`KCH_BITS - 1:0] hsl_kchr,
  input  wire                    hsl_vld,

  // ack/nack interface (to frame assembler)
  // regain tx credit and frame resend control
  output reg                     ack_type,
  output reg   [`CLR_BITS - 1:0] ack_colour,
  output reg   [`SEQ_BITS - 1:0] ack_seq,
  output reg  			 ack_vld,

  // channel flow control interface (to frame assembler)
  // use remote cfc to mask local channels
  output reg  [`NUM_CHANS - 1:0] cfc_rem,
 
  // packet interface (to packet dispatcher)
  output reg   [`PKT_BITS - 1:0] pkt_data0,
  output reg  			 pkt_vld0,

  output reg   [`PKT_BITS - 1:0] pkt_data1,
  output reg  			 pkt_vld1,

  output reg   [`PKT_BITS - 1:0] pkt_data2,
  output reg  			 pkt_vld2,

  output reg   [`PKT_BITS - 1:0] pkt_data3,
  output reg  			 pkt_vld3,

  output reg   [`PKT_BITS - 1:0] pkt_data4,
  output reg  			 pkt_vld4,

  output reg   [`PKT_BITS - 1:0] pkt_data5,
  output reg  			 pkt_vld5,

  output reg   [`PKT_BITS - 1:0] pkt_data6,
  output reg  			 pkt_vld6,

  output reg   [`PKT_BITS - 1:0] pkt_data7,
  output reg  			 pkt_vld7,

  // frame interface (to packet dispatcher)
  output reg   [`CLR_BITS - 1:0] frm_colour,
  output reg   [`SEQ_BITS - 1:0] frm_seq,
  output reg  			 frm_vld,

  // out-of-credit interface (to packet dispatcher)
  output reg   [`CLR_BITS - 1:0] ooc_colour,
  output reg  			 ooc_vld
);

  //---------------------------------------------------------------
  // constants
  //---------------------------------------------------------------
  //# Xilinx recommends one-hot state encoding
  localparam STATE_BITS = 5;
  localparam STRT_ST = 0;
  localparam HDR0_ST = STRT_ST + 1;
  localparam HDR1_ST = HDR0_ST + 1;
  localparam KEY0_ST = HDR1_ST + 1;
  localparam PLD0_ST = KEY0_ST + 1;
  localparam KEY1_ST = PLD0_ST + 1;
  localparam PLD1_ST = KEY1_ST + 1;
  localparam KEY2_ST = PLD1_ST + 1;
  localparam PLD2_ST = KEY2_ST + 1;
  localparam KEY3_ST = PLD2_ST + 1;
  localparam PLD3_ST = KEY3_ST + 1;
  localparam KEY4_ST = PLD3_ST + 1;
  localparam PLD4_ST = KEY4_ST + 1;
  localparam KEY5_ST = PLD4_ST + 1;
  localparam PLD5_ST = KEY5_ST + 1;
  localparam KEY6_ST = PLD5_ST + 1;
  localparam PLD6_ST = KEY6_ST + 1;
  localparam KEY7_ST = PLD6_ST + 1;
  localparam PLD7_ST = KEY7_ST + 1;
  localparam LAST_ST = PLD7_ST + 1;
 

  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  reg  [STATE_BITS - 1:0] state;
  reg  [STATE_BITS - 1:0] nxp_state;
  reg  [`NUM_CHANS - 1:0] nxp_mask;
  
  reg  [`NUM_CHANS - 1:0] pre_dat;
  reg  [`NUM_CHANS - 1:0] len_dat;

  reg crc_error;
  reg frm_error;

  reg nak_frm;
  reg ack_frm;
  reg ooc_frm;
  reg dat_frm;
  reg lst_frm;
  reg cfc_frm;
  reg idl_frm;
  reg vld_frm;
  reg bad_frm;


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //------------------------------ crc ----------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // compute crc on the fly (combinatorial)
  //---------------------------------------------------------------
  reg    [`FRM_BITS - 1:0] crc_in;
  wire   [`FRM_BITS - 1:0] crc_out;
  reg                      crc_last;
  reg                      crc_chk;

  spio_hss_multiplexer_crc_gen crc_gen_0
  (
    .clk      (clk),
    .rst      (rst),
    .crc_go   (hsl_vld),
    .crc_last (crc_last),
    .crc_in   (crc_in),
    .crc_out  (crc_out)
  );
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // crc input multiplexer (combinatorial)
  //---------------------------------------------------------------
  always @ (*)
    if (crc_last)
      crc_in = {hsl_data[31:16], `CRC_PAD};
    else
      crc_in = hsl_data;
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // crc check and initialization (combinatorial)
  //---------------------------------------------------------------
  always @ (*)
    crc_chk = nak_frm || ack_frm || ooc_frm || lst_frm || cfc_frm;

  always @ (*)
    crc_last = crc_chk || frm_error || idl_frm;
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // crc error flag
  //---------------------------------------------------------------
  always @ (*)
    if (crc_chk)
      crc_error = (hsl_data[`FRM_CRC_RNG] != crc_out[`FRM_CRC_RNG]);
    else
      crc_error = 1'b0;
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //--------------------------- datapath --------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // packet interface
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
    begin
      pkt_data0 <= {`FRM_BITS {1'b0}}; // not really necessary!
      pkt_data1 <= {`FRM_BITS {1'b0}}; // not really necessary!
      pkt_data2 <= {`FRM_BITS {1'b0}}; // not really necessary!
      pkt_data3 <= {`FRM_BITS {1'b0}}; // not really necessary!
      pkt_data4 <= {`FRM_BITS {1'b0}}; // not really necessary!
      pkt_data5 <= {`FRM_BITS {1'b0}}; // not really necessary!
      pkt_data6 <= {`FRM_BITS {1'b0}}; // not really necessary!
      pkt_data7 <= {`FRM_BITS {1'b0}}; // not really necessary!
    end
    else
      if (hsl_vld)
        case (state)
          HDR0_ST: begin
                     pkt_data0[`PKT_HDR_RNG] <= hsl_data[`DFRM_HD0_RNG];
                     pkt_data1[`PKT_HDR_RNG] <= hsl_data[`DFRM_HD1_RNG];
                     pkt_data2[`PKT_HDR_RNG] <= hsl_data[`DFRM_HD2_RNG];
                     pkt_data3[`PKT_HDR_RNG] <= hsl_data[`DFRM_HD3_RNG];
                   end
          HDR1_ST: begin
                     pkt_data4[`PKT_HDR_RNG] <= hsl_data[`DFRM_HD4_RNG];
                     pkt_data5[`PKT_HDR_RNG] <= hsl_data[`DFRM_HD5_RNG];
                     pkt_data6[`PKT_HDR_RNG] <= hsl_data[`DFRM_HD6_RNG];
                     pkt_data7[`PKT_HDR_RNG] <= hsl_data[`DFRM_HD7_RNG];
                   end
          KEY0_ST: pkt_data0[`PKT_KEY_RNG] <= hsl_data;
          PLD0_ST: pkt_data0[`PKT_PLD_RNG] <= hsl_data;
          KEY1_ST: pkt_data1[`PKT_KEY_RNG] <= hsl_data;
          PLD1_ST: pkt_data1[`PKT_PLD_RNG] <= hsl_data;
          KEY2_ST: pkt_data2[`PKT_KEY_RNG] <= hsl_data;
          PLD2_ST: pkt_data2[`PKT_PLD_RNG] <= hsl_data;
          KEY3_ST: pkt_data3[`PKT_KEY_RNG] <= hsl_data;
          PLD3_ST: pkt_data3[`PKT_PLD_RNG] <= hsl_data;
          KEY4_ST: pkt_data4[`PKT_KEY_RNG] <= hsl_data;
          PLD4_ST: pkt_data4[`PKT_PLD_RNG] <= hsl_data;
          KEY5_ST: pkt_data5[`PKT_KEY_RNG] <= hsl_data;
          PLD5_ST: pkt_data5[`PKT_PLD_RNG] <= hsl_data;
          KEY6_ST: pkt_data6[`PKT_KEY_RNG] <= hsl_data;
          PLD6_ST: pkt_data6[`PKT_PLD_RNG] <= hsl_data;
          KEY7_ST: pkt_data7[`PKT_KEY_RNG] <= hsl_data;
          PLD7_ST: pkt_data7[`PKT_PLD_RNG] <= hsl_data;
        endcase

  always @ (posedge clk or posedge rst)
    if (rst)
    begin
      pkt_vld0  <= 1'b0;
      pkt_vld1  <= 1'b0;
      pkt_vld2  <= 1'b0;
      pkt_vld3  <= 1'b0;
      pkt_vld4  <= 1'b0;
      pkt_vld5  <= 1'b0;
      pkt_vld6  <= 1'b0;
      pkt_vld7  <= 1'b0;
    end
    else
      if (hsl_vld && (state == LAST_ST) && !frm_error && !crc_error)
        begin
          pkt_vld0 <= pre_dat[0];
          pkt_vld1 <= pre_dat[1];
          pkt_vld2 <= pre_dat[2];
          pkt_vld3 <= pre_dat[3];
          pkt_vld4 <= pre_dat[4];
          pkt_vld5 <= pre_dat[5];
          pkt_vld6 <= pre_dat[6];
          pkt_vld7 <= pre_dat[7];
        end
      else
        begin
          pkt_vld0 <= 1'b0;
          pkt_vld1 <= 1'b0;
          pkt_vld2 <= 1'b0;
          pkt_vld3 <= 1'b0;
          pkt_vld4 <= 1'b0;
          pkt_vld5 <= 1'b0;
          pkt_vld6 <= 1'b0;
          pkt_vld7 <= 1'b0;
        end
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // frame interface
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
    begin
      len_dat <= {`NUM_CHANS {1'b0}}; // not really necessary!
      pre_dat <= {`NUM_CHANS {1'b0}}; // not really necessary!

      frm_colour <= {`CLR_BITS {1'b0}};  // not really necessary!
      frm_seq    <= `SEQ_BITS'd0;        // not really necessary!
    end
    else
      if (dat_frm)
      begin
        len_dat <= hsl_data[`DFRM_LEN_RNG];
        pre_dat <= hsl_data[`DFRM_PRE_RNG];

        frm_colour <= hsl_data[`FRM_CLR_RNG];
        frm_seq    <= hsl_data[`FRM_SEQ_RNG];
      end

  always @ (posedge clk or posedge rst)
    if (rst)
      frm_vld <= 1'b0;
    else
      frm_vld <= lst_frm && !frm_error && !crc_error;
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // Idle frame handling (simply latch the latest sentinel value)
  //---------------------------------------------------------------

  always @ (posedge clk or posedge rst)
    if (rst)
      reg_idsi <= {`IDLE_BITS{1'b1}};
    else
      if (idl_frm)
        reg_idsi <= hsl_data[`IDLE_BITS-1:0];

  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // out-of-credit interface
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      ooc_vld <= 1'b0;
    else
      ooc_vld <= ooc_frm && !frm_error && !crc_error;

  always @ (posedge clk or posedge rst)
    if (rst)
      ooc_colour <= 1'b0;  // not really necessary!
    else
      if (ooc_frm && !frm_error && !crc_error)
        ooc_colour <= hsl_data[`FRM_CLR_RNG];
                   
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // ack/nak interface
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
    begin
      ack_type   <= `NAK_T;             // not really necessary!
      ack_colour <= {`CLR_BITS {1'b0}}; // not really necessary!
      ack_seq    <= `SEQ_BITS'd0;       // not really necessary!
      ack_vld    <= 1'b0;
    end // if (rst)
    else
      casex ({frm_error, crc_error, ack_frm, nak_frm})
        4'b001x: begin
                   ack_type   <= `ACK_T;
                   ack_colour <= hsl_data[`FRM_CLR_RNG];
                   ack_seq    <= hsl_data[`FRM_SEQ_RNG];
                   ack_vld    <= 1'b1;
                 end
	4'b0001: begin
                   ack_type   <= `NAK_T;
                   ack_colour <= hsl_data[`FRM_CLR_RNG];
                   ack_seq    <= hsl_data[`FRM_SEQ_RNG];
                   ack_vld    <= 1'b1;
                 end
	default:   ack_vld    <= 1'b0;
      endcase
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // channel flow control interface
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      cfc_rem <= {`NUM_CHANS {1'b1}};
    else
      if ((lst_frm || cfc_frm) && !frm_error && !crc_error)
        cfc_rem <= hsl_data[`DFRM_CFC_RNG];
      else
        cfc_rem <= cfc_rem;  // no change!
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // register bank interface
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      reg_dfrm <= 1'b0;
    else
      reg_dfrm <= lst_frm && !frm_error && !crc_error;

  always @ (posedge clk or posedge rst)
    if (rst)
      reg_crce <= 1'b0;
    else
      reg_crce <= hsl_vld && crc_error;

  always @ (posedge clk or posedge rst)
    if (rst)
      reg_frme <= 1'b0;
    else
      reg_frme <= hsl_vld && frm_error;

  always @ (posedge clk or posedge rst)
    if (rst)
      reg_rooc <= 1'b0;
    else
      reg_rooc <= ooc_frm && !frm_error && !crc_error;
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	 
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------- control --------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // frame type decoding (combinatorial)
  //---------------------------------------------------------------
  always @ (*)
    bad_frm = !idl_frm && !ack_frm && !nak_frm && !ooc_frm && !cfc_frm && !dat_frm;

  // Idle frames have two k-chars
  always @ (*)
    idl_frm = hsl_vld && (hsl_kchr == `IDLE_KBITS) && (hsl_data[31:16] == `KCH_IDLE);

  // Normal frames have one kchar
  always @ (*)
    vld_frm = hsl_vld && (hsl_kchr == {1'b1, {(`KCH_BITS - 1) {1'b0}}});

  always @ (*)
    ack_frm = vld_frm && (hsl_data[`FRM_KCH_RNG] == `KCH_ACK);

  always @ (*)
    nak_frm = vld_frm && (hsl_data[`FRM_KCH_RNG] == `KCH_NAK);

  always @ (*)
    ooc_frm = vld_frm && (hsl_data[`FRM_KCH_RNG] == `KCH_OOC);

  always @ (*)
    cfc_frm = vld_frm && (hsl_data[`FRM_KCH_RNG] == `KCH_CFC);

  // first word in a data frame
  always @ (*)
    dat_frm = vld_frm && (hsl_data[`FRM_KCH_RNG] == `KCH_DATA);

  // last word in a data frame
  always @ (*)
    lst_frm = hsl_vld && (state == LAST_ST);
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // frame error flag
  //---------------------------------------------------------------
  always @ (*)
    frm_error = (((state == STRT_ST) && bad_frm)
	          || ((state != STRT_ST) && hsl_vld && (hsl_kchr != 0))
                );
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // next-possible state (based on channel presence mask)
  // (combinatorial)
  //---------------------------------------------------------------
  always @ (*)
    casex (pre_dat & nxp_mask)
      8'bxxxxxxx1: nxp_state = KEY0_ST;
      8'bxxxxxx10: nxp_state = KEY1_ST;
      8'bxxxxx100: nxp_state = KEY2_ST;
      8'bxxxx1000: nxp_state = KEY3_ST;
      8'bxxx10000: nxp_state = KEY4_ST;
      8'bxx100000: nxp_state = KEY5_ST;
      8'bx1000000: nxp_state = KEY6_ST;
      8'b10000000: nxp_state = KEY7_ST;
      8'b00000000: nxp_state = LAST_ST;
    endcase
  //---------------------------------------------------------------
  
  //---------------------------------------------------------------
  // next-possible-state mask (based on current state)
  // (combinatorial)
  //---------------------------------------------------------------
  always @ (*)
    casex (state)
      STRT_ST: nxp_mask = 8'b11111111;
      HDR0_ST: nxp_mask = 8'b11111111;
      HDR1_ST: nxp_mask = 8'b11111111;
      KEY0_ST: nxp_mask = 8'b11111110;
      PLD0_ST: nxp_mask = 8'b11111110;
      KEY1_ST: nxp_mask = 8'b11111100;
      PLD1_ST: nxp_mask = 8'b11111100;
      KEY2_ST: nxp_mask = 8'b11111000;
      PLD2_ST: nxp_mask = 8'b11111000;
      KEY3_ST: nxp_mask = 8'b11110000;
      PLD3_ST: nxp_mask = 8'b11110000;
      KEY4_ST: nxp_mask = 8'b11100000;
      PLD4_ST: nxp_mask = 8'b11100000;
      KEY5_ST: nxp_mask = 8'b11000000;
      PLD5_ST: nxp_mask = 8'b11000000;
      KEY6_ST: nxp_mask = 8'b10000000;
      PLD6_ST: nxp_mask = 8'b10000000;
      KEY7_ST: nxp_mask = 8'b11111111;
      PLD7_ST: nxp_mask = 8'b11111111;
      LAST_ST: nxp_mask = 8'b11111111;
      default: nxp_mask = 8'bxxxxxxxx;
    endcase // case (state)
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // next state
  //---------------------------------------------------------------
  always @ (posedge clk or posedge rst)
    if (rst)
      state <= STRT_ST;
    else
      if (hsl_vld)
        case (state)
          STRT_ST: if (dat_frm)
                     state <= HDR0_ST;
                   else
                     state <= STRT_ST;  // no change!
          HDR0_ST: if (frm_error)
                     state <= STRT_ST;
                   else
                     state <= HDR1_ST;
          HDR1_ST: if (frm_error)
                     state <= STRT_ST;
                   else
                     state <= nxp_state;
          KEY0_ST: if (frm_error)
                     state <= STRT_ST;
                   else if (len_dat[0])
                     state <= PLD0_ST;
                   else
                     state <= nxp_state;
          PLD0_ST: if (frm_error)
                     state <= STRT_ST;
                   else
                     state <= nxp_state;
          KEY1_ST: if (frm_error)
                     state <= STRT_ST;
                   else if (len_dat[1])
                     state <= PLD1_ST;
                   else
    		   state <= nxp_state;
          PLD1_ST: if (frm_error)
                     state <= STRT_ST;
                   else
                     state <= nxp_state;
          KEY2_ST: if (frm_error)
                     state <= STRT_ST;
                   else if (len_dat[2])
                     state <= PLD2_ST;
                   else
    		   state <= nxp_state;
          PLD2_ST: if (frm_error)
                     state <= STRT_ST;
                   else
                     state <= nxp_state;
          KEY3_ST: if (frm_error)
                     state <= STRT_ST;
                   else if (len_dat[3])
                     state <= PLD3_ST;
                   else
    		   state <= nxp_state;
          PLD3_ST: if (frm_error)
                     state <= STRT_ST;
                   else state <= nxp_state;
          KEY4_ST: if (frm_error)
                     state <= STRT_ST;
                   else if (len_dat[4])
                     state <= PLD4_ST;
                   else
    		   state <= nxp_state;
          PLD4_ST: if (frm_error)
                     state <= STRT_ST;
                   else
                     state <= nxp_state;
          KEY5_ST: if (frm_error)
                     state <= STRT_ST;
                   else if (len_dat[5])
                     state <= PLD5_ST;
                   else
    		   state <= nxp_state;
          PLD5_ST: if (frm_error)
                     state <= STRT_ST;
                   else
                     state <= nxp_state;
          KEY6_ST: if (frm_error)
                     state <= STRT_ST;
                   else if (len_dat[6])
                     state <= PLD6_ST;
                   else
    		   state <= nxp_state;
          PLD6_ST: if (frm_error)
                     state <= STRT_ST;
                   else
                     state <= nxp_state;
          KEY7_ST: if (frm_error)
                     state <= STRT_ST;
                   else if (len_dat[7])
                     state <= PLD7_ST;
                   else
    		   state <= LAST_ST;
          PLD7_ST: if (frm_error)
                     state <= STRT_ST;
                   else
                     state <= LAST_ST;
          LAST_ST: state <= STRT_ST;
        endcase
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

endmodule
