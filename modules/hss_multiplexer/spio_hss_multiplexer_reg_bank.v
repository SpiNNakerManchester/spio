// -------------------------------------------------------------------------
// $Id: reg_bank.v 2516 2013-08-19 09:27:43Z plana $
//  spiNNlink register bank module
//
// -------------------------------------------------------------------------
// AUTHOR
//  lap - luis.plana@manchester.ac.uk
//
// -------------------------------------------------------------------------
// DETAILS
//  Created on       : 28 Mar 2013
//  Version          : $Revision: 2516 $
//  Last modified on : $Date: 2013-08-19 10:27:43 +0100 (Mon, 19 Aug 2013) $
//  Last modified by : $Author: plana $
//  $HeadURL: https://solem.cs.man.ac.uk/svn/spiNNlink/testing/src/reg_bank.v $
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
`include "spio_hss_multiplexer_common.h"
`include "spio_hss_multiplexer_reg_bank.h"
// ----------------------------------------------------------------

`timescale 1ns / 1ps
module spio_hss_multiplexer_reg_bank
(
  input  wire 			 clk,
  input  wire 			 rst,

  // frame assembler interface
  input  wire                    reg_sfrm,
  input  wire                    reg_looc,
  input  wire [`CRDT_BITS - 1:0] reg_crdt,
  input  wire [`NUM_CHANS - 1:0] reg_empt,
  input  wire [`NUM_CHANS - 1:0] reg_full,

  // frame transmitter interface
  input  wire                    reg_tfrm,
  output reg  [`IDLE_BITS - 1:0] reg_idso,
  output reg                     reg_stop,

  // frame disassembler interface
  input  wire                    reg_dfrm,
  input  wire                    reg_crce,
  input  wire                    reg_frme,
  input  wire                    reg_rnak,
  input  wire                    reg_rack,
  input  wire                    reg_rooc,
  input  wire [`NUM_CHANS - 1:0] reg_cfcr,
  input  wire [`IDLE_BITS - 1:0] reg_idsi,

  // packet dispatcher interface
  input  wire                    reg_rfrm,
  input  wire                    reg_busy,
  input  wire                    reg_lnak,
  input  wire                    reg_lack,
  input  wire [`NUM_CHANS - 1:0] reg_cfcl,
  
  // TX/RX Control interface
  input  wire [1:0]              reg_hand,

  // register access interface
  input  wire                    reg_write,
  input  wire [`REGA_BITS - 1:0] reg_addr,
  output reg  [`REGD_BITS - 1:0] reg_read_data,
  input  wire [`REGD_BITS - 1:0] reg_write_data
);

  //---------------------------------------------------------------
  // constants
  //---------------------------------------------------------------
   

  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  reg  [`REGD_BITS - 1:0] looc_ctr;
  reg  [`REGD_BITS - 1:0] crce_ctr;
  reg  [`REGD_BITS - 1:0] frme_ctr;
  reg  [`REGD_BITS - 1:0] rfrm_ctr;
  reg  [`REGD_BITS - 1:0] dfrm_ctr;
  reg  [`REGD_BITS - 1:0] tfrm_ctr;
  reg  [`REGD_BITS - 1:0] sfrm_ctr;
  reg  [`REGD_BITS - 1:0] busy_ctr;
  reg  [`REGD_BITS - 1:0] lnak_ctr;
  reg  [`REGD_BITS - 1:0] rnak_ctr;
  reg  [`REGD_BITS - 1:0] lack_ctr;
  reg  [`REGD_BITS - 1:0] rack_ctr;
  reg  [`REGD_BITS - 1:0] rooc_ctr;
  reg  [`REGD_BITS - 1:0] reco_ctr;

  reg  [1:0] last_reg_hand;
  reg        reg_reco;


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //--------------------------- datapath --------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  //---------------------------------------------------------------
  // reg_hand edge detection
  //---------------------------------------------------------------
  always @(posedge clk or posedge rst)
    if (rst)
      last_reg_hand <= 2'b00;
    else
      last_reg_hand <= reg_hand;

  always @ (*)
    reg_reco = last_reg_hand[0] == 1'b0 && reg_hand[0] == 1'b1;

  //---------------------------------------------------------------
  // status bit counters
  //---------------------------------------------------------------
  // CRC error counter
  always @ (posedge clk or posedge rst)
    if (rst)
      crce_ctr <= 1'b0;
    else
      if (reg_crce)
        crce_ctr <= crce_ctr + 1;

  // frame error counter
  always @ (posedge clk or posedge rst)
    if (rst)
      frme_ctr <= 1'b0;
    else
      if (reg_frme)
        frme_ctr <= frme_ctr + 1;

  // packet dispatcher busy counter
  always @ (posedge clk or posedge rst)
    if (rst)
      busy_ctr <= 1'b0;
    else
      if (reg_busy)
        busy_ctr <= busy_ctr + 1;

  // packet dispatcher valid received frame counter
  always @ (posedge clk or posedge rst)
    if (rst)
      rfrm_ctr <= 1'b0;
    else
      if (reg_rfrm)
        rfrm_ctr <= rfrm_ctr + 1;

  // frame disassembler valid frame counter
  always @ (posedge clk or posedge rst)
    if (rst)
      dfrm_ctr <= 1'b0;
    else
      if (reg_dfrm)
        dfrm_ctr <= dfrm_ctr + 1;

  // frame transmitter frame counter
  always @ (posedge clk or posedge rst)
    if (rst)
      tfrm_ctr <= 1'b0;
    else
      if (reg_tfrm)
        tfrm_ctr <= tfrm_ctr + 1;

  // frame assembler valid sent frame counter
  always @ (posedge clk or posedge rst)
    if (rst)
      sfrm_ctr <= 1'b0;
    else
      if (reg_sfrm)
        sfrm_ctr <= sfrm_ctr + 1;

  // local nack'd frame counter
  always @ (posedge clk or posedge rst)
    if (rst)
      lnak_ctr <= 1'b0;
    else
      if (reg_lnak)
        lnak_ctr <= lnak_ctr + 1;

  // remote nack counter
  always @ (posedge clk or posedge rst)
    if (rst)
      rnak_ctr <= 1'b0;
    else
      if (reg_rnak)
        rnak_ctr <= rnak_ctr + 1;

  // local ack'd frame counter
  always @ (posedge clk or posedge rst)
    if (rst)
      lack_ctr <= 1'b0;
    else
      if (reg_lack)
        lack_ctr <= lack_ctr + 1;

  // remote ack counter
  always @ (posedge clk or posedge rst)
    if (rst)
      rack_ctr <= 1'b0;
    else
      if (reg_rack)
        rack_ctr <= rack_ctr + 1;

  // local out-of-credit counter
  always @ (posedge clk or posedge rst)
    if (rst)
      looc_ctr <= 1'b0;
    else
      if (reg_looc)
        looc_ctr <= looc_ctr + 1;

  // remote out-of-credit counter
  always @ (posedge clk or posedge rst)
    if (rst)
      rooc_ctr <= 1'b0;
    else
      if (reg_rooc)
        rooc_ctr <= rooc_ctr + 1;

  // Reconnection counter (count of handshake complete posedges)
  always @ (posedge clk or posedge rst)
    if (rst)
      reco_ctr <= 1'b0;
    else
      if (reg_reco)
        reco_ctr <= reco_ctr + 1;
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // register writing
  //---------------------------------------------------------------

  always @(posedge clk or posedge rst)
    if (rst)
      begin
        reg_idso <= {`IDLE_BITS{1'b0}};
        reg_stop <= 1'b0;
      end
    else
      if (reg_write)
        case (reg_addr)
          `IDSO_REG: reg_idso <= reg_write_data[`IDLE_BITS-1:0];
          `STOP_REG: reg_stop <= reg_write_data[0];
        endcase

  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // register output selection
  //---------------------------------------------------------------
  always @ (*)
    case (reg_addr)
      `VERS_REG: reg_read_data = {`HSS_MULTIPLEXER_VERSION,`PROTOCOL_VERSION};
      `CRCE_REG: reg_read_data = crce_ctr;
      `FRME_REG: reg_read_data = frme_ctr;
      `BUSY_REG: reg_read_data = busy_ctr;
      `LNAK_REG: reg_read_data = lnak_ctr;
      `RNAK_REG: reg_read_data = rnak_ctr;
      `LACK_REG: reg_read_data = lack_ctr;
      `RACK_REG: reg_read_data = rack_ctr;
      `LOOC_REG: reg_read_data = looc_ctr;
      `ROOC_REG: reg_read_data = rooc_ctr;
      `CRDT_REG: reg_read_data = {{(`REGD_BITS - `CRDT_BITS) {1'b0}},
                                    reg_crdt
                                  };  // not a counter!
      `SFRM_REG: reg_read_data = sfrm_ctr;
      `TFRM_REG: reg_read_data = tfrm_ctr;
      `DFRM_REG: reg_read_data = dfrm_ctr;
      `RFRM_REG: reg_read_data = rfrm_ctr;
      `EMPT_REG: reg_read_data = {{(`REGD_BITS - `NUM_CHANS) {1'b0}},
                                    reg_empt
                                  };  // not a counter!
      `FULL_REG: reg_read_data = {{(`REGD_BITS - `NUM_CHANS) {1'b0}},
                                    reg_full
                                  };  // not a counter!
      `CFCL_REG: reg_read_data = {{(`REGD_BITS - `NUM_CHANS) {1'b0}},
                                    reg_cfcl
                                  };  // not a counter!
      `CFCR_REG: reg_read_data = {{(`REGD_BITS - `NUM_CHANS) {1'b0}},
                               reg_cfcr
                             };  // not a counter!
      `IDSO_REG: reg_read_data = {{(`REGD_BITS-`IDLE_BITS){1'b0}}, reg_idso};
      `IDSI_REG: reg_read_data = {{(`REGD_BITS-`IDLE_BITS){1'b0}}, reg_idsi};
      `HAND_REG: reg_read_data = {{(`REGD_BITS-2){1'b0}}, reg_hand};
      `RECO_REG: reg_read_data = reco_ctr;
      `STOP_REG: reg_read_data = {{(`REGD_BITS-1){1'b0}}, reg_stop};
      default:   reg_read_data = {`REGD_BITS {1'b1}};
    endcase
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

endmodule
