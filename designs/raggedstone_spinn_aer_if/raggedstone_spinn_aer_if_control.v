// -------------------------------------------------------------------------
//  SpiNNaker <-> AER interface control module
//
// -------------------------------------------------------------------------
// AUTHOR
//  lap - luis.plana@manchester.ac.uk
//
// -------------------------------------------------------------------------
// COPYRIGHT
//  Copyright (c) The University of Manchester, 2012-2017.
//  SpiNNaker Project
//  Advanced Processor Technologies Group
//  School of Computer Science
// -------------------------------------------------------------------------
// TODO
// -------------------------------------------------------------------------

`include "raggedstone_spinn_aer_if_top.h"
`include "../../modules/spinnaker_link/spio_spinnaker_link.h"

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`timescale 1ns / 1ps
module raggedstone_spinn_aer_if_control
(
  input  wire                   rst,
  input  wire                   clk,

  // packet interface
  input  wire [`PKT_BITS - 1:0] cpkt_data,
  input  wire                   cpkt_vld,
  output reg                    cpkt_rdy,

  // control interface
  input  wire [`MODE_BITS - 1:0] ui_mode,
  input  wire   [`VC_BITS - 1:0] ui_vcoord,
  output reg  [`MODE_BITS - 1:0] ct_mode,
  output reg  [`VCRD_BITS - 1:0] vcoord,
  output reg                    go
);
  //---------------------------------------------------------------
  // packet interface 
  // generate cpkt_rdy signal
  //---------------------------------------------------------------
  always @(*)
    cpkt_rdy <= 1'b1;
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // mode selection
  //---------------------------------------------------------------
  always @(*)
    ct_mode = ui_mode;
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // virtual coord selection
  //---------------------------------------------------------------
  always @(*)
    case (ui_vcoord)
      `VC_ALT: vcoord = `VIRTUAL_COORD_ALT;
      default: vcoord = `VIRTUAL_COORD_DEF;
    endcase
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // generate go signal (initial value configurable)
  //---------------------------------------------------------------
  always @(posedge clk or posedge rst)
    if (rst)
      go <= `INIT_GO;
    else
      if (cpkt_vld)
        go <= cpkt_data[8];
endmodule
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
