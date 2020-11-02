// -------------------------------------------------------------------------
//  dvs_on_hssl_top_tb
//
//  DVS input to SpiNN-5 board through High-Speed Serial Link (HSSL)
//  testbench for simulation
//
// -------------------------------------------------------------------------
// AUTHOR
//  lap - luis.plana@manchester.ac.uk
// -------------------------------------------------------------------------
// DETAILS
//  Created on       : 21 Oct 2020
//  Last modified on : Wed 21 Oct 16:13:42 BST 2020
//  Last modified by : $Author: plana $
// -------------------------------------------------------------------------
// COPYRIGHT
//  Copyright (c) The University of Manchester, 2020.
//  SpiNNaker Project
//  Advanced Processor Technologies Group
//  School of Computer Science
// -------------------------------------------------------------------------
// TODO
//  * everything
// -------------------------------------------------------------------------

`timescale 1ps/1ps
module dvs_on_hssl_top_tb ();

  //---------------------------------------------------------------
  // testbench constants
  //---------------------------------------------------------------
  // GTH MGT refence clock period
  localparam MGT_CLK_PERIOD = 6734;  // picoseconds
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // testbench signals
  //---------------------------------------------------------------
  // GTH differential clocks
  reg  mgtrefclk0_x1y3;

  // HSSL loop back
  wire ch0_gthxn;
  wire ch0_gthxp;
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  //provide differential clocks to GTH block
  //NOTE: simulation only!
  //---------------------------------------------------------------
  initial begin
    mgtrefclk0_x1y3 = 1'b0;
    forever
      mgtrefclk0_x1y3 = #(MGT_CLK_PERIOD / 2) ~mgtrefclk0_x1y3;
  end
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // processor sub-system + hssl interface + GTH block
  //---------------------------------------------------------------
  dvs_on_hssl_top dvs_on_hssl (
    .mgtrefclk0_x1y3_p (mgtrefclk0_x1y3),
    .mgtrefclk0_x1y3_n (~mgtrefclk0_x1y3),

    .ch0_gthrxn_in     (ch0_gthxn),
    .ch0_gthrxp_in     (ch0_gthxp),
    .ch0_gthtxn_out    (ch0_gthxn),
    .ch0_gthtxp_out    (ch0_gthxp)
    );
  //---------------------------------------------------------------

endmodule
