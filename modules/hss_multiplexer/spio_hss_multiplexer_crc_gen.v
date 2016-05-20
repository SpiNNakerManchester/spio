// -------------------------------------------------------------------------
// $Id: crc_gen.v 2098 2013-01-19 11:38:48Z plana $
//  spiNNlink CRC generation module
//
// -------------------------------------------------------------------------
// AUTHOR
//  lap - luis.plana@manchester.ac.uk
//  Based on work by J Pepper (Date 07/08/2012)
//
// -------------------------------------------------------------------------
// DETAILS
//  Created on       : 28 Nov 2012
//  Version          : $Revision: 2098 $
//  Last modified on : $Date: 2013-01-19 11:38:48 +0000 (Sat, 19 Jan 2013) $
//  Last modified by : $Author: plana $
//  $HeadURL: https://solem.cs.man.ac.uk/svn/spiNNlink/testing/src/crc_gen.v $
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


// -------------------------------------------------------------------------
// include spiNNlink global constants and parameters
//
`include "spio_hss_multiplexer_common.h"
// -------------------------------------------------------------------------

`timescale 1ns / 1ps
module spio_hss_multiplexer_crc_gen
(
  input wire 	    clk,
  input wire 	    rst,

  // CRC data interface
  input wire 	    crc_go,
  input wire 	    crc_last,
  input wire [31:0] crc_in,
  output reg [31:0] crc_out
);

  //---------------------------------------------------------------
  // constants
  //---------------------------------------------------------------
  // NOTE: standard practice is to reset checksum to all 1s
  localparam CHKSUM_RST = {16 {1'b1}};

  //---------------------------------------------------------------
  // internal variables
  //---------------------------------------------------------------
  reg [15:0] new_chksum;
  reg [15:0] chksum;


  //---------------------------------------------------------------
  // functions
  //---------------------------------------------------------------
`include "CRC16_D32.v"   // function nextCRC16_D32 (Data, crc)
   

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //--------------------------- datapath --------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //---------------------------------------------------------------
  // new checksum (combinatorial)
  //---------------------------------------------------------------
  always @(*)
//#    new_chksum = nextCRC16_D32 (.Data (crc_in), .crc (chksum));
    new_chksum = nextCRC16_D32 (crc_in, chksum);
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // crc output (combinatorial)
  //---------------------------------------------------------------
  always @(*)
     if (crc_last && crc_go)
       // substitute crc in last flit of every frame
       crc_out = {crc_in[31:16], new_chksum};
     else
       crc_out = crc_in;
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // keep new checksum for next clock cycle
  //---------------------------------------------------------------
  always @(posedge clk or posedge rst)
     if (rst)
       chksum <= CHKSUM_RST;
     else
       if (crc_go)
         if (crc_last)
           chksum <= CHKSUM_RST;
         else
           chksum <= new_chksum;
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
endmodule
