// -------------------------------------------------------------------------
//  $Id: txc.v 2517 2013-08-19 09:33:30Z plana $
// spiNNlink transmitter control module
//
// -------------------------------------------------------------------------
// AUTHOR
//  lap - luis.plana@manchester.ac.uk
//
// -------------------------------------------------------------------------
// DETAILS
//  Created on       : 29 Nov 2012
//  Version          : $Revision: 2517 $
//  Last modified on : $Date: 2013-08-19 10:33:30 +0100 (Mon, 19 Aug 2013) $
//  Last modified by : $Author: plana $
//  $HeadURL: https://solem.cs.man.ac.uk/svn/spiNNlink/testing/src/txc.v $
//
// -------------------------------------------------------------------------
// COPYRIGHT
//  Copyright (c) The University of Manchester, 2012. All rights reserved.
//  SpiNNaker Project
//  Advanced Processor Technologies Group
//  School of Computer Science
// -------------------------------------------------------------------------
//
// TODO
// -------------------------------------------------------------------------


// ----------------------------------------------------------------
// include spiNNlink global constants and parameters
//
`include "spio_hss_multiplexer_common.h"
// ----------------------------------------------------------------

`timescale 1ns / 1ps
module txc
(
    input wire         clk,
    input wire         rst,

    input  wire  [1:0] rx_state,

    input  wire [31:0] tx_in_data,
    input  wire  [3:0] tx_in_kchr,
//#    input  wire        tx_in_vld,
    output reg         tx_in_rdy,

    output reg  [31:0] gtp_data,
    output reg   [3:0] gtp_kchr
);

  //---------------------------------------------------------------
  // constants
  //---------------------------------------------------------------
  localparam CNT_BITS = 9;
  localparam CNT_RNG = 8;
   

  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  reg [CNT_BITS - 1:0] count;


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //--------------------------- datapath --------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  always @ (posedge clk or posedge rst)
    if (rst)
      tx_in_rdy <= 1'b0;
    else
        tx_in_rdy <= (rx_state == 2'b11);

  always @ (posedge clk or posedge rst)
    if (rst)
    begin
      gtp_data <= 32'h00000000; // not really necessary!
      gtp_kchr <= 4'b0000;      // not really necessary!
    end
    else
      case (rx_state)
	2'b00:
          if (count[0] == 1'b1)
          begin
            gtp_data <= {`COMMA, 8'h00, `SYNC, count[1 +: CNT_RNG]};
            gtp_kchr <= 4'b1010;
          end
	  else
          begin
            gtp_data <= {`CLKC, `CLKC, `CLKC, `CLKC};
            gtp_kchr <= 4'b0001;
          end

        2'b01:
          if (count[0] == 1'b1)
          begin
            gtp_data <= {`COMMA, 8'h01, `SYNC, count[1 +: CNT_RNG]};
            gtp_kchr <= 4'b1010;
          end
          else
          begin
            gtp_data <= {`CLKC, `CLKC, `CLKC, `CLKC};
            gtp_kchr <= 4'b0001;
          end

        2'b10:
          if (count[0] == 1'b1)
          begin
            gtp_data <= {`COMMA, 8'h02, `SYNC, count[1 +: CNT_RNG]};
            gtp_kchr <= 4'b1010;
          end
          else
          begin
            gtp_data <= {`CLKC, `CLKC, `CLKC, `CLKC};
            gtp_kchr <= 4'b0001;
          end

        default:
          begin
            gtp_data <= tx_in_data;
            gtp_kchr <= tx_in_kchr;
          end
      endcase

  always @ (posedge clk or posedge rst)
    if (rst)
      count <= {CNT_BITS {1'b0}};
    else
      count <= count + 1;
  //---------------------------------------------------------------
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

endmodule
