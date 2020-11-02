// -------------------------------------------------------------------------
//  hssl_interface
//
//  DVS input to SpiNN-5 board through High-Speed Serial Link (HSSL)
//  interface to the GTH transceiver
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
module hssl_interface (
  input  wire        hsslif_clk,
  input  wire        hsslif_reset,

  output wire        handshake_complete_out,
  output wire        version_mismatch_out,
  output wire [15:0] reg_idsi_out,

  input  wire  [0:0] qpll0outclk_in,
  input  wire  [0:0] qpll0outrefclk_in,
  input  wire  [0:0] gtpowergood_in,

  input  wire  [0:0] userclk_tx_usrclk_in,
  input  wire  [0:0] userclk_tx_usrclk2_in,
  input  wire  [0:0] userclk_tx_active_in,

  input  wire  [0:0] reset_tx_done_in,
  input  wire  [0:0] txpmaresetdone_in,

  output wire [31:0] userdata_tx_out,
  output wire  [0:0] tx8b10ben_out,
  output wire [15:0] txctrl0_out,
  output wire [15:0] txctrl1_out,
  output wire  [7:0] txctrl2_out,

  input  wire  [0:0] userclk_rx_usrclk_in,
  input  wire  [0:0] userclk_rx_usrclk2_in,
  input  wire  [0:0] userclk_rx_active_in,

  input  wire  [0:0] reset_rx_done_in,
  input  wire  [0:0] reset_rx_cdr_stable_in,
  input  wire  [0:0] rxpmaresetdone_in,

  input  wire [31:0] userdata_rx_in,
  output wire  [0:0] rx8b10ben_out,
  output wire  [0:0] rxbufreset_out,
  output wire  [0:0] rxcommadeten_out,
  output wire  [0:0] rxmcommaalignen_out,
  output wire  [0:0] rxpcommaalignen_out,
  input  wire  [2:0] rxbufstatus_in,
  input  wire  [0:0] rxbyteisaligned_in,
  input  wire  [0:0] rxbyterealign_in,
  input  wire  [1:0] rxclkcorcnt_in,
  input  wire  [0:0] rxcommadet_in,
  input  wire [15:0] rxctrl0_in,
  input  wire [15:0] rxctrl1_in,
  input  wire  [7:0] rxctrl2_in,
  input  wire  [7:0] rxctrl3_in
);

  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  wire gth_side_tx_reset = !reset_tx_done_in || !userclk_tx_active_in;
  wire gth_side_tx_clk   = userclk_tx_usrclk2_in;

  wire gth_side_rx_reset = !reset_rx_done_in || !userclk_rx_active_in;
  wire gth_side_rx_clk   = userclk_rx_usrclk2_in;

  wire handshake_phase_int;
  wire handshake_phase_sync;
  //---------------------------------------------------------------

  //---------------------------------------------------------------
  // GTH configuration
  //---------------------------------------------------------------
  assign tx8b10ben_out[0:0] = 1'b1;

  assign rx8b10ben_out[0:0]       = 1'b1;
  assign rxbufreset_out[0:0]      = 1'b0;
  assign rxcommadeten_out[0:0]    = 1'b1;
  assign rxmcommaalignen_out[0:0] = 1'b1;
  assign rxpcommaalignen_out[0:0] = 1'b1;
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // drive GTH tx data
  //---------------------------------------------------------------
  // synchronise signals from GTH rx side of interface
  (* DONT_TOUCH = "TRUE" *)
  gth_x1y11_3Gbs_example_bit_synchronizer bit_synchronizer_handshake_phase_inst (
    .clk_in (gth_side_tx_clk),
    .i_in   (handshake_phase_int),
    .o_out  (handshake_phase_sync)
    );

  wire  [3:0] tx_char_is_k_int;

  spio_hss_multiplexer_tx_control
  spio_hss_multiplexer_tx_control_i (
      .CLK_IN                 (gth_side_tx_clk)
    , .RESET_IN               (gth_side_tx_reset)

    , .SCRMBL_IDL_DAT         (1'b0)
    , .REG_IDSO_IN            (16'hdead)

    , .HANDSHAKE_COMPLETE_IN  (handshake_complete_out)
    , .HANDSHAKE_PHASE_IN     (handshake_phase_sync)

      // GTH tx data interface
    , .TXDATA_OUT             (userdata_tx_out)
    , .TXCHARISK_OUT          (tx_char_is_k_int)

      // incoming frame interface
    , .TXDATA_IN              ()
    , .TXCHARISK_IN           ()
    , .TXVLD_IN               (1'b0)
    , .TXRDY_OUT              ()
    );

  assign txctrl0_out = 16'h0000;
  assign txctrl1_out = 16'h0000;
  assign txctrl2_out = {4'h0, tx_char_is_k_int};
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // process GTH rx data
  //---------------------------------------------------------------
  spio_hss_multiplexer_rx_control #()
  spio_hss_multiplexer_rx_control_i (
      .CLK_IN                 (gth_side_rx_clk)
    , .RESET_IN               (gth_side_rx_reset)

    , .REG_IDSI_OUT           (reg_idsi_out)

    , .HANDSHAKE_COMPLETE_OUT (handshake_complete_out)
    , .HANDSHAKE_PHASE_OUT    (handshake_phase_int)
    , .VERSION_MISMATCH_OUT   (version_mismatch_out)

      // GTH rx data interface
    , .RXDATA_IN              (userdata_rx_in)
    , .RXCHARISCOMMA_IN       (rxctrl2_in[3:0])
    , .RXLOSSOFSYNC_IN        (2'b01)
    , .RXCHARISK_IN           (rxctrl0_in[3:0])

    // outgoing frame interface
    , .RXDATA_OUT             ()
    , .RXCHARISK_OUT          ()
    , .RXVLD_OUT              ()
    );
  //---------------------------------------------------------------
endmodule
