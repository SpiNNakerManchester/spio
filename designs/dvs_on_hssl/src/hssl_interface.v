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
//  Last modified on : Mon  9 Nov 08:54:58 GMT 2020
//  Last modified by : lap
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

`include "spio_hss_multiplexer_common.h"

`timescale 1ps/1ps
module hssl_interface (
  input  wire        clk,
  input  wire        reset,

  input  wire [`PKT_BITS-1:0] tx_pkt0_data_in,
  input  wire                 tx_pkt0_vld_in,
  output wire                 tx_pkt0_rdy_out,
  input  wire [`PKT_BITS-1:0] tx_pkt1_data_in,
  input  wire                 tx_pkt1_vld_in,
  output wire                 tx_pkt1_rdy_out,
  input  wire [`PKT_BITS-1:0] tx_pkt2_data_in,
  input  wire                 tx_pkt2_vld_in,
  output wire                 tx_pkt2_rdy_out,
  input  wire [`PKT_BITS-1:0] tx_pkt3_data_in,
  input  wire                 tx_pkt3_vld_in,
  output wire                 tx_pkt3_rdy_out,
  input  wire [`PKT_BITS-1:0] tx_pkt4_data_in,
  input  wire                 tx_pkt4_vld_in,
  output wire                 tx_pkt4_rdy_out,
  input  wire [`PKT_BITS-1:0] tx_pkt5_data_in,
  input  wire                 tx_pkt5_vld_in,
  output wire                 tx_pkt5_rdy_out,
  input  wire [`PKT_BITS-1:0] tx_pkt6_data_in,
  input  wire                 tx_pkt6_vld_in,
  output wire                 tx_pkt6_rdy_out,
  input  wire [`PKT_BITS-1:0] tx_pkt7_data_in,
  input  wire                 tx_pkt7_vld_in,
  output wire                 tx_pkt7_rdy_out,

  output wire [`PKT_BITS-1:0] rx_pkt0_data_out,
  output wire                 rx_pkt0_vld_out,
  input  wire                 rx_pkt0_rdy_in,
  output wire [`PKT_BITS-1:0] rx_pkt1_data_out,
  output wire                 rx_pkt1_vld_out,
  input  wire                 rx_pkt1_rdy_in,
  output wire [`PKT_BITS-1:0] rx_pkt2_data_out,
  output wire                 rx_pkt2_vld_out,
  input  wire                 rx_pkt2_rdy_in,
  output wire [`PKT_BITS-1:0] rx_pkt3_data_out,
  output wire                 rx_pkt3_vld_out,
  input  wire                 rx_pkt3_rdy_in,
  output wire [`PKT_BITS-1:0] rx_pkt4_data_out,
  output wire                 rx_pkt4_vld_out,
  input  wire                 rx_pkt4_rdy_in,
  output wire [`PKT_BITS-1:0] rx_pkt5_data_out,
  output wire                 rx_pkt5_vld_out,
  input  wire                 rx_pkt5_rdy_in,
  output wire [`PKT_BITS-1:0] rx_pkt6_data_out,
  output wire                 rx_pkt6_vld_out,
  input  wire                 rx_pkt6_rdy_in,
  output wire [`PKT_BITS-1:0] rx_pkt7_data_out,
  output wire                 rx_pkt7_vld_out,
  input  wire                 rx_pkt7_rdy_in,

  output wire        handshake_complete_out,
  output wire        version_mismatch_out,
  output wire [15:0] reg_idsi_out,

  output wire [31:0] userdata_tx_out,
  output wire  [0:0] tx8b10ben_out,
  output wire [15:0] txctrl0_out,
  output wire [15:0] txctrl1_out,
  output wire  [7:0] txctrl2_out,

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
  // hssl tx and rx signals
  wire [31:0] txdata_int;
  wire  [3:0] txcharisk_int;
  wire        txvld_int;
  wire        txrdy_int;

  wire [31:0] rxdata_int;
  wire  [3:0] rxcharisk_int;
  wire        rxen_int;

  // link state signals
  wire handshake_phase_int;
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // spiNNlink multiplexer -
  // tx side assembles and transmits spiNNlink frames
  // rx side receives and disassembles spiNNlink frames 
  //---------------------------------------------------------------
  spio_hss_multiplexer_spinnlink
    spio_hss_multiplexer_spinnlink_i( 
        .clk        (clk)
      , .rst        (reset)

      // diagnostic signals from frame assembler
      , .reg_sfrm   ()
      , .reg_looc   ()
      , .reg_crdt   ()
      , .reg_empt   ()
      , .reg_full   ()

      // diagnostic/control signals from/to frame transmitter
      , .reg_tfrm   ()
      , .reg_stop   (1'b0)

      // diagnostic signals from frame disassembler
      , .reg_dfrm   ()
      , .reg_crce   ()
      , .reg_frme   ()
      , .reg_rnak   ()
      , .reg_rack   ()
      , .reg_rooc   ()
      , .reg_cfcr   ()

      // diagnostic signals from packet dispatcher
      , .reg_rfrm   ()
      , .reg_busy   ()
      , .reg_lnak   ()
      , .reg_lack   ()
      , .reg_cfcl   ()

      // to high-speed serial: assembled frames out
      , .hsl_data   (txdata_int)
      , .hsl_kchr   (txcharisk_int)
      , .hsl_vld    (txvld_int)
      , .hsl_rdy    (txrdy_int)

      // from high-speed serial: assembled frames in
      , .ihsl_data  (rxdata_int)
      , .ihsl_kchr  (rxcharisk_int)
      , .ihsl_vld   (rxen_int)

      // incoming packet streams
      , .pkt_data0  (tx_pkt0_data_in)
      , .pkt_vld0   (tx_pkt0_vld_in)
      , .pkt_rdy0   (tx_pkt0_rdy_out)

      , .pkt_data1  (tx_pkt1_data_in)
      , .pkt_vld1   (tx_pkt1_vld_in)
      , .pkt_rdy1   (tx_pkt1_rdy_out)

      , .pkt_data2  (tx_pkt2_data_in)
      , .pkt_vld2   (tx_pkt2_vld_in)
      , .pkt_rdy2   (tx_pkt2_rdy_out)

      , .pkt_data3  (tx_pkt3_data_in)
      , .pkt_vld3   (tx_pkt3_vld_in)
      , .pkt_rdy3   (tx_pkt3_rdy_out)

      , .pkt_data4  (tx_pkt4_data_in)
      , .pkt_vld4   (tx_pkt4_vld_in)
      , .pkt_rdy4   (tx_pkt4_rdy_out)

      , .pkt_data5  (tx_pkt5_data_in)
      , .pkt_vld5   (tx_pkt5_vld_in)
      , .pkt_rdy5   (tx_pkt5_rdy_out)

      , .pkt_data6  (tx_pkt6_data_in)
      , .pkt_vld6   (tx_pkt6_vld_in)
      , .pkt_rdy6   (tx_pkt6_rdy_out)

      , .pkt_data7  (tx_pkt7_data_in)
      , .pkt_vld7   (tx_pkt7_vld_in)
      , .pkt_rdy7   (tx_pkt7_rdy_out)

      // outgoing packet streams
      , .opkt_data0 (rx_pkt0_data_out)
      , .opkt_vld0  (rx_pkt0_vld_out)
      , .opkt_rdy0  (rx_pkt0_rdy_in)

      , .opkt_data1 (rx_pkt1_data_out)
      , .opkt_vld1  (rx_pkt1_vld_out)
      , .opkt_rdy1  (rx_pkt1_rdy_in)

      , .opkt_data2 (rx_pkt2_data_out)
      , .opkt_vld2  (rx_pkt2_vld_out)
      , .opkt_rdy2  (rx_pkt2_rdy_in)

      , .opkt_data3 (rx_pkt3_data_out)
      , .opkt_vld3  (rx_pkt3_vld_out)
      , .opkt_rdy3  (rx_pkt3_rdy_in)

      , .opkt_data4 (rx_pkt4_data_out)
      , .opkt_vld4  (rx_pkt4_vld_out)
      , .opkt_rdy4  (rx_pkt4_rdy_in)

      , .opkt_data5 (rx_pkt5_data_out)
      , .opkt_vld5  (rx_pkt5_vld_out)
      , .opkt_rdy5  (rx_pkt5_rdy_in)

      , .opkt_data6 (rx_pkt6_data_out)
      , .opkt_vld6  (rx_pkt6_vld_out)
      , .opkt_rdy6  (rx_pkt6_rdy_in)

      , .opkt_data7 (rx_pkt7_data_out)
      , .opkt_vld7  (rx_pkt7_vld_out)
      , .opkt_rdy7  (rx_pkt7_rdy_in)
    );
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
  wire  [3:0] tx_char_is_k_int;

  spio_hss_multiplexer_tx_control
  spio_hss_multiplexer_tx_control_i (
      .CLK_IN                 (clk)
    , .RESET_IN               (reset)

    , .SCRMBL_IDL_DAT         (1'b0)
    , .REG_IDSO_IN            (16'hdead)

    , .HANDSHAKE_COMPLETE_IN  (handshake_complete_out)
    , .HANDSHAKE_PHASE_IN     (handshake_phase_int)

      // GTH tx data interface
    , .TXDATA_OUT             (userdata_tx_out)
    , .TXCHARISK_OUT          (tx_char_is_k_int)

      // incoming frame interface
    , .TXDATA_IN              (txdata_int)
    , .TXCHARISK_IN           (txcharisk_int)
    , .TXVLD_IN               (txvld_int)
    , .TXRDY_OUT              (txrdy_int)
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
      .CLK_IN                 (clk)
    , .RESET_IN               (reset)

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
    , .RXDATA_OUT             (rxdata_int)
    , .RXCHARISK_OUT          (rxcharisk_int)
    , .RXVLD_OUT              (rxen_int)
    );
  //---------------------------------------------------------------
endmodule
