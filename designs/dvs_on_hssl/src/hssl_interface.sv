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


`timescale 1ps/1ps
module hssl_interface
#(
    parameter PACKET_BITS  = 72,
    parameter NUM_CHANNELS = 8
)
(
  input  wire        clk,
  input  wire        reset,

  input  wire [PACKET_BITS - 1:0] tx_pkt_data_in  [NUM_CHANNELS - 1:0],
  input  wire                     tx_pkt_vld_in   [NUM_CHANNELS - 1:0],
  output wire                     tx_pkt_rdy_out  [NUM_CHANNELS - 1:0],

  output wire [PACKET_BITS - 1:0] rx_pkt_data_out [NUM_CHANNELS - 1:0],
  output wire                     rx_pkt_vld_out  [NUM_CHANNELS - 1:0],
  input  wire                     rx_pkt_rdy_in   [NUM_CHANNELS - 1:0],

  output wire        handshake_complete_out,
  output wire        version_mismatch_out,

  output wire [15:0] reg_idsi_out,
  input  wire        reg_stop_in,

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
      , .reg_stop   (reg_stop_in)

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
      , .pkt_data0  (tx_pkt_data_in[0])
      , .pkt_vld0   (tx_pkt_vld_in[0])
      , .pkt_rdy0   (tx_pkt_rdy_out[0])

      , .pkt_data1  (tx_pkt_data_in[1])
      , .pkt_vld1   (tx_pkt_vld_in[1])
      , .pkt_rdy1   (tx_pkt_rdy_out[1])

      , .pkt_data2  (tx_pkt_data_in[2])
      , .pkt_vld2   (tx_pkt_vld_in[2])
      , .pkt_rdy2   (tx_pkt_rdy_out[2])

      , .pkt_data3  (tx_pkt_data_in[3])
      , .pkt_vld3   (tx_pkt_vld_in[3])
      , .pkt_rdy3   (tx_pkt_rdy_out[3])

      , .pkt_data4  (tx_pkt_data_in[4])
      , .pkt_vld4   (tx_pkt_vld_in[4])
      , .pkt_rdy4   (tx_pkt_rdy_out[4])

      , .pkt_data5  (tx_pkt_data_in[5])
      , .pkt_vld5   (tx_pkt_vld_in[5])
      , .pkt_rdy5   (tx_pkt_rdy_out[5])

      , .pkt_data6  (tx_pkt_data_in[6])
      , .pkt_vld6   (tx_pkt_vld_in[6])
      , .pkt_rdy6   (tx_pkt_rdy_out[6])

      , .pkt_data7  (tx_pkt_data_in[7])
      , .pkt_vld7   (tx_pkt_vld_in[7])
      , .pkt_rdy7   (tx_pkt_rdy_out[7])

      // outgoing packet streams
      , .opkt_data0 (rx_pkt_data_out[0])
      , .opkt_vld0  (rx_pkt_vld_out[0])
      , .opkt_rdy0  (rx_pkt_rdy_in[0])

      , .opkt_data1 (rx_pkt_data_out[1])
      , .opkt_vld1  (rx_pkt_vld_out[1])
      , .opkt_rdy1  (rx_pkt_rdy_in[1])

      , .opkt_data2 (rx_pkt_data_out[2])
      , .opkt_vld2  (rx_pkt_vld_out[2])
      , .opkt_rdy2  (rx_pkt_rdy_in[2])

      , .opkt_data3 (rx_pkt_data_out[3])
      , .opkt_vld3  (rx_pkt_vld_out[3])
      , .opkt_rdy3  (rx_pkt_rdy_in[3])

      , .opkt_data4 (rx_pkt_data_out[4])
      , .opkt_vld4  (rx_pkt_vld_out[4])
      , .opkt_rdy4  (rx_pkt_rdy_in[4])

      , .opkt_data5 (rx_pkt_data_out[5])
      , .opkt_vld5  (rx_pkt_vld_out[5])
      , .opkt_rdy5  (rx_pkt_rdy_in[5])

      , .opkt_data6 (rx_pkt_data_out[6])
      , .opkt_vld6  (rx_pkt_vld_out[6])
      , .opkt_rdy6  (rx_pkt_rdy_in[6])

      , .opkt_data7 (rx_pkt_data_out[7])
      , .opkt_vld7  (rx_pkt_vld_out[7])
      , .opkt_rdy7  (rx_pkt_rdy_in[7])
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
