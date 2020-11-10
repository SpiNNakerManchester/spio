// -------------------------------------------------------------------------
//  dvs_on_hssl_top
//
//  DVS input to SpiNN-5 board through High-Speed Serial Link (HSSL)
//  top-level module: processor sub-system + hssl interface + GTH block
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
module dvs_on_hssl_top
#(
  // SpiNNaker link number (for back-pressure settings)
  parameter INTER_PACKET_DELAY = 32'd75000000
)
(
  // Differential reference clock inputs
  input  wire mgtrefclk0_x1y3_p,
  input  wire mgtrefclk0_x1y3_n,

  // GTH channel 0 HSSL data ports
  input  wire ch0_gthrxn_in,
  input  wire ch0_gthrxp_in,
  output wire ch0_gthtxn_out,
  output wire ch0_gthtxp_out
);

  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  // top-level clock and reset
  wire        pl_freerun_clk_int;
  wire  [0:0] pl_reset_all_int;

  // hssl interface block signals
  wire        hsslif_clk_int;
  wire        hsslif_reset_int;
  wire        handshake_complete_int;
  wire        version_mismatch_int;
  wire [15:0] reg_idsi_int;

  // GTH block signals
  wire  [0:0] gth_freerun_clk_int;
  wire  [0:0] gth_reset_all_int;

  wire  [0:0] gth_gtrefclk00_int;

  wire  [0:0] gth_qpll0outclk_int;
  wire  [0:0] gth_qpll0outrefclk_int;
  wire  [0:0] gth_gtpowergood_int;

  wire  [0:0] gth_userclk_tx_reset_int;
  wire  [0:0] gth_userclk_tx_srcclk_int;
  wire  [0:0] gth_userclk_tx_usrclk_int;
  wire  [0:0] gth_userclk_tx_usrclk2_int;
  wire  [0:0] gth_userclk_tx_active_int;

  wire  [0:0] gth_reset_tx_pll_and_datapath_int;
  wire  [0:0] gth_reset_tx_datapath_int;
  wire  [0:0] gth_reset_tx_done_int;
  wire  [0:0] gth_txpmaresetdone_int;

  wire [31:0] gth_userdata_tx_int;
  wire  [0:0] gth_tx8b10ben_int;
  wire [15:0] gth_txctrl0_int;
  wire [15:0] gth_txctrl1_int;
  wire  [7:0] gth_txctrl2_int;

  wire  [0:0] gth_userclk_rx_reset_int;
  wire  [0:0] gth_userclk_rx_srcclk_int;
  wire  [0:0] gth_userclk_rx_usrclk_int;
  wire  [0:0] gth_userclk_rx_usrclk2_int;
  wire  [0:0] gth_userclk_rx_active_int;

  wire  [0:0] gth_reset_rx_pll_and_datapath_int;
  wire  [0:0] gth_reset_rx_datapath_int;
  wire  [0:0] gth_reset_rx_done_int;
  wire  [0:0] gth_reset_rx_cdr_stable_int;
  wire  [0:0] gth_rxpmaresetdone_int;

  wire [31:0] gth_userdata_rx_int;
  wire  [0:0] gth_rx8b10ben_int;
  wire  [0:0] gth_rxbufreset_int;
  wire  [0:0] gth_rxcommadeten_int;
  wire  [0:0] gth_rxmcommaalignen_int;
  wire  [0:0] gth_rxpcommaalignen_int;
  wire  [2:0] gth_rxbufstatus_int;
  wire  [0:0] gth_rxbyteisaligned_int;
  wire  [0:0] gth_rxbyterealign_int;
  wire  [1:0] gth_rxclkcorcnt_int;
  wire  [0:0] gth_rxcommadet_int;
  wire [15:0] gth_rxctrl0_int;
  wire [15:0] gth_rxctrl1_int;
  wire  [7:0] gth_rxctrl2_int;
  wire  [7:0] gth_rxctrl3_int;

  // VIO control signals
  wire        vio_freerun_clk_int;
  wire        vio_reset_all_int;
  wire  [0:0] vio_reset_tx_pll_and_datapath_int;
  wire  [0:0] vio_reset_tx_datapath_int;
  wire  [0:0] vio_reset_rx_pll_and_datapath_int;
  wire  [0:0] vio_reset_rx_datapath_int;
  wire        vio_reset_link_down_latched_int;
  wire  [2:0] vio_loopback_int;
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // processor sub-system -
  // implements an AXI4-stream interface to the HSSL and
  // provides the free-running clock and the reset signal
  //---------------------------------------------------------------
  wire peripheral_reset_0_int;
  wire pl_clk0_int;

  proc_sys proc_sys_block (
      .peripheral_reset_0 (peripheral_reset_0_int)
    , .pl_clk0_0          (pl_clk0_int)
    );
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // generate clock for hssl interface and GTH block
  // derived from the PL clock provided by the processor sub-system
  //NOTE: avoids reconfiguration of the processor sub-system PL clock
  //---------------------------------------------------------------
  wire pl_clk0_buf_int;

  reg  clk_enable_int = 1'b0;

  //NOTE: this buffer may be redundant - used only for clk_enable_int
  BUFG fast_clk (
      .I (pl_clk0_int)
    , .O (pl_clk0_buf_int)
    );

  // toggle CE every clock cycle to divide p2pl_clk frequency by 2
  always @(posedge pl_clk0_buf_int)
    clk_enable_int <= ~clk_enable_int;

  //NOTE: pl_freerun_clk_int has 25% duty cycle / same pulse width as pl_clk0_int
  BUFGCE slow_clk (
      .I  (pl_clk0_int)
    , .CE (clk_enable_int)
    , .O  (pl_freerun_clk_int)
    );

  assign gth_freerun_clk_int[0:0] = pl_freerun_clk_int;

  assign vio_freerun_clk_int = pl_freerun_clk_int;

  assign hsslif_clk_int = gth_userclk_tx_usrclk2_int;
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // differential reference clock buffer for MGTREFCLK0_X1Y3
  //---------------------------------------------------------------
  wire mgtrefclk0_x1y3_int;

  IBUFDS_GTE4 #(
      .REFCLK_EN_TX_PATH  (1'b0)
    , .REFCLK_HROW_CK_SEL (2'b00)
    , .REFCLK_ICNTL_RX    (2'b00)
    ) 
  IBUFDS_GTE4_MGTREFCLK0_X1Y3_INST (
      .I     (mgtrefclk0_x1y3_p)
    , .IB    (mgtrefclk0_x1y3_n)
    , .CEB   (1'b0)
    , .O     (mgtrefclk0_x1y3_int)
    , .ODIV2 ()
    );

  assign gth_gtrefclk00_int[0] = mgtrefclk0_x1y3_int;
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // generate/buffer reset signals
  //---------------------------------------------------------------
  // buffer peripheral_reset_0_int
  wire peripheral_reset_0_buf_int;
  IBUF peripheral_reset_0_int_buffer (
      .I (peripheral_reset_0_int)
  	, .O (peripheral_reset_0_buf_int)
  	);

  // global and function-specific resets
  assign pl_reset_all_int = peripheral_reset_0_buf_int || vio_reset_all_int;

  assign gth_reset_all_int[0:0] = pl_reset_all_int;

  assign gth_reset_tx_pll_and_datapath_int[0:0] = vio_reset_tx_pll_and_datapath_int;
  assign gth_reset_tx_datapath_int[0:0]         = vio_reset_tx_datapath_int;

  assign gth_reset_rx_pll_and_datapath_int[0:0] = vio_reset_rx_pll_and_datapath_int;
  assign gth_reset_rx_datapath_int[0:0]         = vio_reset_rx_datapath_int;

  assign gth_userclk_tx_reset_int[0:0] = ~(&gth_txpmaresetdone_int);
  assign gth_userclk_rx_reset_int[0:0] = ~(&gth_rxpmaresetdone_int);

  assign hsslif_reset_int = !gth_reset_tx_done_int || !gth_userclk_tx_active_int;
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // HSSL signals
  //---------------------------------------------------------------
  wire [0:0] gth_rxn_int;
  assign gth_rxn_int[0:0] = ch0_gthrxn_in;

  wire [0:0] gth_rxp_int;
  assign gth_rxp_int[0:0] = ch0_gthrxp_in;

  wire [0:0] gth_txn_int;
  assign ch0_gthtxn_out = gth_txn_int[0:0];

  wire [0:0] gth_txp_int;
  assign ch0_gthtxp_out = gth_txp_int[0:0];
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // drive the tx HSSL interface
  //---------------------------------------------------------------
  wire          [31:0] tx_pkt0_key_int = 32'hdeed_cafe;
  wire          [31:0] tx_pkt0_pld_int = 32'h0000_0000;
  wire                 tx_pkt0_pty_int = ~(^tx_pkt0_key_int ^ ^tx_pkt0_pld_int);
  wire           [7:0] tx_pkt0_hdr_int = {7'b000_0000, tx_pkt0_pty_int};

  wire [`PKT_BITS-1:0] tx_pkt0_data_int =
    {tx_pkt0_pld_int, tx_pkt0_key_int, tx_pkt0_hdr_int};

  reg                  tx_pkt0_vld_int;
  wire                 tx_pkt0_rdy_int;

  reg           [31:0] tx_pkt0_vld_cnt_int;

  always @ (posedge hsslif_clk_int or posedge hsslif_reset_int)
    if (hsslif_reset_int)
      tx_pkt0_vld_cnt_int = INTER_PACKET_DELAY;
    else
    if ((tx_pkt0_vld_int == 1'b1) && (tx_pkt0_rdy_int == 1'b1))
        tx_pkt0_vld_cnt_int = INTER_PACKET_DELAY;
      else
        if ((tx_pkt0_vld_int == 1'b0) && (tx_pkt0_vld_cnt_int != 0))
          tx_pkt0_vld_cnt_int = tx_pkt0_vld_cnt_int - 1;

  always @ (posedge hsslif_clk_int or posedge hsslif_reset_int)
    if (hsslif_reset_int)
      tx_pkt0_vld_int = 1'b0;
    else
      if (tx_pkt0_vld_cnt_int == 0)
        tx_pkt0_vld_int = 1'b1;
      else
        tx_pkt0_vld_int = 1'b0;
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // HSSL interface
  //---------------------------------------------------------------
  hssl_interface hssl_interface_inst (
      .clk                            (hsslif_clk_int)
    , .reset                          (hsslif_reset_int)

    , .tx_pkt0_data_in                (tx_pkt0_data_int)
    , .tx_pkt0_vld_in                 (tx_pkt0_vld_int)
    , .tx_pkt0_rdy_out                (tx_pkt0_rdy_int)
    , .tx_pkt1_data_in                ()
    , .tx_pkt1_vld_in                 (1'b0)
    , .tx_pkt1_rdy_out                ()
    , .tx_pkt2_data_in                ()
    , .tx_pkt2_vld_in                 (1'b0)
    , .tx_pkt2_rdy_out                ()
    , .tx_pkt3_data_in                ()
    , .tx_pkt3_vld_in                 (1'b0)
    , .tx_pkt3_rdy_out                ()
    , .tx_pkt4_data_in                ()
    , .tx_pkt4_vld_in                 (1'b0)
    , .tx_pkt4_rdy_out                ()
    , .tx_pkt5_data_in                ()
    , .tx_pkt5_vld_in                 (1'b0)
    , .tx_pkt5_rdy_out                ()
    , .tx_pkt6_data_in                ()
    , .tx_pkt6_vld_in                 (1'b0)
    , .tx_pkt6_rdy_out                ()
    , .tx_pkt7_data_in                ()
    , .tx_pkt7_vld_in                 (1'b0)
    , .tx_pkt7_rdy_out                ()

    , .rx_pkt0_data_out               ()
    , .rx_pkt0_vld_out                ()
    , .rx_pkt0_rdy_in                 (1'b0)
    , .rx_pkt1_data_out               ()
    , .rx_pkt1_vld_out                ()
    , .rx_pkt1_rdy_in                 (1'b0)
    , .rx_pkt2_data_out               ()
    , .rx_pkt2_vld_out                ()
    , .rx_pkt2_rdy_in                 (1'b0)
    , .rx_pkt3_data_out               ()
    , .rx_pkt3_vld_out                ()
    , .rx_pkt3_rdy_in                 (1'b0)
    , .rx_pkt4_data_out               ()
    , .rx_pkt4_vld_out                ()
    , .rx_pkt4_rdy_in                 (1'b0)
    , .rx_pkt5_data_out               ()
    , .rx_pkt5_vld_out                ()
    , .rx_pkt5_rdy_in                 (1'b0)
    , .rx_pkt6_data_out               ()
    , .rx_pkt6_vld_out                ()
    , .rx_pkt6_rdy_in                 (1'b0)
    , .rx_pkt7_data_out               ()
    , .rx_pkt7_vld_out                ()
    , .rx_pkt7_rdy_in                 (1'b0)

    , .handshake_complete_out         (handshake_complete_int)
    , .version_mismatch_out           (version_mismatch_int)
    , .reg_idsi_out                   (reg_idsi_int)

    , .userdata_tx_out                (gth_userdata_tx_int)
    , .tx8b10ben_out                  (gth_tx8b10ben_int)
    , .txctrl0_out                    (gth_txctrl0_int)
    , .txctrl1_out                    (gth_txctrl1_int)
    , .txctrl2_out                    (gth_txctrl2_int)

    , .userdata_rx_in                 (gth_userdata_rx_int)
    , .rx8b10ben_out                  (gth_rx8b10ben_int)
    , .rxbufreset_out                 (gth_rxbufreset_int)
    , .rxcommadeten_out               (gth_rxcommadeten_int)
    , .rxmcommaalignen_out            (gth_rxmcommaalignen_int)
    , .rxpcommaalignen_out            (gth_rxpcommaalignen_int)
    , .rxbufstatus_in                 (gth_rxbufstatus_int)
    , .rxbyteisaligned_in             (gth_rxbyteisaligned_int)
    , .rxbyterealign_in               (gth_rxbyterealign_int)
    , .rxclkcorcnt_in                 (gth_rxclkcorcnt_int)
    , .rxcommadet_in                  (gth_rxcommadet_int)
    , .rxctrl0_in                     (gth_rxctrl0_int)
    , .rxctrl1_in                     (gth_rxctrl1_int)
    , .rxctrl2_in                     (gth_rxctrl2_int)
    , .rxctrl3_in                     (gth_rxctrl3_int)
    );
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // GTH transceiver and tx and rx user clock helper blocks
  //---------------------------------------------------------------
  gth_x1y11_3Gbs_example_wrapper example_wrapper_inst (
      .gtwiz_reset_clk_freerun_in              (gth_freerun_clk_int)
    , .gtwiz_reset_all_in                      (gth_reset_all_int)

    , .gtrefclk00_in                           (gth_gtrefclk00_int)

    , .qpll0outclk_out                         (gth_qpll0outclk_int)
    , .qpll0outrefclk_out                      (gth_qpll0outrefclk_int)
    , .gtpowergood_out                         (gth_gtpowergood_int)

    , .loopback_in                             (vio_loopback_int)

    , .gthrxn_in                               (gth_rxn_int)
    , .gthrxp_in                               (gth_rxp_int)
    , .gthtxn_out                              (gth_txn_int)
    , .gthtxp_out                              (gth_txp_int)

    , .gtwiz_userclk_tx_reset_in               (gth_userclk_tx_reset_int)
    , .gtwiz_userclk_tx_srcclk_out             (gth_userclk_tx_srcclk_int)
    , .gtwiz_userclk_tx_usrclk_out             (gth_userclk_tx_usrclk_int)
    , .gtwiz_userclk_tx_usrclk2_out            (gth_userclk_tx_usrclk2_int)
    , .gtwiz_userclk_tx_active_out             (gth_userclk_tx_active_int)

    , .gtwiz_userclk_rx_reset_in               (gth_userclk_rx_reset_int)
    , .gtwiz_userclk_rx_srcclk_out             (gth_userclk_rx_srcclk_int)
    , .gtwiz_userclk_rx_usrclk_out             (gth_userclk_rx_usrclk_int)
    , .gtwiz_userclk_rx_usrclk2_out            (gth_userclk_rx_usrclk2_int)
    , .gtwiz_userclk_rx_active_out             (gth_userclk_rx_active_int)

    , .gtwiz_reset_tx_pll_and_datapath_in      (gth_reset_tx_pll_and_datapath_int)
    , .gtwiz_reset_tx_datapath_in              (gth_reset_tx_datapath_int)
    , .gtwiz_reset_tx_done_out                 (gth_reset_tx_done_int)
    , .txpmaresetdone_out                      (gth_txpmaresetdone_int)

    , .gtwiz_reset_rx_pll_and_datapath_in      (gth_reset_rx_pll_and_datapath_int)
    , .gtwiz_reset_rx_datapath_in              (gth_reset_rx_datapath_int)
    , .gtwiz_reset_rx_done_out                 (gth_reset_rx_done_int)
    , .gtwiz_reset_rx_cdr_stable_out           (gth_reset_rx_cdr_stable_int)
    , .rxpmaresetdone_out                      (gth_rxpmaresetdone_int)

    , .gtwiz_userdata_tx_in                    (gth_userdata_tx_int)
    , .gtwiz_userdata_rx_out                   (gth_userdata_rx_int)

    , .tx8b10ben_in                            (gth_tx8b10ben_int)
    , .txctrl0_in                              (gth_txctrl0_int)
    , .txctrl1_in                              (gth_txctrl1_int)
    , .txctrl2_in                              (gth_txctrl2_int)

    , .rx8b10ben_in                            (gth_rx8b10ben_int)
    , .rxbufreset_in                           (gth_rxbufreset_int)
    , .rxcommadeten_in                         (gth_rxcommadeten_int)
    , .rxmcommaalignen_in                      (gth_rxmcommaalignen_int)
    , .rxpcommaalignen_in                      (gth_rxpcommaalignen_int)
    , .rxbufstatus_out                         (gth_rxbufstatus_int)
    , .rxbyteisaligned_out                     (gth_rxbyteisaligned_int)
    , .rxbyterealign_out                       (gth_rxbyterealign_int)
    , .rxclkcorcnt_out                         (gth_rxclkcorcnt_int)
    , .rxcommadet_out                          (gth_rxcommadet_int)
    , .rxctrl0_out                             (gth_rxctrl0_int)
    , .rxctrl1_out                             (gth_rxctrl1_int)
    , .rxctrl2_out                             (gth_rxctrl2_int)
    , .rxctrl3_out                             (gth_rxctrl3_int)
    );
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // virtual I/O for HSSL interface and GTH block
  //---------------------------------------------------------------
  hssl_vio hssl_vio_inst (
      .clk              (vio_freerun_clk_int)

      // HSSL interface probes
    , .probe_in0        ()
    , .probe_in1        (handshake_complete_int)
    , .probe_in2        (version_mismatch_int)
    , .probe_in3        (reg_idsi_int[3:0])

      // GTH block probes
    , .probe_in4        (gth_gtpowergood_int)
    , .probe_in5        (gth_txpmaresetdone_int)
    , .probe_in6        (gth_rxpmaresetdone_int)
    , .probe_in7        (gth_reset_tx_done_int)
    , .probe_in8        (gth_reset_rx_done_int)
    , .probe_in9        (gth_rxbufstatus_int)
    , .probe_in10       (gth_rxctrl0_int)
    , .probe_in11       (gth_rxctrl2_int)
    , .probe_in12       (gth_userdata_rx_int)

      // VIO control signals
    , .probe_out0       (vio_reset_all_int)
    , .probe_out1       (vio_reset_tx_pll_and_datapath_int)
    , .probe_out2       (vio_reset_tx_datapath_int)
    , .probe_out3       (vio_reset_rx_pll_and_datapath_int)
    , .probe_out4       (vio_reset_rx_datapath_int)
    , .probe_out5       (vio_reset_link_down_latched_int)
    , .probe_out6       (vio_loopback_int)
    );
  //---------------------------------------------------------------
endmodule
