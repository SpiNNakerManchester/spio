// -------------------------------------------------------------------------
//  dvs_on_hssl_top
//
//  DVS input to SpiNN-5 board through High-Speed Serial Link (HSSL)
//  top-level module: processor subsystem + hssl interface + GTH block
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
  parameter PACKET_BITS  = `PKT_BITS,
  parameter NUM_CHANNELS = 8
)
(
  // differential reference clock inputs
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

  // processor subsystem interface
  wire [31:0] inter_pkt_delay_int;
  wire        axi_clk_int;
  wire        axi_resetn_int;

  wire [39:0] apb_paddr_int;
  wire        apb_penable_int;
  wire [31:0] apb_prdata_int;
  wire  [0:0] apb_pready_int;
  wire  [0:0] apb_psel_int;
  wire  [0:0] apb_pslverr_int;
  wire [31:0] apb_pwdata_int;
  wire        apb_pwrite_int;

  // register bank - routing table
  wire [31:0] reg_bank_int  [15:0];
  wire [31:0] reg_key_int   [15:0];
  wire [31:0] reg_mask_int  [15:0];
  wire  [2:0] reg_route_int [15:0];

  // hssl interface block signals
  wire        hsslif_clk_int;
  wire        hsslif_reset_int;
  wire  [0:0] hsslif_control_int;

  wire [31:0] evt_data_int;
  wire        evt_vld_int;
  wire        evt_rdy_int;

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
  // processor subsystem -
  // implements an AXI4-stream interface to the HSSL and
  // provides the free-running clock and the reset signal
  //---------------------------------------------------------------
  wire peripheral_reset_0_int;
  wire pl_clk0_int;

  proc_sys proc_sys_block (
      .peripheral_reset_0       (peripheral_reset_0_int)
    , .pl_clk0_0                (pl_clk0_int)

    , .s_axi_aresetn_0          (axi_resetn_int)
    , .s_axi_aclk_0             (axi_clk_int)

    , .GPIO_0_tri_o             (hsslif_control_int)
    , .GPIO2_0_tri_o            (inter_pkt_delay_int)

    , .APB_M_0_paddr            (apb_paddr_int)
    , .APB_M_0_penable          (apb_penable_int)
    , .APB_M_0_prdata           (apb_prdata_int)
    , .APB_M_0_pready           (apb_pready_int)
    , .APB_M_0_psel             (apb_psel_int)
    , .APB_M_0_pslverr          (apb_pslverr_int)
    , .APB_M_0_pwdata           (apb_pwdata_int)
    , .APB_M_0_pwrite           (apb_pwrite_int)

    , .AXI_STR_TXD_0_tdata      (evt_data_int)
    , .AXI_STR_TXD_0_tlast      ()
    , .AXI_STR_TXD_0_tvalid     (evt_vld_int)
    , .AXI_STR_TXD_0_tready     (evt_rdy_int)
    , .mm2s_prmry_reset_out_n_0 ()
    );
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // register bank (APB peripheral)
  //---------------------------------------------------------------
  hssl_reg_bank rb
    (
      .clk             (axi_clk_int)
    , .resetn          (axi_resetn_int)

    , .apb_psel_in     (apb_psel_int)
    , .apb_pwrite_in   (apb_pwrite_int)
    , .apb_penable_in  (apb_penable_int)

    , .apb_paddr_in    (apb_paddr_int)
    , .apb_pwdata_in   (apb_pwdata_int)
    , .apb_prdata_out  (apb_prdata_int)

    , .apb_pready_out  (apb_pready_int)
    , .apb_pslverr_out (apb_pslverr_int)

    , .reg_bank_out    (reg_bank_int)
    , .reg_key_out     (reg_key_int)
    , .reg_mask_out    (reg_mask_int)
    , .reg_route_out   (reg_route_int)
    );
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // HSSL interface and GTH block clocks
  // free-running clock for the GTH block derived from the PL clock
  // provided by the processor subsystem.
  //NOTE: (/2) avoids reconfiguration of the processor subsystem PL clock
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

  assign axi_clk_int = gth_userclk_tx_usrclk2_int;
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

  assign axi_resetn_int = gth_reset_tx_done_int && gth_userclk_tx_active_int;
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
  wire [PACKET_BITS - 1:0] pkt_data_int;
  wire                     pkt_vld_int;
  wire                     pkt_rdy_int;

  wire [PACKET_BITS - 1:0] tx_pkt_data_int [NUM_CHANNELS - 1:0];
  wire                     tx_pkt_vld_int  [NUM_CHANNELS - 1:0];
  wire                     tx_pkt_rdy_int  [NUM_CHANNELS - 1:0];

  //make unused receivers *not* ready
  wire rx_pkt_rdy_int [NUM_CHANNELS - 1:0];
  genvar chan;
  generate
    for (chan = 0; chan < NUM_CHANNELS; chan = chan + 1)
      begin : rx_not_rdy
        assign rx_pkt_rdy_int[chan] = 1'b0;
      end
  endgenerate

  // assemble packets using events sent by processor subsystem
  pkt_assembler pa
    (
      .clk                (hsslif_clk_int)
    , .reset              (hsslif_reset_int)

    , .evt_data_in        (evt_data_int)
    , .evt_vld_in         (evt_vld_int)
    , .evt_rdy_out        (evt_rdy_int)

    , .pkt_data_out       (pkt_data_int)
    , .pkt_vld_out        (pkt_vld_int)
    , .pkt_rdy_in         (pkt_rdy_int)
    );

  // route packets to hssl channels
  pkt_router pr
    (
      // routing table data from register bank
      .reg_key_in         (reg_key_int)
    , .reg_mask_in        (reg_mask_int)
    , .reg_route_in       (reg_route_int)

      // incoming packet
    , .pkt_in_data_in     (pkt_data_int)
    , .pkt_in_vld_in      (pkt_vld_int)
    , .pkt_in_rdy_out     (pkt_rdy_int)

      // outgoing packet channels
    , .pkt_out_data_out   (tx_pkt_data_int)
    , .pkt_out_vld_out    (tx_pkt_vld_int)
    , .pkt_out_rdy_in     (tx_pkt_rdy_int)
    );
  //---------------------------------------------------------------


  //---------------------------------------------------------------
  // HSSL interface
  //---------------------------------------------------------------
  hssl_interface hssl_interface_inst (
      .clk                            (hsslif_clk_int)
    , .reset                          (hsslif_reset_int)

    , .tx_pkt_data_in                 (tx_pkt_data_int)
    , .tx_pkt_vld_in                  (tx_pkt_vld_int)
    , .tx_pkt_rdy_out                 (tx_pkt_rdy_int)

    , .rx_pkt_data_out                ()
    , .rx_pkt_vld_out                 ()
    , .rx_pkt_rdy_in                  (rx_pkt_rdy_int)

    , .handshake_complete_out         (handshake_complete_int)
    , .version_mismatch_out           (version_mismatch_int)
    , .reg_idsi_out                   (reg_idsi_int)
    , .reg_stop_in                    (hsslif_control_int[0])

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
