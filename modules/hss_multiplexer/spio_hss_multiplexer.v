/**
 * A module to multiplex eight bi-directional streams of packets over a single
 * high-speed-serial link on a Spartan-6 FPGA.
 */

`include "spio_hss_multiplexer_common.h"
`include "spio_hss_multiplexer_reg_bank.h"

module spio_hss_multiplexer #( // number of HSSL mux input channels
                               parameter NUM_INPUT_CHANS = 8
                               // The interval at which clock correction sequences should
                               // be inserted (in cycles).
                             , parameter CLOCK_CORRECTION_INTERVAL = 1000
                               // The number of bits for the above counter.
                             , parameter CLOCK_CORRECTION_INTERVAL_BITS = 10
                               // Number of consecutive handshakes which must arrive
                               // before advancing the handshake phase.
                             , parameter NUM_HANDSHAKES = 100
                               // Number of bits required for the above counter.
                             , parameter NUM_HANDSHAKES_BITS = 7
                             )
                             ( // The clock on which the serial interface accepts 32-bit
                               // words for transmission/reception.
                               input wire  CLK_IN
                               // Reset the module. It is assumed that once the module
                               // comes out of reset, the serial interface wil
                               // accept/produce a 32-bit packet every cycle.
                             , input wire  RESET_IN
                               
                               // Status Signals
                                 // High if a basic level of connectivity and matching
                                 // protocol version number has been established.
                             ,   output wire HANDSHAKE_COMPLETE_OUT
                                 // High if the remote device is reporting an incomatible
                                 // version number.
                             ,   output wire VERSION_MISMATCH_OUT
                             
                               // High-Speed-Serial Interface
                                 // Receiver connections
                             ,   input wire [31:0] RXDATA_IN
                             ,   input wire  [3:0] RXCHARISCOMMA_IN
                             ,   input wire  [3:0] RXCHARISK_IN
                             ,   input wire  [1:0] RXLOSSOFSYNC_IN
                             
                                 // Transmitter connections
                             ,   output wire [31:0] TXDATA_OUT
                             ,   output wire  [3:0] TXCHARISK_OUT
                             
                               // Packet interface
                                 // Incoming data from the eight streams.  Since verilog does not
                                 // support array types for inputs or outputs, the different
                                 // channels are simply enumerated below.
                             ,   input  wire [`PKT_BITS-1:0] TX_PKT0_DATA_IN
                             ,   input  wire                 TX_PKT0_VLD_IN
                             ,   output wire                 TX_PKT0_RDY_OUT
                             ,   input  wire [`PKT_BITS-1:0] TX_PKT1_DATA_IN
                             ,   input  wire                 TX_PKT1_VLD_IN
                             ,   output wire                 TX_PKT1_RDY_OUT
                             ,   input  wire [`PKT_BITS-1:0] TX_PKT2_DATA_IN
                             ,   input  wire                 TX_PKT2_VLD_IN
                             ,   output wire                 TX_PKT2_RDY_OUT
                             ,   input  wire [`PKT_BITS-1:0] TX_PKT3_DATA_IN
                             ,   input  wire                 TX_PKT3_VLD_IN
                             ,   output wire                 TX_PKT3_RDY_OUT
                             ,   input  wire [`PKT_BITS-1:0] TX_PKT4_DATA_IN
                             ,   input  wire                 TX_PKT4_VLD_IN
                             ,   output wire                 TX_PKT4_RDY_OUT
                             ,   input  wire [`PKT_BITS-1:0] TX_PKT5_DATA_IN
                             ,   input  wire                 TX_PKT5_VLD_IN
                             ,   output wire                 TX_PKT5_RDY_OUT
                             ,   input  wire [`PKT_BITS-1:0] TX_PKT6_DATA_IN
                             ,   input  wire                 TX_PKT6_VLD_IN
                             ,   output wire                 TX_PKT6_RDY_OUT
                             ,   input  wire [`PKT_BITS-1:0] TX_PKT7_DATA_IN
                             ,   input  wire                 TX_PKT7_VLD_IN
                             ,   output wire                 TX_PKT7_RDY_OUT
                                 // Outgoing data from eight streams each
                                 // `PKT_BITS wide.
                             ,   output wire [`PKT_BITS-1:0] RX_PKT0_DATA_OUT
                             ,   output wire                 RX_PKT0_VLD_OUT
                             ,   input  wire                 RX_PKT0_RDY_IN
                             ,   output wire [`PKT_BITS-1:0] RX_PKT1_DATA_OUT
                             ,   output wire                 RX_PKT1_VLD_OUT
                             ,   input  wire                 RX_PKT1_RDY_IN
                             ,   output wire [`PKT_BITS-1:0] RX_PKT2_DATA_OUT
                             ,   output wire                 RX_PKT2_VLD_OUT
                             ,   input  wire                 RX_PKT2_RDY_IN
                             ,   output wire [`PKT_BITS-1:0] RX_PKT3_DATA_OUT
                             ,   output wire                 RX_PKT3_VLD_OUT
                             ,   input  wire                 RX_PKT3_RDY_IN
                             ,   output wire [`PKT_BITS-1:0] RX_PKT4_DATA_OUT
                             ,   output wire                 RX_PKT4_VLD_OUT
                             ,   input  wire                 RX_PKT4_RDY_IN
                             ,   output wire [`PKT_BITS-1:0] RX_PKT5_DATA_OUT
                             ,   output wire                 RX_PKT5_VLD_OUT
                             ,   input  wire                 RX_PKT5_RDY_IN
                             ,   output wire [`PKT_BITS-1:0] RX_PKT6_DATA_OUT
                             ,   output wire                 RX_PKT6_VLD_OUT
                             ,   input  wire                 RX_PKT6_RDY_IN
                             ,   output wire [`PKT_BITS-1:0] RX_PKT7_DATA_OUT
                             ,   output wire                 RX_PKT7_VLD_OUT
                             ,   input  wire                 RX_PKT7_RDY_IN
                             
                               // top-level control inputs
                             ,   input  wire                 SCRMBL_IDL_DAT

                               // High-level protocol Performance counters
                               // Accessible as a bank of read-only registers
                               // whose addresses are given in
                               // spio_hss_multiplexer_register_bank.h
                                 // Write a value to a register
                             ,   input  wire                    REG_WRITE_IN
                                 // Address of register to read/write
                             ,   input  wire [`REGA_BITS - 1:0] REG_ADDR_IN
                                 // Register value being read/written
                             ,   output wire [`REGD_BITS - 1:0] REG_READ_DATA_OUT
                             ,   input  wire [`REGD_BITS - 1:0] REG_WRITE_DATA_IN
                             );


////////////////////////////////////////////////////////////////////////////////
// Internal signals
////////////////////////////////////////////////////////////////////////////////

// Low-level serial transmission interface
wire [31:0] txdata_i;
wire  [3:0] txcharisk_i;
wire        txvld_i;
wire        txrdy_i;

wire [31:0] rxdata_i;
wire  [3:0] rxcharisk_i;
wire        rxvld_i;

// frame assembler interface
wire                    reg_sfrm_i;
wire                    reg_looc_i;
wire [`CRDT_BITS - 1:0] reg_crdt_i;
wire [`NUM_CHANS - 1:0] reg_empt_i;
wire [`NUM_CHANS - 1:0] reg_full_i;

// frame transmitter interface
wire                    reg_tfrm_i;
wire [`IDLE_BITS - 1:0] reg_idso_i;
wire                    reg_stop_i;

// frame disassembler interface
wire                    reg_dfrm_i;
wire                    reg_crce_i;
wire                    reg_frme_i;
wire                    reg_rnak_i;
wire                    reg_rack_i;
wire                    reg_rooc_i;
wire [`NUM_CHANS - 1:0] reg_cfcr_i;
wire [`IDLE_BITS - 1:0] reg_idsi_i;

// packet dispatcher interface
wire                    reg_rfrm_i;
wire                    reg_busy_i;
wire                    reg_lnak_i;
wire                    reg_lack_i;
wire [`NUM_CHANS - 1:0] reg_cfcl_i;

// TX/RX Control registers
wire [1:0]              reg_hand_i;

////////////////////////////////////////////////////////////////////////////////
// Link diagnostics registers
////////////////////////////////////////////////////////////////////////////////

spio_hss_multiplexer_reg_bank
spio_hss_multiplexer_reg_bank_i( .clk (CLK_IN)
                               , .rst (RESET_IN)
                               
                                 // frame assembler interface
                               , .reg_sfrm (reg_sfrm_i)
                               , .reg_looc (reg_looc_i)
                               , .reg_crdt (reg_crdt_i)
                               , .reg_empt (reg_empt_i)
                               , .reg_full (reg_full_i)
                               
                                 // frame transmitter interface
                               , .reg_tfrm (reg_tfrm_i)
                               , .reg_idso (reg_idso_i)
                               , .reg_stop (reg_stop_i)
                               
                                 // frame disassembler interface
                               , .reg_dfrm (reg_dfrm_i)
                               , .reg_crce (reg_crce_i)
                               , .reg_frme (reg_frme_i)
                               , .reg_rnak (reg_rnak_i)
                               , .reg_rack (reg_rack_i)
                               , .reg_rooc (reg_rooc_i)
                               , .reg_cfcr (reg_cfcr_i)
                               , .reg_idsi (reg_idsi_i)
                               
                                 // packet dispatcher interface
                               , .reg_rfrm (reg_rfrm_i)
                               , .reg_busy (reg_busy_i)
                               , .reg_lnak (reg_lnak_i)
                               , .reg_lack (reg_lack_i)
                               , .reg_cfcl (reg_cfcl_i)
                               
                                 // TX/RX Control interface
                               , .reg_hand (reg_hand_i)
                               
                                 // register access interface
                               , .reg_write      (REG_WRITE_IN)
                               , .reg_addr       (REG_ADDR_IN)
                               , .reg_read_data  (REG_READ_DATA_OUT)
                               , .reg_write_data (REG_WRITE_DATA_IN)
                               );


////////////////////////////////////////////////////////////////////////////////
// Low-level serial TX/RX control blocks
////////////////////////////////////////////////////////////////////////////////

wire handshake_phase_i;

spio_hss_multiplexer_tx_control
spio_hss_multiplexer_tx_control_i( .CLK_IN                 (CLK_IN)
                                 , .RESET_IN               (RESET_IN)
                                 , .SCRMBL_IDL_DAT         (SCRMBL_IDL_DAT)
                                 , .REG_IDSO_IN            (reg_idso_i)
                                 , .HANDSHAKE_COMPLETE_IN  (HANDSHAKE_COMPLETE_OUT)
                                 , .HANDSHAKE_PHASE_IN     (handshake_phase_i)
                                 , .TXDATA_OUT             (TXDATA_OUT)
                                 , .TXCHARISK_OUT          (TXCHARISK_OUT)
                                 , .TXDATA_IN              (txdata_i)
                                 , .TXCHARISK_IN           (txcharisk_i)
                                 , .TXVLD_IN               (txvld_i)
                                 , .TXRDY_OUT              (txrdy_i)
                                 );


spio_hss_multiplexer_rx_control #( .NUM_HANDSHAKES(NUM_HANDSHAKES)
                                 , .NUM_HANDSHAKES_BITS(NUM_HANDSHAKES_BITS)
                                 )
spio_hss_multiplexer_rx_control_i( .CLK_IN                 (CLK_IN)
                                 , .RESET_IN               (RESET_IN)
                                 , .REG_IDSI_OUT           (reg_idsi_i)
                                 , .HANDSHAKE_COMPLETE_OUT (HANDSHAKE_COMPLETE_OUT)
                                 , .HANDSHAKE_PHASE_OUT    (handshake_phase_i)
                                 , .VERSION_MISMATCH_OUT   (VERSION_MISMATCH_OUT)
                                 , .RXDATA_IN              (RXDATA_IN)
                                 , .RXCHARISCOMMA_IN       (RXCHARISCOMMA_IN)
                                 , .RXLOSSOFSYNC_IN        (RXLOSSOFSYNC_IN)
                                 , .RXCHARISK_IN           (RXCHARISK_IN)
                                 , .RXDATA_OUT             (rxdata_i)
                                 , .RXCHARISK_OUT          (rxcharisk_i)
                                 , .RXVLD_OUT              (rxvld_i)
                                 );

assign reg_hand_i = {VERSION_MISMATCH_OUT, HANDSHAKE_COMPLETE_OUT};

////////////////////////////////////////////////////////////////////////////////
// High-level packet-framing protocol.
////////////////////////////////////////////////////////////////////////////////

// TODO: Check parameters selected above are compatible

spio_hss_multiplexer_spinnlink #( .NUM_INPUT_CHANS  (NUM_INPUT_CHANS)
                                )
spio_hss_multiplexer_spinnlink_i( .clk       (CLK_IN)
                                , .rst       (RESET_IN)
                                
                                  // Diagnostic signals from frame assembler
                                , .reg_sfrm  (reg_sfrm_i)
                                , .reg_looc  (reg_looc_i)
                                , .reg_crdt  (reg_crdt_i)
                                , .reg_empt  (reg_empt_i)
                                , .reg_full  (reg_full_i)
                                  
                                  // Diagnostic/control signals from/to frame transmitter
                                , .reg_tfrm  (reg_tfrm_i)
                                , .reg_stop  (reg_stop_i)
                                  
                                  // Diagnostic signals from frame disassembler
                                , .reg_dfrm  (reg_dfrm_i)
                                , .reg_crce  (reg_crce_i)
                                , .reg_frme  (reg_frme_i)
                                , .reg_rnak  (reg_rnak_i)
                                , .reg_rack  (reg_rack_i)
                                , .reg_rooc  (reg_rooc_i)
                                , .reg_cfcr  (reg_cfcr_i)
                                  
                                  // Diagnostic signals from packet dispatcher
                                , .reg_rfrm  (reg_rfrm_i)
                                , .reg_busy  (reg_busy_i)
                                , .reg_lnak  (reg_lnak_i)
                                , .reg_lack  (reg_lack_i)
                                , .reg_cfcl  (reg_cfcl_i)
                                
                                  // To high-speed serial: assembled frames out
                                , .hsl_data  (txdata_i)
                                , .hsl_kchr  (txcharisk_i)
                                , .hsl_vld   (txvld_i)
                                , .hsl_rdy   (txrdy_i)
                                
                                  // From high-speed serial: assembled frames in
                                , .ihsl_data  (rxdata_i)
                                , .ihsl_kchr  (rxcharisk_i)
                                , .ihsl_vld   (rxvld_i)
                                
                                  // Incoming packet streams
                                , .pkt_data0 (TX_PKT0_DATA_IN)
                                , .pkt_vld0  (TX_PKT0_VLD_IN)
                                , .pkt_rdy0  (TX_PKT0_RDY_OUT)
                                
                                , .pkt_data1 (TX_PKT1_DATA_IN)
                                , .pkt_vld1  (TX_PKT1_VLD_IN)
                                , .pkt_rdy1  (TX_PKT1_RDY_OUT)
                                
                                , .pkt_data2 (TX_PKT2_DATA_IN)
                                , .pkt_vld2  (TX_PKT2_VLD_IN)
                                , .pkt_rdy2  (TX_PKT2_RDY_OUT)
                                
                                , .pkt_data3 (TX_PKT3_DATA_IN)
                                , .pkt_vld3  (TX_PKT3_VLD_IN)
                                , .pkt_rdy3  (TX_PKT3_RDY_OUT)
                                
                                , .pkt_data4 (TX_PKT4_DATA_IN)
                                , .pkt_vld4  (TX_PKT4_VLD_IN)
                                , .pkt_rdy4  (TX_PKT4_RDY_OUT)
                                
                                , .pkt_data5 (TX_PKT5_DATA_IN)
                                , .pkt_vld5  (TX_PKT5_VLD_IN)
                                , .pkt_rdy5  (TX_PKT5_RDY_OUT)
                                
                                , .pkt_data6 (TX_PKT6_DATA_IN)
                                , .pkt_vld6  (TX_PKT6_VLD_IN)
                                , .pkt_rdy6  (TX_PKT6_RDY_OUT)
                                
                                , .pkt_data7 (TX_PKT7_DATA_IN)
                                , .pkt_vld7  (TX_PKT7_VLD_IN)
                                , .pkt_rdy7  (TX_PKT7_RDY_OUT)
                                
                                  // Outgoing packet streams
                                , .opkt_data0(RX_PKT0_DATA_OUT)
                                , .opkt_vld0 (RX_PKT0_VLD_OUT)
                                , .opkt_rdy0 (RX_PKT0_RDY_IN)
                                
                                , .opkt_data1(RX_PKT1_DATA_OUT)
                                , .opkt_vld1 (RX_PKT1_VLD_OUT)
                                , .opkt_rdy1 (RX_PKT1_RDY_IN)
                                
                                , .opkt_data2(RX_PKT2_DATA_OUT)
                                , .opkt_vld2 (RX_PKT2_VLD_OUT)
                                , .opkt_rdy2 (RX_PKT2_RDY_IN)
                                
                                , .opkt_data3(RX_PKT3_DATA_OUT)
                                , .opkt_vld3 (RX_PKT3_VLD_OUT)
                                , .opkt_rdy3 (RX_PKT3_RDY_IN)
                                
                                , .opkt_data4(RX_PKT4_DATA_OUT)
                                , .opkt_vld4 (RX_PKT4_VLD_OUT)
                                , .opkt_rdy4 (RX_PKT4_RDY_IN)
                                
                                , .opkt_data5(RX_PKT5_DATA_OUT)
                                , .opkt_vld5 (RX_PKT5_VLD_OUT)
                                , .opkt_rdy5 (RX_PKT5_RDY_IN)
                                
                                , .opkt_data6(RX_PKT6_DATA_OUT)
                                , .opkt_vld6 (RX_PKT6_VLD_OUT)
                                , .opkt_rdy6 (RX_PKT6_RDY_IN)
                                
                                , .opkt_data7(RX_PKT7_DATA_OUT)
                                , .opkt_vld7 (RX_PKT7_VLD_OUT)
                                , .opkt_rdy7 (RX_PKT7_RDY_IN)
                                );

endmodule
