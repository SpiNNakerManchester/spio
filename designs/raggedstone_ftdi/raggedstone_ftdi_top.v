/**
 * Raggedstone FPGA design providing low-speed connctivity between a
 * USB-connected host and the SpiNNaker system via a S-ATA cable link. Top level
 * module.
 */

`include "../../modules/hss_multiplexer/spio_hss_multiplexer_common.h"
`include "../../modules/hss_multiplexer/spio_hss_multiplexer_reg_bank.h"


module raggedstone_ftdi_top #( // Enable simulation mode for GTP tile
                               parameter SIMULATION = 0
                               // Speed up simulated reset of GTP tile
                             , parameter SIMULATION_GTPRESET_SPEEDUP = 0
                               // Enable chip-scope Virtual I/O interface (e.g.
                               // to access the HSS multiplexer debug register bank)
                             , parameter DEBUG_CHIPSCOPE_VIO = 1
                               // The interval at which clock correction sequences should
                               // be inserted (in cycles).
                             , parameter CLOCK_CORRECTION_INTERVAL = 1000
                               // The number of bits for the above counters.
                             , parameter CLOCK_CORRECTION_INTERVAL_BITS = 10
                               // Number of consecutive handshakes which must arrive
                               // before advancing the handshake phase.
                             , parameter NUM_HANDSHAKES = 100
                               // Number of bits required for the above counters.
                             , parameter NUM_HANDSHAKES_BITS = 7
                               // The number of bits required to address a
                               // buffer of packets received by the UART (i.e.
                               // the packet receive buffer will have size
                               // (1<<BUFFER_ADDR_BITS)-1). If set to zero, no
                               // buffer will be used.
                             , parameter UART_RX_BUFFER_ADDR_BITS = 4
                               // The high water mark for RX packet buffer occupancy beyond
                               // which the clear-to-send signal is deasserted.
                             , parameter UART_RX_HIGH_WATER_MARK = 8
                             )
                             ( // Active-low Reset signal (Top FPGA button, SW2)
                               input wire NRESET_IN
                               
                               // Another button input (Bottom FPGA button, SW1)
                             , input wire NBUTTON_IN
                               
                               // Status LEDs
                             , output wire [5:2] LEDS_OUT
                               
                               // Differential 150 MHz clock source
                             , input wire REFCLK_PAD_P_IN
                             , input wire REFCLK_PAD_N_IN
                             
                               // Wires for the first S-ATA port
                             , input  wire HSS_RXN_IN
                             , input  wire HSS_RXP_IN
                             , output wire HSS_TXN_OUT
                             , output wire HSS_TXP_OUT
                               
                               // FTDI-connected UART pins
                             , output wire FTDI_CTS_N_OUT
                             , output wire FTDI_DCD_N_OUT
                             , output wire FTDI_DSR_N_OUT
                             , output wire FTDI_RI_N_OUT
                             , input  wire FTDI_RTS_N_IN
                             , input  wire FTDI_DTR_N_IN
                             , input  wire FTDI_TXD_IN
                             , output wire FTDI_RXD_OUT
                             );

genvar i;

// GTP internal loopback connectivity (for debugging)
localparam GTP_LOOPBACK = 3'b000;

// GTP Analog signal generation settings (either found via IBERT or left as zeros)
localparam RXEQMIX       =   2'b00; // Default
localparam TXDIFFCTRL    = 4'b0000; // Default
localparam TXPREEMPHASIS =  3'b000; // Default


////////////////////////////////////////////////////////////////////////////////
// Internal signals
////////////////////////////////////////////////////////////////////////////////

// External reset signal
wire nreset_i;
wire reset_i;

// Reset GTP blocks
wire gtp_reset_i;

// Reset clock scaler
wire clk_reset_i;

// Reset for HSS link modules
wire hss_reset_i;

// Reset for LED control
wire led_reset_i;

// LED signals
wire [5:2] leds_i;

// Button signals (including an active-high version)
wire nbutton_i;
wire button_i;

// Input clock to the GTP modules from the external clock
wire gtpclkin_i;

// The GTP's external clock signal, exposed for general use
wire [1:0] unbuffered_gtpclkout_i;
wire gtpclkout_i;

// GTP tile PLL stability signal
wire plllkdet_i;

// GTP tile reset completion signals
wire gtpresetdone_i;

// User clocks, all of these are positive-edge aligned.
wire usrclk_i;
wire usrclk2_i;

// LED flasher clock
wire led_clk_i;

// Are the user clocks stable?
wire usrclks_stable_i;

// Comma detection and byte alignment status
wire [1:0] rxlossofsync_i;

// Data being received [tile][block]
wire [31:0] rxdata_i;
wire  [3:0] rxchariscomma_i;
wire  [3:0] rxcharisk_i;

// Data to transmit
wire [31:0] txdata_i;
wire  [3:0] txcharisk_i;


// HSS Link Status
wire handshake_complete_i;
wire version_mismatch_i;


// HSS multiplexer packet interfaces [port]
wire [`PKT_BITS-1:0] hss_pkt_txdata_i [`NUM_CHANS-1:0];
wire                 hss_pkt_txvld_i  [`NUM_CHANS-1:0];
wire                 hss_pkt_txrdy_i  [`NUM_CHANS-1:0];
wire [`PKT_BITS-1:0] hss_pkt_rxdata_i [`NUM_CHANS-1:0];
wire                 hss_pkt_rxvld_i  [`NUM_CHANS-1:0];
wire                 hss_pkt_rxrdy_i  [`NUM_CHANS-1:0];


// UART packet interface
wire [`PKT_BITS-1:0] uart_pkt_txdata_i;
wire                 uart_pkt_txvld_i;
wire                 uart_pkt_txrdy_i;
wire [`PKT_BITS-1:0] uart_pkt_rxdata_i;
wire                 uart_pkt_rxvld_i;
wire                 uart_pkt_rxrdy_i;

// UART signals
wire uart_rx_i;
wire uart_rx_synced_i;
wire uart_tx_i;
wire uart_cts_i;

// UART status signal
wire uart_synchronising_i;

// A signal asserted whenever at least one packet link transfers a packet (used
// for status indication)
wire hss_activity_i;
wire uart_activity_i;

// HSS Multiplexer debug registers (for access by chipscope)
wire [`REGA_BITS-1:0] reg_addr_i;
wire [`REGD_BITS-1:0] reg_data_i;


////////////////////////////////////////////////////////////////////////////////
// Buttons
////////////////////////////////////////////////////////////////////////////////

// Buffer the button signals
IBUF nbuttons_buf_i (.I(NBUTTON_IN), .O(nbutton_i));

// Convert to active high
assign button_i = ~nbutton_i;



////////////////////////////////////////////////////////////////////////////////
// Reset
////////////////////////////////////////////////////////////////////////////////

// Active-low button, invert to get active-high reset signal.
IBUF reset_buf (.I (NRESET_IN), .O (nreset_i));
assign reset_i = ~nreset_i;

assign gtp_reset_i = reset_i;

assign clk_reset_i = reset_i | !plllkdet_i;

// The HSS block is connected to the GTP block and so must wait until it has
// completely reset.
assign hss_reset_i = !gtpresetdone_i & !usrclks_stable_i;

assign led_reset_i = !usrclks_stable_i;


////////////////////////////////////////////////////////////////////////////////
// LEDs
////////////////////////////////////////////////////////////////////////////////

// Buffer the LEDs
OBUF leds_buf_i [5:2] (.I(leds_i[5:2]), .O(LEDS_OUT[5:2]));


// Generate a LED status for the UART and high-speed serial link
spio_status_led_generator     #( // The number of devices (and thus LEDs)
                                 .NUM_DEVICES(2)
                                 // Animation period in clock cycles
                               , .ANIMATION_PERIOD_BITS(SIMULATION ? 10 : 26)
                                 // Duration of brief pulses (cycles)
                               , .PULSE_DURATION(SIMULATION ? 2 : 3750000)
                                 // Which bit of the period counter should be
                                 // used to produce the activity blink
                               , .ACTIVITY_BLINK_BIT(SIMULATION ? 5 : 21)
                                 // Number of bits PWM resolution
                               , .PWM_BITS(7)
                                 // Timeout for non-activity before
                                 // deasserting the activity status.
                               , .ACTIVITY_TIMEOUT(SIMULATION ? 2048 : 9375000)
                               , .ACTIVITY_TIMEOUT_BITS(24)
                               )
spio_status_led_generator_hss_i( .CLK_IN               (led_clk_i)
                               , .RESET_IN             (led_reset_i)
                               , .ERROR_IN             ({1'b0,                  version_mismatch_i})
                               , .CONNECTED_IN         ({!uart_synchronising_i, handshake_complete_i})
                               , .ACTIVITY_IN          ({uart_activity_i,       hss_activity_i})
                               , .LED_OUT              (leds_i[3:2])
                               , .ANIMATION_REPEAT_OUT () // Unused
                               );

// Just turn off the other LEDs
assign leds_i[5:4] = 2'b00;


// Generate the link activity signal for the link.
assign hss_activity_i = (hss_pkt_rxvld_i[0] & hss_pkt_rxrdy_i[0])
                      | (hss_pkt_rxvld_i[1] & hss_pkt_rxrdy_i[1])
                      | (hss_pkt_rxvld_i[2] & hss_pkt_rxrdy_i[2])
                      | (hss_pkt_rxvld_i[3] & hss_pkt_rxrdy_i[3])
                      | (hss_pkt_rxvld_i[4] & hss_pkt_rxrdy_i[4])
                      | (hss_pkt_rxvld_i[5] & hss_pkt_rxrdy_i[5])
                      | (hss_pkt_rxvld_i[6] & hss_pkt_rxrdy_i[6])
                      | (hss_pkt_rxvld_i[7] & hss_pkt_rxrdy_i[7])
                      | (hss_pkt_txvld_i[0] & hss_pkt_txrdy_i[0])
                      | (hss_pkt_txvld_i[1] & hss_pkt_txrdy_i[1])
                      | (hss_pkt_txvld_i[2] & hss_pkt_txrdy_i[2])
                      | (hss_pkt_txvld_i[3] & hss_pkt_txrdy_i[3])
                      | (hss_pkt_txvld_i[4] & hss_pkt_txrdy_i[4])
                      | (hss_pkt_txvld_i[5] & hss_pkt_txrdy_i[5])
                      | (hss_pkt_txvld_i[6] & hss_pkt_txrdy_i[6])
                      | (hss_pkt_txvld_i[7] & hss_pkt_txrdy_i[7])
                      ;


// Generate the link activity signal for the link.
assign uart_activity_i = (uart_pkt_rxvld_i & uart_pkt_rxrdy_i)
                       | (uart_pkt_txvld_i & uart_pkt_txrdy_i)
                       ;


////////////////////////////////////////////////////////////////////////////////
// Clock generation/scaling
////////////////////////////////////////////////////////////////////////////////

// External differential clock signal for GTP tile (150 MHz)
IBUFDS refclk_ibufds_i ( .O (gtpclkin_i)
                       , .I (REFCLK_PAD_P_IN)
                       , .IB(REFCLK_PAD_N_IN)
                       );

// Buffer the copy of the transceiver's external clock
BUFIO2 # (.DIVIDE (1), .DIVIDE_BYPASS ("TRUE"))
gtpclkout_0_pll_bufio2_i ( .I            (unbuffered_gtpclkout_i[0])
                         , .DIVCLK       (gtpclkout_i)
                         , .IOCLK        () // Unused
                         , .SERDESSTROBE () // Unused
                         );

// Multiply/divide the transceiver's external clock to get the two user clocks
// for each link type
wire clk_150_i;
wire clk_37_5_i;
clock_scaler
clock_scaler_i ( .CLK_IN1  (gtpclkout_i) // 150  Mhz (Input)
               , .CLK_OUT1 (clk_150_i)   // 150  MHz (Output)
               , .CLK_OUT2 (clk_37_5_i)  // 37.5 MHz (Output)
               , .RESET    (clk_reset_i)
               , .LOCKED   (usrclks_stable_i)
               );

assign usrclk_i  = clk_150_i;
assign usrclk2_i = clk_37_5_i;

assign led_clk_i = clk_37_5_i;


////////////////////////////////////////////////////////////////////////////////
// GTP Block
////////////////////////////////////////////////////////////////////////////////

// The transceiver IP block.

// X0Y0: The S-ATA link on block 1, block 0 unused but enabled to enable
// Loss-of-Sync FSM workaround.
gtp_x0_y0  #( .WRAPPER_SIM_GTPRESET_SPEEDUP (SIMULATION_GTPRESET_SPEEDUP)
            , .WRAPPER_SIMULATION           (SIMULATION)
            )
gtp_x0_y0_i ( // TILE0 (X0_Y0)
                // Loopback and Powerdown Ports
                .TILE0_LOOPBACK0_IN         (3'b000) // Block 0 Unused
            ,   .TILE0_LOOPBACK1_IN         (GTP_LOOPBACK)
                // PLL Ports
            ,   .TILE0_CLK00_IN             (gtpclkin_i)
            ,   .TILE0_CLK01_IN             (1'b0) // Uses the first block's clock, just tie-off
            ,   .TILE0_GTPRESET0_IN         (gtp_reset_i)
            ,   .TILE0_GTPRESET1_IN         (gtp_reset_i)
            ,   .TILE0_PLLLKDET0_OUT        (plllkdet_i)
            ,   .TILE0_RESETDONE0_OUT       () // Block 0 Unused
            ,   .TILE0_RESETDONE1_OUT       (gtpresetdone_i)
                // Receive Ports - 8b10b Decoder
            ,   .TILE0_RXCHARISCOMMA0_OUT   () // Block 0 Unused
            ,   .TILE0_RXCHARISCOMMA1_OUT   (rxchariscomma_i)
            ,   .TILE0_RXCHARISK0_OUT       () // Block 0 Unused
            ,   .TILE0_RXCHARISK1_OUT       (rxcharisk_i)
            ,   .TILE0_RXDISPERR0_OUT       () // Unused
            ,   .TILE0_RXDISPERR1_OUT       () // Unused
            ,   .TILE0_RXNOTINTABLE0_OUT    () // Unused
            ,   .TILE0_RXNOTINTABLE1_OUT    () // Unused
                // Receive Ports - Clock Correction
            ,   .TILE0_RXCLKCORCNT0_OUT     () // Unused
            ,   .TILE0_RXCLKCORCNT1_OUT     () // Unused
                // Receive Ports - Comma Detection and Alignment
            ,   .TILE0_RXENMCOMMAALIGN0_IN  (1'b1) // Always realign
            ,   .TILE0_RXENMCOMMAALIGN1_IN  (1'b1) // Always realign
            ,   .TILE0_RXENPCOMMAALIGN0_IN  (1'b1) // Always realign
            ,   .TILE0_RXENPCOMMAALIGN1_IN  (1'b1) // Always realign
                // Receive Ports - RX Data Path interface
            ,   .TILE0_RXDATA0_OUT          () // Block 0 Unused
            ,   .TILE0_RXDATA1_OUT          (rxdata_i)
            ,   .TILE0_RXUSRCLK0_IN         (1'b0) // Block 0 Unused
            ,   .TILE0_RXUSRCLK1_IN         (usrclk_i)
            ,   .TILE0_RXUSRCLK20_IN        (1'b0) // Block 0 Unused
            ,   .TILE0_RXUSRCLK21_IN        (usrclk2_i)
                // Receive Ports - RX Driver,OOB signalling,Coupling and Eq.,CDR
            ,   .TILE0_RXN0_IN              (1'bX) // Block 0 Unused
            ,   .TILE0_RXN1_IN              (HSS_RXN_IN)
            ,   .TILE0_RXP0_IN              (1'bX) // Block 0 Unused
            ,   .TILE0_RXP1_IN              (HSS_RXP_IN)
                // Receive Ports - RX Loss-of-sync State Machine
            ,   .TILE0_RXLOSSOFSYNC0_OUT    () // Block 0 Unused
            ,   .TILE0_RXLOSSOFSYNC1_OUT    (rxlossofsync_i)
                // TX/RX Datapath Ports
            ,   .TILE0_GTPCLKOUT0_OUT       (unbuffered_gtpclkout_i)
            ,   .TILE0_GTPCLKOUT1_OUT       () // TILE0_GTPCLKOUT0_OUT used for everything
                // Transmit Ports - 8b10b Encoder Control
            ,   .TILE0_TXCHARISK0_IN        (4'bXXXX) // Block 0 Unused
            ,   .TILE0_TXCHARISK1_IN        (txcharisk_i)
                // Transmit Ports - TX Data Path interface
            ,   .TILE0_TXDATA0_IN           (32'hXXXX) // Block 0 Unused
            ,   .TILE0_TXDATA1_IN           (txdata_i)
            ,   .TILE0_TXUSRCLK0_IN         (1'b0) // Block 0 Unused
            ,   .TILE0_TXUSRCLK1_IN         (usrclk_i)
            ,   .TILE0_TXUSRCLK20_IN        (1'b0) // Block 0 Unused
            ,   .TILE0_TXUSRCLK21_IN        (usrclk2_i)
                // Transmit Ports - TX Driver and OOB signalling
            ,   .TILE0_TXN0_OUT             () // Block 0 Unused
            ,   .TILE0_TXN1_OUT             (HSS_TXN_OUT)
            ,   .TILE0_TXP0_OUT             () // Block 0 Unused
            ,   .TILE0_TXP1_OUT             (HSS_TXP_OUT)
                // Receive Ports - Channel Bonding (Unused)
            ,   .TILE0_RXCHANBONDSEQ0_OUT   () // Unused
            ,   .TILE0_RXCHANBONDSEQ1_OUT   () // Unused
            ,   .TILE0_RXCHANISALIGNED0_OUT () // Unused
            ,   .TILE0_RXCHANISALIGNED1_OUT () // Unused
            ,   .TILE0_RXCHANREALIGN0_OUT   () // Unused
            ,   .TILE0_RXCHANREALIGN1_OUT   () // Unused
            ,   .TILE0_RXCHBONDMASTER0_IN   (1'b0) // Unused
            ,   .TILE0_RXCHBONDMASTER1_IN   (1'b0) // Unused
            ,   .TILE0_RXCHBONDSLAVE0_IN    (1'b0) // Unused
            ,   .TILE0_RXCHBONDSLAVE1_IN    (1'b0) // Unused
            ,   .TILE0_RXENCHANSYNC0_IN     (1'b0) // Unused
            ,   .TILE0_RXENCHANSYNC1_IN     (1'b0) // Unused
                // Analog signal generation settings
            ,   .TILE0_RXEQMIX0_IN              (2'b00) // Block 0 Unused
            ,   .TILE0_RXEQMIX1_IN              (RXEQMIX)
            ,   .TILE0_TXDIFFCTRL0_IN           (4'b0000) // Block 0 Unused
            ,   .TILE0_TXDIFFCTRL1_IN           (TXDIFFCTRL)
            ,   .TILE0_TXPREEMPHASIS0_IN        (3'b000) // Block 0 Unused
            ,   .TILE0_TXPREEMPHASIS1_IN        (TXPREEMPHASIS)
            );


////////////////////////////////////////////////////////////////////////////////
// High-speed serial multiplexer for communication with SpiNNaker
////////////////////////////////////////////////////////////////////////////////

spio_hss_multiplexer   #( .CLOCK_CORRECTION_INTERVAL      (CLOCK_CORRECTION_INTERVAL)
                        , .CLOCK_CORRECTION_INTERVAL_BITS (CLOCK_CORRECTION_INTERVAL_BITS)
                        , .NUM_HANDSHAKES                 (NUM_HANDSHAKES)
                        , .NUM_HANDSHAKES_BITS            (NUM_HANDSHAKES_BITS)
                        )
periph_hss_multiplexer_i( .CLK_IN                         (usrclk2_i)
                        , .RESET_IN                       (hss_reset_i)
                          // Status Signals
                        , .HANDSHAKE_COMPLETE_OUT         (handshake_complete_i)
                        , .VERSION_MISMATCH_OUT           (version_mismatch_i)
                          // High-Speed-Serial Interface
                        , .RXDATA_IN                      (rxdata_i)
                        , .RXCHARISCOMMA_IN               (rxchariscomma_i)
                        , .RXCHARISK_IN                   (rxcharisk_i)
                        , .RXLOSSOFSYNC_IN                (rxlossofsync_i)
                        , .TXDATA_OUT                     (txdata_i)
                        , .TXCHARISK_OUT                  (txcharisk_i)
                          // Packet interface
                        , .TX_PKT0_DATA_IN                (hss_pkt_txdata_i[0])
                        , .TX_PKT0_VLD_IN                 (hss_pkt_txvld_i[0])
                        , .TX_PKT0_RDY_OUT                (hss_pkt_txrdy_i[0])
                        , .TX_PKT1_DATA_IN                (hss_pkt_txdata_i[1])
                        , .TX_PKT1_VLD_IN                 (hss_pkt_txvld_i[1])
                        , .TX_PKT1_RDY_OUT                (hss_pkt_txrdy_i[1])
                        , .TX_PKT2_DATA_IN                (hss_pkt_txdata_i[2])
                        , .TX_PKT2_VLD_IN                 (hss_pkt_txvld_i[2])
                        , .TX_PKT2_RDY_OUT                (hss_pkt_txrdy_i[2])
                        , .TX_PKT3_DATA_IN                (hss_pkt_txdata_i[3])
                        , .TX_PKT3_VLD_IN                 (hss_pkt_txvld_i[3])
                        , .TX_PKT3_RDY_OUT                (hss_pkt_txrdy_i[3])
                        , .TX_PKT4_DATA_IN                (hss_pkt_txdata_i[4])
                        , .TX_PKT4_VLD_IN                 (hss_pkt_txvld_i[4])
                        , .TX_PKT4_RDY_OUT                (hss_pkt_txrdy_i[4])
                        , .TX_PKT5_DATA_IN                (hss_pkt_txdata_i[5])
                        , .TX_PKT5_VLD_IN                 (hss_pkt_txvld_i[5])
                        , .TX_PKT5_RDY_OUT                (hss_pkt_txrdy_i[5])
                        , .TX_PKT6_DATA_IN                (hss_pkt_txdata_i[6])
                        , .TX_PKT6_VLD_IN                 (hss_pkt_txvld_i[6])
                        , .TX_PKT6_RDY_OUT                (hss_pkt_txrdy_i[6])
                        , .TX_PKT7_DATA_IN                (hss_pkt_txdata_i[7])
                        , .TX_PKT7_VLD_IN                 (hss_pkt_txvld_i[7])
                        , .TX_PKT7_RDY_OUT                (hss_pkt_txrdy_i[7])
                        , .RX_PKT0_DATA_OUT               (hss_pkt_rxdata_i[0])
                        , .RX_PKT0_VLD_OUT                (hss_pkt_rxvld_i[0])
                        , .RX_PKT0_RDY_IN                 (hss_pkt_rxrdy_i[0])
                        , .RX_PKT1_DATA_OUT               (hss_pkt_rxdata_i[1])
                        , .RX_PKT1_VLD_OUT                (hss_pkt_rxvld_i[1])
                        , .RX_PKT1_RDY_IN                 (hss_pkt_rxrdy_i[1])
                        , .RX_PKT2_DATA_OUT               (hss_pkt_rxdata_i[2])
                        , .RX_PKT2_VLD_OUT                (hss_pkt_rxvld_i[2])
                        , .RX_PKT2_RDY_IN                 (hss_pkt_rxrdy_i[2])
                        , .RX_PKT3_DATA_OUT               (hss_pkt_rxdata_i[3])
                        , .RX_PKT3_VLD_OUT                (hss_pkt_rxvld_i[3])
                        , .RX_PKT3_RDY_IN                 (hss_pkt_rxrdy_i[3])
                        , .RX_PKT4_DATA_OUT               (hss_pkt_rxdata_i[4])
                        , .RX_PKT4_VLD_OUT                (hss_pkt_rxvld_i[4])
                        , .RX_PKT4_RDY_IN                 (hss_pkt_rxrdy_i[4])
                        , .RX_PKT5_DATA_OUT               (hss_pkt_rxdata_i[5])
                        , .RX_PKT5_VLD_OUT                (hss_pkt_rxvld_i[5])
                        , .RX_PKT5_RDY_IN                 (hss_pkt_rxrdy_i[5])
                        , .RX_PKT6_DATA_OUT               (hss_pkt_rxdata_i[6])
                        , .RX_PKT6_VLD_OUT                (hss_pkt_rxvld_i[6])
                        , .RX_PKT6_RDY_IN                 (hss_pkt_rxrdy_i[6])
                        , .RX_PKT7_DATA_OUT               (hss_pkt_rxdata_i[7])
                        , .RX_PKT7_VLD_OUT                (hss_pkt_rxvld_i[7])
                        , .RX_PKT7_RDY_IN                 (hss_pkt_rxrdy_i[7])
                          // High-level protocol performance counters
                        , .REG_ADDR_IN                    (reg_addr_i)
                        , .REG_DATA_OUT                   (reg_data_i)
                        );


////////////////////////////////////////////////////////////////////////////////
// FTDI UART (PC) Interface
////////////////////////////////////////////////////////////////////////////////

// Buffering for FTDI signals
OBUF ftdi_cts_n_buf_i (.I (!uart_cts_i), .O(FTDI_CTS_N_OUT));
OBUF ftdi_dcd_n_buf_i (.I (1'b1),        .O(FTDI_DCD_N_OUT)); // Unused
OBUF ftdi_dsr_n_buf_i (.I (1'b1),        .O(FTDI_DSR_N_OUT)); // Unused
OBUF ftdi_ri_n_buf_i  (.I (1'b1),        .O(FTDI_RI_N_OUT));  // Unused
IBUF ftdi_rts_n_buf_i (.O (),            .I(FTDI_RTS_N_IN));  // Unused
IBUF ftdi_dtr_n_buf_i (.O (),            .I(FTDI_DTR_N_IN));  // Unused
IBUF ftdi_txd_buf_i   (.O (uart_rx_i),   .I(FTDI_TXD_IN));
OBUF ftdi_rxd_buf_i   (.I (uart_tx_i),   .O(FTDI_RXD_OUT));


// Synchroniser for the serial data signal
spio_uart_sync         #( .NUM_BITS      (1)
                        , .NUM_STAGES    (2)
                        , .INITIAL_VALUE (1'b1)
                        )
spio_uart_sync_tx_data_i( .CLK_IN   (usrclk2_i)
                        , .RESET_IN (reset_i)
                        , .DATA_IN  (uart_rx_i)
                        , .DATA_OUT (uart_rx_synced_i)
                        );


// The UART module
spio_uart #( .IS_MASTER           (1'b0) // Slave mode
           , .BAUD_PERIOD         (37_500_000/115_200) // 115200 baud
           , .BAUD_NUM_BITS       (9)
           , .RX_BUFFER_ADDR_BITS (UART_RX_BUFFER_ADDR_BITS)
           , .RX_HIGH_WATER_MARK  (UART_RX_HIGH_WATER_MARK)
           )
spio_uart_i( .CLK_IN                (usrclk2_i)
           , .RESET_IN              (reset_i)
             // UART signals
           , .RX_IN                 (uart_rx_synced_i)
           , .CTS_IN                (1'b1) // FTDI doesn't provide these bidirectionally
           , .TX_OUT                (uart_tx_i)
           , .CTS_OUT               (uart_cts_i)
             // Packet sending/receving
           , .RX_DATA_OUT           (uart_pkt_rxdata_i)
           , .RX_VLD_OUT            (uart_pkt_rxvld_i)
           , .RX_RDY_IN             (uart_pkt_rxrdy_i)
           , .RX_PACKET_DROPPED_OUT () // Unused
           , .TX_DATA_IN            (uart_pkt_txdata_i)
           , .TX_VLD_IN             (uart_pkt_txvld_i)
           , .TX_RDY_OUT            (uart_pkt_txrdy_i)
           , .TX_PACKET_DROPPED_OUT () // Unused
             // Synchronisation status/control
           , .SYNC_TRIGGER_IN       (1'b0) // Slaves don't need triggering
           , .SYNCHRONISING_OUT     (uart_synchronising_i)
           );

////////////////////////////////////////////////////////////////////////////////
// Transmit packets to SpiNNaker
////////////////////////////////////////////////////////////////////////////////

// Connect first channel to the UART
assign hss_pkt_txdata_i[0] = uart_pkt_rxdata_i;
assign hss_pkt_txvld_i[0]  = uart_pkt_rxvld_i;
assign uart_pkt_rxrdy_i    = hss_pkt_txrdy_i[0];

// Tie-off the other channels
generate for (i = 1; i < `NUM_CHANS; i = i + 1)
	begin : spinnaker_inactive_tx_links
		assign hss_pkt_txdata_i[i] = {`PKT_BITS{1'bX}};
		assign hss_pkt_txvld_i[i]  = 1'b0;
	end
endgenerate


////////////////////////////////////////////////////////////////////////////////
// Handle incoming packets from SpiNNaker
////////////////////////////////////////////////////////////////////////////////

// Connect first channel to UART
assign uart_pkt_txdata_i  = hss_pkt_rxdata_i[0];
assign uart_pkt_txvld_i   = hss_pkt_rxvld_i[0];
assign hss_pkt_rxrdy_i[0] = uart_pkt_rxrdy_i;

// Accept (and ignore) other incoming packets
generate for (i = 1; i < `NUM_CHANS; i = i + 1)
	begin : spinnaker_inactive_rx_links
		assign hss_pkt_rxrdy_i[i] = 1'b1;
	end
endgenerate


////////////////////////////////////////////////////////////////////////////////
// Chipscope virtual I/O
////////////////////////////////////////////////////////////////////////////////

// Virtual I/O connections to allow observation of useful values

generate if (DEBUG_CHIPSCOPE_VIO)
	begin : chipscope_enabled
		
		wire [35:0] chipscope_control_i;
		
		wire [63:0] chipscope_sync_in_i;
		wire [4:0]  chipscope_sync_out_i;
		
		// Chipscope Integrated CONtroller (ICON)
		chipscope_icon chipscope_icon_i (.CONTROL0 (chipscope_control_i));
		
		// Chipscope Virtual I/O (VIO) Port
		chipscope_vio chipscope_vio_i ( .CONTROL (chipscope_control_i)
		                              , .CLK     (usrclk2_i)
		                              , .SYNC_IN (chipscope_sync_in_i)
		                              , .SYNC_OUT(chipscope_sync_out_i)
		                              );
		
		// HSS Multiplexer debug registers
		assign reg_addr_i                         = chipscope_sync_out_i[0+:`REGA_BITS];
		assign chipscope_sync_in_i[0+:`REGD_BITS] = reg_data_i;
		
		// HSS mux port RX ready/vld
		assign chipscope_sync_in_i[`REGD_BITS+0 +:8] = { hss_pkt_rxrdy_i[7]
		                                               , hss_pkt_rxrdy_i[6]
		                                               , hss_pkt_rxrdy_i[5]
		                                               , hss_pkt_rxrdy_i[4]
		                                               , hss_pkt_rxrdy_i[3]
		                                               , hss_pkt_rxrdy_i[2]
		                                               , hss_pkt_rxrdy_i[1]
		                                               , hss_pkt_rxrdy_i[0]
		                                               };
		assign chipscope_sync_in_i[`REGD_BITS+8 +:8] = { hss_pkt_rxvld_i[7]
		                                               , hss_pkt_rxvld_i[6]
		                                               , hss_pkt_rxvld_i[5]
		                                               , hss_pkt_rxvld_i[4]
		                                               , hss_pkt_rxvld_i[3]
		                                               , hss_pkt_rxvld_i[2]
		                                               , hss_pkt_rxvld_i[1]
		                                               , hss_pkt_rxvld_i[0]
		                                               };
		// HSS mux port TX ready/vld
		assign chipscope_sync_in_i[`REGD_BITS+16+:8] = { hss_pkt_txrdy_i[7]
		                                               , hss_pkt_txrdy_i[6]
		                                               , hss_pkt_txrdy_i[5]
		                                               , hss_pkt_txrdy_i[4]
		                                               , hss_pkt_txrdy_i[3]
		                                               , hss_pkt_txrdy_i[2]
		                                               , hss_pkt_txrdy_i[1]
		                                               , hss_pkt_txrdy_i[0]
		                                               };
		assign chipscope_sync_in_i[`REGD_BITS+24+:8] = { hss_pkt_txvld_i[7]
		                                               , hss_pkt_txvld_i[6]
		                                               , hss_pkt_txvld_i[5]
		                                               , hss_pkt_txvld_i[4]
		                                               , hss_pkt_txvld_i[3]
		                                               , hss_pkt_txvld_i[2]
		                                               , hss_pkt_txvld_i[1]
		                                               , hss_pkt_txvld_i[0]
		                                               };
		
		
	end
else
	begin : chipscope_disabled
		
		// Tie off register addresses to an arbitrary register
		assign reg_addr_i    = `VERS_REG;
		
	end
endgenerate


endmodule
