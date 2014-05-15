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
                             )
                             ( // Reset signal (only used during simulation)
                               input wire RESET_IN
                               
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
                               
                               // TODO: FTDI pins
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
wire [`PKT_BITS-1:0] pkt_txdata_i [`NUM_CHANS-1:0];
wire                 pkt_txvld_i  [`NUM_CHANS-1:0];
wire                 pkt_txrdy_i  [`NUM_CHANS-1:0];
wire [`PKT_BITS-1:0] pkt_rxdata_i [`NUM_CHANS-1:0];
wire                 pkt_rxvld_i  [`NUM_CHANS-1:0];
wire                 pkt_rxrdy_i  [`NUM_CHANS-1:0];


// A signal asserted whenever at least one packet link transfers a packet (used
// for status indication)
wire activity_i;


////////////////////////////////////////////////////////////////////////////////
// Reset
////////////////////////////////////////////////////////////////////////////////

IBUF reset_buf (.I (RESET_IN), .O (reset_i));

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


// Generate a LED status for each serial link
spio_status_led_generator #( // The number of devices (and thus LEDs)
                             .NUM_DEVICES(1)
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
spio_status_led_generator_i( .CLK_IN               (led_clk_i)
                           , .RESET_IN             (led_reset_i)
                           , .ERROR_IN             (version_mismatch_i)
                           , .CONNECTED_IN         (handshake_complete_i)
                           , .ACTIVITY_IN          (activity_i)
                           , .LED_OUT              (leds_i[2])
                           , .ANIMATION_REPEAT_OUT () // Unused
                           );

// Just turn off the other LEDs
assign leds_i[5:3] = 3'b000;


// Generate the link activity signal for the link.
assign activity_i = (pkt_rxvld_i[0] & pkt_rxrdy_i[0])
                  | (pkt_rxvld_i[1] & pkt_rxrdy_i[1])
                  | (pkt_rxvld_i[2] & pkt_rxrdy_i[2])
                  | (pkt_rxvld_i[3] & pkt_rxrdy_i[3])
                  | (pkt_rxvld_i[4] & pkt_rxrdy_i[4])
                  | (pkt_rxvld_i[5] & pkt_rxrdy_i[5])
                  | (pkt_rxvld_i[6] & pkt_rxrdy_i[6])
                  | (pkt_rxvld_i[7] & pkt_rxrdy_i[7])
                  | (pkt_txvld_i[0] & pkt_txrdy_i[0])
                  | (pkt_txvld_i[1] & pkt_txrdy_i[1])
                  | (pkt_txvld_i[2] & pkt_txrdy_i[2])
                  | (pkt_txvld_i[3] & pkt_txrdy_i[3])
                  | (pkt_txvld_i[4] & pkt_txrdy_i[4])
                  | (pkt_txvld_i[5] & pkt_txrdy_i[5])
                  | (pkt_txvld_i[6] & pkt_txrdy_i[6])
                  | (pkt_txvld_i[7] & pkt_txrdy_i[7])
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
                        , .TX_PKT0_DATA_IN                (pkt_txdata_i[0])
                        , .TX_PKT0_VLD_IN                 (pkt_txvld_i[0])
                        , .TX_PKT0_RDY_OUT                (pkt_txrdy_i[0])
                        , .TX_PKT1_DATA_IN                (pkt_txdata_i[1])
                        , .TX_PKT1_VLD_IN                 (pkt_txvld_i[1])
                        , .TX_PKT1_RDY_OUT                (pkt_txrdy_i[1])
                        , .TX_PKT2_DATA_IN                (pkt_txdata_i[2])
                        , .TX_PKT2_VLD_IN                 (pkt_txvld_i[2])
                        , .TX_PKT2_RDY_OUT                (pkt_txrdy_i[2])
                        , .TX_PKT3_DATA_IN                (pkt_txdata_i[3])
                        , .TX_PKT3_VLD_IN                 (pkt_txvld_i[3])
                        , .TX_PKT3_RDY_OUT                (pkt_txrdy_i[3])
                        , .TX_PKT4_DATA_IN                (pkt_txdata_i[4])
                        , .TX_PKT4_VLD_IN                 (pkt_txvld_i[4])
                        , .TX_PKT4_RDY_OUT                (pkt_txrdy_i[4])
                        , .TX_PKT5_DATA_IN                (pkt_txdata_i[5])
                        , .TX_PKT5_VLD_IN                 (pkt_txvld_i[5])
                        , .TX_PKT5_RDY_OUT                (pkt_txrdy_i[5])
                        , .TX_PKT6_DATA_IN                (pkt_txdata_i[6])
                        , .TX_PKT6_VLD_IN                 (pkt_txvld_i[6])
                        , .TX_PKT6_RDY_OUT                (pkt_txrdy_i[6])
                        , .TX_PKT7_DATA_IN                (pkt_txdata_i[7])
                        , .TX_PKT7_VLD_IN                 (pkt_txvld_i[7])
                        , .TX_PKT7_RDY_OUT                (pkt_txrdy_i[7])
                        , .RX_PKT0_DATA_OUT               (pkt_rxdata_i[0])
                        , .RX_PKT0_VLD_OUT                (pkt_rxvld_i[0])
                        , .RX_PKT0_RDY_IN                 (pkt_rxrdy_i[0])
                        , .RX_PKT1_DATA_OUT               (pkt_rxdata_i[1])
                        , .RX_PKT1_VLD_OUT                (pkt_rxvld_i[1])
                        , .RX_PKT1_RDY_IN                 (pkt_rxrdy_i[1])
                        , .RX_PKT2_DATA_OUT               (pkt_rxdata_i[2])
                        , .RX_PKT2_VLD_OUT                (pkt_rxvld_i[2])
                        , .RX_PKT2_RDY_IN                 (pkt_rxrdy_i[2])
                        , .RX_PKT3_DATA_OUT               (pkt_rxdata_i[3])
                        , .RX_PKT3_VLD_OUT                (pkt_rxvld_i[3])
                        , .RX_PKT3_RDY_IN                 (pkt_rxrdy_i[3])
                        , .RX_PKT4_DATA_OUT               (pkt_rxdata_i[4])
                        , .RX_PKT4_VLD_OUT                (pkt_rxvld_i[4])
                        , .RX_PKT4_RDY_IN                 (pkt_rxrdy_i[4])
                        , .RX_PKT5_DATA_OUT               (pkt_rxdata_i[5])
                        , .RX_PKT5_VLD_OUT                (pkt_rxvld_i[5])
                        , .RX_PKT5_RDY_IN                 (pkt_rxrdy_i[5])
                        , .RX_PKT6_DATA_OUT               (pkt_rxdata_i[6])
                        , .RX_PKT6_VLD_OUT                (pkt_rxvld_i[6])
                        , .RX_PKT6_RDY_IN                 (pkt_rxrdy_i[6])
                        , .RX_PKT7_DATA_OUT               (pkt_rxdata_i[7])
                        , .RX_PKT7_VLD_OUT                (pkt_rxvld_i[7])
                        , .RX_PKT7_RDY_IN                 (pkt_rxrdy_i[7])
                          // High-level protocol performance counters (unused)
                        , .REG_ADDR_IN                    (`VERS_REG)
                        , .REG_DATA_OUT                   () // Ignored
                        );


////////////////////////////////////////////////////////////////////////////////
// Transmit packets to SpiNNaker
////////////////////////////////////////////////////////////////////////////////

// TODO
// XXX: Temporarily just send nothing
generate for (i = 0; i < `NUM_CHANS; i = i + 1)
	begin : xxx_spinnaker_rx_links
		assign pkt_txdata_i[i] = {`PKT_BITS{1'bX}};
		assign pkt_txvld_i[i]  = 1'b0;
	end
endgenerate

////////////////////////////////////////////////////////////////////////////////
// Handle incoming packets from SpiNNaker
////////////////////////////////////////////////////////////////////////////////

// TODO
// XXX: Temporarily just ignore all incoming packets
generate for (i = 0; i < `NUM_CHANS; i = i + 1)
	begin : xxx_spinnaker_tx_links
		assign pkt_rxrdy_i[i] = 1'b1;
	end
endgenerate


endmodule
