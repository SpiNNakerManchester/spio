/**
 * SpiNNaker FPGA design providing board-to-board, peripheral and inter-FPGA
 * ring network connectivity for SpiNN-5 boards. Top level module.
 *
 * In this source:
 *   Nets named b2b_* are signals for logic driving the two board-to-board links.
 *   Nets named periph_* are signals for logic driving peripheral links
 *   Nets named ring_* are signals for logic driving the FPGA ring network links
 *
 * Missing features:
 *  * Ring network implementation.
 *  * Peripheral connectivity.
 */

`include "spio_hss_multiplexer_reg_bank.h"
`include "spio_spinnaker_link.h"


module spinnaker_fpgas_top #( // Enable simulation mode for GTP tile
                              parameter SIMULATION = 0
                              // Speed up simulated reset of GTP tile
                            , parameter SIMULATION_GTPRESET_SPEEDUP = 0
                              // The interval at which clock correction sequences should
                              // be inserted (in cycles).
                            , parameter    B2B_CLOCK_CORRECTION_INTERVAL = 1000
                            , parameter PERIPH_CLOCK_CORRECTION_INTERVAL = 1000
                            , parameter   RING_CLOCK_CORRECTION_INTERVAL = 1000
                              // The number of bits for the above counters.
                            , parameter    B2B_CLOCK_CORRECTION_INTERVAL_BITS = 10
                            , parameter PERIPH_CLOCK_CORRECTION_INTERVAL_BITS = 10
                            , parameter   RING_CLOCK_CORRECTION_INTERVAL_BITS = 10
                              // Number of consecutive handshakes which must arrive
                              // before advancing the handshake phase.
                            , parameter    B2B_NUM_HANDSHAKES = 100
                            , parameter PERIPH_NUM_HANDSHAKES = 100
                            , parameter   RING_NUM_HANDSHAKES = 100
                              // Number of bits required for the above counters.
                            , parameter    B2B_NUM_HANDSHAKES_BITS = 7
                            , parameter PERIPH_NUM_HANDSHAKES_BITS = 7
                            , parameter   RING_NUM_HANDSHAKES_BITS = 7
                            )
                            ( // Reset signal (only used during simulation)
                              input wire RESET_IN
                              
                              // Status LEDs
                            , output wire RED_LED_OUT
                            , output wire GRN_LED_OUT
                              
                              // A unique identifier signal used to determine
                              // FPGA pin connections.
                            , input  wire [1:0] FPGA_ID_IN
                              
                              // Differential 150 MHz clock source for each of
                              // the tiles
                            , input wire REFCLK_PAD_P_IN
                            , input wire REFCLK_PAD_N_IN
                            
                              // Wires for all four high speed differential
                              // links from the two GTP tiles
                            , input  wire [3:0] HSS_RXN_IN
                            , input  wire [3:0] HSS_RXP_IN
                            , output wire [3:0] HSS_TXN_OUT
                            , output wire [3:0] HSS_TXP_OUT
                              
                              // Wires for the SpiNNaker 2-of-7 links. Since the
                              // three different FPGAs are connected to
                              // different configurations, these pins have their
                              // directions set by the FPGA ID signal.
                              inout wire [15:0] SL00_INOUT,
                            , inout wire [15:0] SL01_INOUT,
                            , inout wire [15:0] SL02_INOUT,
                            , inout wire [15:0] SL03_INOUT,
                            , inout wire [15:0] SL04_INOUT,
                            , inout wire [15:0] SL05_INOUT,
                            , inout wire [15:0] SL06_INOUT,
                            , inout wire [15:0] SL07_INOUT,
                            , inout wire [15:0] SL08_INOUT,
                            , inout wire [15:0] SL09_INOUT,
                            , inout wire [15:0] SL10_INOUT,
                            , inout wire [15:0] SL11_INOUT,
                            , inout wire [15:0] SL12_INOUT,
                            , inout wire [15:0] SL13_INOUT,
                            , inout wire [15:0] SL14_INOUT,
                            , inout wire [15:0] SL15_INOUT,
                            );

genvar i;


// FPGA unique identifiers (as in FPGA_ID_IN)
localparam FPGA0 = 2'b00;
localparam FPGA1 = 2'b01;
localparam FPGA2 = 2'b10;


// GTP internal loopback connectivity (for debugging)
localparam    B2B_GTP_LOOPBACK = 3'b000;
localparam PERIPH_GTP_LOOPBACK = 3'b000;
localparam   RING_GTP_LOOPBACK = 3'b000;


////////////////////////////////////////////////////////////////////////////////
// Internal signals
////////////////////////////////////////////////////////////////////////////////

// External reset signal
wire reset_i;

// Reset GTP blocks
wire gtp_reset_i;

// Reset for HSS link modules
wire    b2b_hss_reset_i [1:0];
wire periph_hss_reset_i;
wire   ring_hss_reset_i;

// Reset for SpiNNaker link blocks
wire spinnaker_link_reset_i;

// Reset for LED control
wire led_reset_i;

wire red_led_i;;
wire grn_led_i;;

wire [1:0] fpga_id_i;

// Give the SpiNNaker links an indexable interface.
wire [15:0] sl_inout_i [15:0];
assign sl_inout_i[ 0] = SL00_INOUT;
assign sl_inout_i[ 1] = SL01_INOUT;
assign sl_inout_i[ 2] = SL02_INOUT;
assign sl_inout_i[ 3] = SL03_INOUT;
assign sl_inout_i[ 4] = SL04_INOUT;
assign sl_inout_i[ 5] = SL05_INOUT;
assign sl_inout_i[ 6] = SL06_INOUT;
assign sl_inout_i[ 7] = SL07_INOUT;
assign sl_inout_i[ 8] = SL10_INOUT;
assign sl_inout_i[ 9] = SL11_INOUT;
assign sl_inout_i[10] = SL12_INOUT;
assign sl_inout_i[11] = SL13_INOUT;
assign sl_inout_i[12] = SL14_INOUT;
assign sl_inout_i[13] = SL15_INOUT;

// Input clock to the GTP modules from the external clock
wire gtpclkin_i;

// The GTP's external clock signal, exposed for general use
wire unbuffered_gtpclkout_i;
wire gtpclkout_i;

// GTP tile reset completion signals
wire    b2b_gtpresetdone_i [1:0];
wire periph_gtpresetdone_i;
wire   ring_gtpresetdone_i;

// User clocks [tile][block], all of these are positive-edge aligned.
wire    b2b_usrclk_i;
wire periph_usrclk_i;
wire   ring_usrclk_i;

wire    b2b_usrclk2_i;
wire periph_usrclk2_i;
wire   ring_usrclk2_i;

// SpiNNaker link interface clocks
wire spinnaker_link_clk0_i;
wire spinnaker_link_clk1_i;

// Are the user clocks stable?
wire usrclks_stable_i;

// Clock correction state
wire [2:0]    b2b_rxclkcorcnt_i [1:0];
wire [2:0] periph_rxclkcorcnt_i;
wire [2:0]   ring_rxclkcorcnt_i;

// Comma detection and byte alignment status
wire [1:0]    b2b_rxlossofsync_i [1:0];
wire [1:0] periph_rxlossofsync_i;
wire [1:0]   ring_rxlossofsync_i;

// Data being received [tile][block]
wire [31:0]    b2b_rxdata_i [1:0];
wire [31:0] periph_rxdata_i;
wire [31:0]   ring_rxdata_i;

wire  [3:0]    b2b_rxchariscomma_ie [1:0];
wire  [3:0] periph_rxchariscomma_ie;
wire  [3:0]   ring_rxchariscomma_ie;

wire  [3:0]    b2b_rxcharisk_i [1:0];
wire  [3:0] periph_rxcharisk_i;
wire  [3:0]   ring_rxcharisk_i;

// Data to transmit
wire [31:0]    b2b_txdata_i [1:0];
wire [31:0] periph_txdata_i;
wire [31:0]   ring_txdata_i;

wire  [3:0]    b2b_txcharisk_i [1:0];
wire  [3:0] periph_txcharisk_i;
wire  [3:0]   ring_txcharisk_i;


// HSS Link Status
wire    b2b_handshake_complete_i [1:0];
wire periph_handshake_complete_i;
wire   ring_handshake_complete_i;

wire    b2b_version_mismatch_i [1:0];
wire periph_version_mismatch_i;
wire   ring_version_mismatch_i;


// Buffered (asynchronous) SpiNNaker-link connections
wire [6:0] sl_out_data_i [15:0]; // FPGA --> SpiNNaker Data
wire       sl_out_ack_i  [15:0]; // FPGA <-- SpiNNaker Acknowledge
wire [6:0] sl_in_data_i  [15:0]; // SpiNNaker --> FPGA Data
wire       sl_in_ack_i   [15:0]; // SpiNNaker <-- FPGA Acknowledge


// Board-to-board HSS multiplexer packet interfaces [b2b_link][port]
wire [`PKT_BITS-1:0] b2b_pkt_txdata_i [1:0][`NUM_CHANS:0];
wire                 b2b_pkt_txvld_i  [1:0][`NUM_CHANS:0];
wire                 b2b_pkt_txrdy_i  [1:0][`NUM_CHANS:0];
wire [`PKT_BITS-1:0] b2b_pkt_rxdata_i [1:0][`NUM_CHANS:0];
wire                 b2b_pkt_rxvld_i  [1:0][`NUM_CHANS:0];
wire                 b2b_pkt_rxrdy_i  [1:0][`NUM_CHANS:0];

// Peripheral HSS multiplexer packet interfaces
wire [`PKT_BITS-1:0] periph_pkt_txdata_i [`NUM_CHANS:0];
wire                 periph_pkt_txvld_i  [`NUM_CHANS:0];
wire                 periph_pkt_txrdy_i  [`NUM_CHANS:0];
wire [`PKT_BITS-1:0] periph_pkt_rxdata_i [`NUM_CHANS:0];
wire                 periph_pkt_rxvld_i  [`NUM_CHANS:0];
wire                 periph_pkt_rxrdy_i  [`NUM_CHANS:0];


// SpiNNaker link (synchronous) packet interfaces
wire [`PKT_BITS-1:0] sl_pkt_txdata_i [15:0];
wire                 sl_pkt_txvld_i  [15:0];
wire                 sl_pkt_txrdy_i  [15:0];
wire [`PKT_BITS-1:0] sl_pkt_rxdata_i [15:0];
wire                 sl_pkt_rxvld_i  [15:0];
wire                 sl_pkt_rxrdy_i  [15:0];


////////////////////////////////////////////////////////////////////////////////
// Reset
////////////////////////////////////////////////////////////////////////////////

IBUF reset_buf (.I (RESET_IN), .O (reset_i));

assign gtp_reset_i = reset_i;

// HSS blocks are connected to the GTP blocks and so must wait until they have
// completely reset.
assign    b2b_hss_reset_i[0] =    !b2b_gtpresetdone_i[0] & usrclks_stable_i;
assign    b2b_hss_reset_i[1] =    !b2b_gtpresetdone_i[1] & usrclks_stable_i;
assign periph_hss_reset_i    = !periph_gtpresetdone_i    & usrclks_stable_i;
assign   ring_hss_reset_i    =   !ring_gtpresetdone_i    & usrclks_stable_i;

assign spinnaker_link_reset_i = usrclks_stable_i;

assign led_reset_i = usrclks_stable_i;


////////////////////////////////////////////////////////////////////////////////
// LEDs
////////////////////////////////////////////////////////////////////////////////

OBUF red_led_buf_i (.I (red_led_i), .O (RED_LED_OUT));
OBUF grn_led_buf_i (.I (grn_led_i), .O (GRN_LED_OUT));

// TODO: Flash the LEDs to indicate status


////////////////////////////////////////////////////////////////////////////////
// Unique-Per-FPGA SpiNNaker Link Wiring and Buffering
////////////////////////////////////////////////////////////////////////////////

IBUF fpga_id_buf_i [1:0] (.I (FPGA_ID_IN), .O (fpga_id_i));

// Though SpiNNaker links are connected in order to the SL*_INOUT, the
// input/output direction is switched for different chips on each FPGA (the
// ordering is either a LOW_SL or a HIGH_SL). The signal ordering is as follows:
// * HIGH_SL:
//   * SpiNNaker -> FPGA
//     * HIGH_SL[0]   = Ack
//     * HIGH_SL[7:1] = Data
//   * FPGA -> SpiNNaker
//     * HIGH_SL[8]    = Ack
//     * HIGH_SL[15:9] = Data
// * LOW_SL:
//   * SpiNNaker -> FPGA
//     * HIGH_SL[8]    = Ack
//     * HIGH_SL[15:9] = Data
//   * FPGA -> SpiNNaker
//     * HIGH_SL[0]   = Ack
//     * HIGH_SL[7:1] = Data

// In the below bit fields, 1=input, 0=output with the LSB corresponding to the
// SpiNNaker -> FPGA data pins and FPGA -> SpiNNaker ack pin and the MSB the
// FPGA -> SpiNNaker data pins and SpiNNaker -> FPGA ack pin.
localparam LOW_SL    = 2'b10;
localparam HIGH_SL   = 2'b01;
localparam UNUSED_SL = 2'b11;

localparam [31:0] FPGA0_SL_TYPES = { HIGH_SL, HIGH_SL, HIGH_SL, HIGH_SL
                                   , HIGH_SL, HIGH_SL, HIGH_SL, HIGH_SL
                                   , HIGH_SL, LOW_SL,  HIGH_SL, LOW_SL
                                   , HIGH_SL, LOW_SL,  HIGH_SL, LOW_SL
                                   };

localparam [31:0] FPGA1_SL_TYPES = { HIGH_SL, LOW_SL,  HIGH_SL, LOW_SL
                                   , HIGH_SL, LOW_SL,  HIGH_SL, LOW_SL
                                   , HIGH_SL, HIGH_SL, HIGH_SL, HIGH_SL
                                   , HIGH_SL, HIGH_SL, HIGH_SL, HIGH_SL
                                   };

localparam [31:0] FPGA2_SL_TYPES = { LOW_SL,  LOW_SL,  LOW_SL,  LOW_SL
                                   , LOW_SL,  LOW_SL,  LOW_SL,  LOW_SL
                                   , LOW_SL,  LOW_SL,  LOW_SL,  LOW_SL
                                   , LOW_SL,  LOW_SL,  LOW_SL,  LOW_SL
                                   };

// Get the link types for this FPGA
reg [31:0] sl_types_i;
always @ (*)
	case (fpga_id_i)
		FPGA0:   sl_types_i = FPGA0_SL_TYPES;
		FPGA1:   sl_types_i = FPGA1_SL_TYPES;
		FPGA2:   sl_types_i = FPGA2_SL_TYPES;
		default: sl_types_i = {16{UNUSED_SL}};
	endcase


// Generate the tristate signal for the SpiNNaker links connected to this FPGA
wire [15:0] sl_tristate_i [15:0];
generate for (i = 0; i < 16; i = i + 1)
	begin : spinnaker_link_breakout
		assign sl_tristate_i[i] = { {7{sl_types_i[(2*i)+1]}}, sl_types_i[(2*i)+0]
		                          , {7{sl_types_i[(2*i)+0]}}, sl_types_i[(2*i)+1]
		                          };
	end
endgenerate


// Buffer the incoming SpiNNaker link signals and split input and output parts
generate for (i = 0; i < 16; i = i + 1)
	begin : spinnaker_link_buffering
		wire [15:0] sl_in_pins_i;
		wire [15:0] sl_out_pins_i;
		
		// Buffer the pins
		IOBUF sl_buf_i [15:0] ( .IO (sl_inout_i[i])
		                      , .O  (sl_in_pins_i)
		                      , .I  (sl_out_pins_i)
		                      , .T  (sl_tristate_i[i])
		                      );
		
		// Select the appropriate input signals given the link type
		assign sl_in_data_i[i] = (sl_types_i[2*i+:2] == HIGH_SL) ? sl_in_pins_i[7:1] : sl_in_pins_i[15:9];
		assign sl_out_ack_i[i] = (sl_types_i[2*i+:2] == HIGH_SL) ? sl_in_pins_i[8]   : sl_in_pins_i[0];
		
		// Duplicate the output signals since the tristate will ensure that they get
		// to the right pins
		assign sl_out_pins_i = {2{sl_out_data_i, sl_in_ack_i}};
	end
endgenerate


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
gtpclkout_0_pll_bufio2_i ( .I            (unbuffered_gtpclkout_i)
                         , .DIVCLK       (gtpclkout_i)
                         , .IOCLK        () // Unused
                         , .SERDESSTROBE () // Unused
                         );

// Multiply/divide the transceiver's external clock to get the two user clocks
// for each link type
wire clk_300_i;
wire clk_75_i;
wire clk_150_i;
wire clk_37_5_i;
clock_scaler
clock_scaler_i ( .CLK_IN1  (gtpclkout_i) // 150  Mhz (Input)
               , .CLK_OUT1 (clk_300_i)   // 300  MHz (Output)
               , .CLK_OUT2 (clk_75_i)    // 75   MHz (Output)
               , .CLK_OUT1 (clk_150_i)   // 150  MHz (Output)
               , .CLK_OUT2 (clk_37_5_i)  // 37.5 MHz (Output)
               , .RESET    (reset_i)
               , .LOCKED   (usrclks_stable_i)
               );

assign    b2b_usrclk_i  = clk_300_i;
assign periph_usrclk_i  = clk_150_i;
assign   ring_usrclk_i  = clk_300_i;

assign    b2b_usrclk2_i = clk_75_i;
assign periph_usrclk2_i = clk_37_5_i;
assign   ring_usrclk2_i = clk_75_i;

assign spinnaker_link_clk0_i = clk_150_i;
assign spinnaker_link_clk1_i = b2b_usrclk_i;


////////////////////////////////////////////////////////////////////////////////
// GTP Block
////////////////////////////////////////////////////////////////////////////////

// The transceiver IP blocks (split into two to allow complete, independent
// control of the link speeds while still using the wizard).

// X0Y0: The two board-to-board links
gtp_x0_y0  #( .WRAPPER_SIM_GTPRESET_SPEEDUP (SIMULATION_GTPRESET_SPEEDUP)
            , .WRAPPER_SIMULATION           (SIMULATION)
            )
gtp_x0_y0_i ( // TILE0 (X0_Y0)
                // Loopback and Powerdown Ports
                .TILE0_LOOPBACK0_IN         (B2B_GTP_LOOPBACK)
            ,   .TILE0_LOOPBACK1_IN         (B2B_GTP_LOOPBACK)
                // PLL Ports
            ,   .TILE0_CLK00_IN             (gtpclkin_i)
            ,   .TILE0_CLK01_IN             (1'b0) // Uses the first block's clock, just tie-off
            ,   .TILE0_GTPRESET0_IN         (reset_i)
            ,   .TILE0_GTPRESET1_IN         (reset_i)
            ,   .TILE0_PLLLKDET0_OUT        () // RESETDONE* implies this
            ,   .TILE0_RESETDONE0_OUT       (b2b_gtpresetdone_i[0])
            ,   .TILE0_RESETDONE1_OUT       (b2b_gtpresetdone_i[1])
                // Receive Ports - 8b10b Decoder
            ,   .TILE0_RXCHARISCOMMA0_OUT   (b2b_rxchariscomma_i[0])
            ,   .TILE0_RXCHARISCOMMA1_OUT   (b2b_rxchariscomma_i[1])
            ,   .TILE0_RXCHARISK0_OUT       (b2b_rxcharisk_i[0])
            ,   .TILE0_RXCHARISK1_OUT       (b2b_rxcharisk_i[1])
            ,   .TILE0_RXDISPERR0_OUT       () // Unused
            ,   .TILE0_RXDISPERR1_OUT       () // Unused
            ,   .TILE0_RXNOTINTABLE0_OUT    () // Unused
            ,   .TILE0_RXNOTINTABLE1_OUT    () // Unused
                // Receive Ports - Clock Correction
            ,   .TILE0_RXCLKCORCNT0_OUT     (rxclkcorcnt_i[0][0])
            ,   .TILE0_RXCLKCORCNT1_OUT     (rxclkcorcnt_i[0][1])
                // Receive Ports - Comma Detection and Alignment
            ,   .TILE0_RXBYTEISALIGNED0_OUT () // Unused
            ,   .TILE0_RXBYTEISALIGNED1_OUT () // Unused
            ,   .TILE0_RXBYTEREALIGN0_OUT   () // Unused
            ,   .TILE0_RXBYTEREALIGN1_OUT   () // Unused
            ,   .TILE0_RXCOMMADET0_OUT      () // Unused
            ,   .TILE0_RXCOMMADET1_OUT      () // Unused
            ,   .TILE0_RXENMCOMMAALIGN0_IN  (1'b1) // Always realign
            ,   .TILE0_RXENMCOMMAALIGN1_IN  (1'b1) // Always realign
            ,   .TILE0_RXENPCOMMAALIGN0_IN  (1'b1) // Always realign
            ,   .TILE0_RXENPCOMMAALIGN1_IN  (1'b1) // Always realign
                // Receive Ports - RX Data Path interface
            ,   .TILE0_RXDATA0_OUT          (b2b_rxdata_i[0])
            ,   .TILE0_RXDATA1_OUT          (b2b_rxdata_i[1])
            ,   .TILE0_RXUSRCLK0_IN         (b2b_usrclk_i)
            ,   .TILE0_RXUSRCLK1_IN         (b2b_usrclk_i)
            ,   .TILE0_RXUSRCLK20_IN        (b2b_usrclk2_i)
            ,   .TILE0_RXUSRCLK21_IN        (b2b_usrclk2_i)
                // Receive Ports - RX Driver,OOB signalling,Coupling and Eq.,CDR
            ,   .TILE0_GATERXELECIDLE0_IN   (1'b0)
            ,   .TILE0_GATERXELECIDLE1_IN   (1'b0)
            ,   .TILE0_IGNORESIGDET0_IN     (1'b0)
            ,   .TILE0_IGNORESIGDET1_IN     (1'b0)
            ,   .TILE0_RXELECIDLE0_OUT      () // Unused
            ,   .TILE0_RXELECIDLE1_OUT      () // Unused
            ,   .TILE0_RXN0_IN              (RXN_IN[0])
            ,   .TILE0_RXN1_IN              (RXN_IN[1])
            ,   .TILE0_RXP0_IN              (RXP_IN[0])
            ,   .TILE0_RXP1_IN              (RXP_IN[1])
                // Receive Ports - RX Loss-of-sync State Machine
            ,   .TILE0_RXLOSSOFSYNC0_OUT    (b2b_rxlossofsync_i[0])
            ,   .TILE0_RXLOSSOFSYNC1_OUT    (b2b_rxlossofsync_i[1])
                // TX/RX Datapath Ports
            ,   .TILE0_GTPCLKOUT0_OUT       (unbuffered_gtpclkout_i)
            ,   .TILE0_GTPCLKOUT1_OUT       () // TILE0_GTPCLKOUT0_OUT used for everything
                // Transmit Ports - 8b10b Encoder Control
            ,   .TILE0_TXCHARISK0_IN        (b2b_txcharisk_i[0])
            ,   .TILE0_TXCHARISK1_IN        (b2b_txcharisk_i[1])
                // Transmit Ports - TX Data Path interface
            ,   .TILE0_TXDATA0_IN           (b2b_txdata_i[0])
            ,   .TILE0_TXDATA1_IN           (b2b_txdata_i[1])
            ,   .TILE0_TXUSRCLK0_IN         (b2b_usrclk_i)
            ,   .TILE0_TXUSRCLK1_IN         (b2b_usrclk_i)
            ,   .TILE0_TXUSRCLK20_IN        (b2b_usrclk2_i)
            ,   .TILE0_TXUSRCLK21_IN        (b2b_usrclk2_i)
                // Transmit Ports - TX Driver and OOB signalling
            ,   .TILE0_TXN0_OUT             (TXN_OUT[0])
            ,   .TILE0_TXN1_OUT             (TXN_OUT[1])
            ,   .TILE0_TXP0_OUT             (TXP_OUT[0])
            ,   .TILE0_TXP1_OUT             (TXP_OUT[1])
            );

// X1Y0: Peripheral links and the ring network
gtp_x1_y0  #( .WRAPPER_SIM_GTPRESET_SPEEDUP (SIMULATION_GTPRESET_SPEEDUP)
            , .WRAPPER_SIMULATION           (SIMULATION)
            )
gtp_x1_y0_i ( // TILE0 (X0_Y0)
                // Loopback and Powerdown Ports
                .TILE0_LOOPBACK0_IN         (PERIPH_GTP_LOOPBACK) // Not loopback mode
            ,   .TILE0_LOOPBACK1_IN         (  RING_GTP_LOOPBACK) // Not loopback mode
                // PLL Ports
            ,   .TILE0_CLK00_IN             (gtpclkin_i)
            ,   .TILE0_CLK01_IN             (1'b0) // Uses the first block's clock, just tie-off
            ,   .TILE0_GTPRESET0_IN         (reset_i)
            ,   .TILE0_GTPRESET1_IN         (reset_i)
            ,   .TILE0_PLLLKDET0_OUT        () // RESETDONE* implies this
            ,   .TILE0_RESETDONE0_OUT       (periph_gtpresetdone_i)
            ,   .TILE0_RESETDONE1_OUT       (  ring_gtpresetdone_i)
                // Receive Ports - 8b10b Decoder
            ,   .TILE0_RXCHARISCOMMA0_OUT   (periph_rxchariscomma_i)
            ,   .TILE0_RXCHARISCOMMA1_OUT   (  ring_rxchariscomma_i)
            ,   .TILE0_RXCHARISK0_OUT       (periph_rxcharisk_i)
            ,   .TILE0_RXCHARISK1_OUT       (  ring_rxcharisk_i)
            ,   .TILE0_RXDISPERR0_OUT       () // Unused
            ,   .TILE0_RXDISPERR1_OUT       () // Unused
            ,   .TILE0_RXNOTINTABLE0_OUT    () // Unused
            ,   .TILE0_RXNOTINTABLE1_OUT    () // Unused
                // Receive Ports - Clock Correction
            ,   .TILE0_RXCLKCORCNT0_OUT     (rxclkcorcnt_i[1][0])
            ,   .TILE0_RXCLKCORCNT1_OUT     (rxclkcorcnt_i[1][1])
                // Receive Ports - Comma Detection and Alignment
            ,   .TILE0_RXBYTEISALIGNED0_OUT () // Unused
            ,   .TILE0_RXBYTEISALIGNED1_OUT () // Unused
            ,   .TILE0_RXBYTEREALIGN0_OUT   () // Unused
            ,   .TILE0_RXBYTEREALIGN1_OUT   () // Unused
            ,   .TILE0_RXCOMMADET0_OUT      () // Unused
            ,   .TILE0_RXCOMMADET1_OUT      () // Unused
            ,   .TILE0_RXENMCOMMAALIGN0_IN  (1'b1) // Always realign
            ,   .TILE0_RXENMCOMMAALIGN1_IN  (1'b1) // Always realign
            ,   .TILE0_RXENPCOMMAALIGN0_IN  (1'b1) // Always realign
            ,   .TILE0_RXENPCOMMAALIGN1_IN  (1'b1) // Always realign
                // Receive Ports - RX Data Path interface
            ,   .TILE0_RXDATA0_OUT          (periph_rxdata_i)
            ,   .TILE0_RXDATA1_OUT          (  ring_rxdata_i)
            ,   .TILE0_RXUSRCLK0_IN         (periph_usrclk_i)
            ,   .TILE0_RXUSRCLK1_IN         (  ring_usrclk_i)
            ,   .TILE0_RXUSRCLK20_IN        (periph_usrclk2_i)
            ,   .TILE0_RXUSRCLK21_IN        (  ring_usrclk2_i)
                // Receive Ports - RX Driver,OOB signalling,Coupling and Eq.,CDR
            ,   .TILE0_GATERXELECIDLE0_IN   (1'b0)
            ,   .TILE0_GATERXELECIDLE1_IN   (1'b0)
            ,   .TILE0_IGNORESIGDET0_IN     (1'b0)
            ,   .TILE0_IGNORESIGDET1_IN     (1'b0)
            ,   .TILE0_RXELECIDLE0_OUT      () // Unused
            ,   .TILE0_RXELECIDLE1_OUT      () // Unused
            ,   .TILE0_RXN0_IN              (RXN_IN[2])
            ,   .TILE0_RXN1_IN              (RXN_IN[3])
            ,   .TILE0_RXP0_IN              (RXP_IN[2])
            ,   .TILE0_RXP1_IN              (RXP_IN[3])
                // Receive Ports - RX Loss-of-sync State Machine
            ,   .TILE0_RXLOSSOFSYNC0_OUT    (periph_rxlossofsync_i)
            ,   .TILE0_RXLOSSOFSYNC1_OUT    (  ring_rxlossofsync_i)
                // TX/RX Datapath Ports
            ,   .TILE0_GTPCLKOUT0_OUT       () // TILE0_GTPCLKOUT0_OUT used for everything
            ,   .TILE0_GTPCLKOUT1_OUT       () // TILE0_GTPCLKOUT0_OUT used for everything
                // Transmit Ports - 8b10b Encoder Control
            ,   .TILE0_TXCHARISK0_IN        (periph_txcharisk_i)
            ,   .TILE0_TXCHARISK1_IN        (  ring_txcharisk_i)
                // Transmit Ports - TX Data Path interface
            ,   .TILE0_TXDATA0_IN           (periph_txdata_i)
            ,   .TILE0_TXDATA1_IN           (  ring_txdata_i)
            ,   .TILE0_TXUSRCLK0_IN         (periph_usrclk_i)
            ,   .TILE0_TXUSRCLK1_IN         (  ring_usrclk_i)
            ,   .TILE0_TXUSRCLK20_IN        (periph_usrclk2_i)
            ,   .TILE0_TXUSRCLK21_IN        (  ring_usrclk2_i)
                // Transmit Ports - TX Driver and OOB signalling
            ,   .TILE0_TXN0_OUT             (TXN_OUT[2])
            ,   .TILE0_TXN1_OUT             (TXN_OUT[3])
            ,   .TILE0_TXP0_OUT             (TXP_OUT[2])
            ,   .TILE0_TXP1_OUT             (TXP_OUT[3])
            );


////////////////////////////////////////////////////////////////////////////////
// High-speed serial multiplexers for board-to-board and peripheral connections
////////////////////////////////////////////////////////////////////////////////

// Connect boards together in the system.
generate for (i = 0; i < 2; i = i + 1)
	begin : b2b_hss_multiplexers
		spio_hss_multiplexer#( .CLOCK_CORRECTION_INTERVAL      (B2B_CLOCK_CORRECTION_INTERVAL)
		                     , .CLOCK_CORRECTION_INTERVAL_BITS (B2B_CLOCK_CORRECTION_INTERVAL_BITS)
		                     , .NUM_HANDSHAKES                 (B2B_NUM_HANDSHAKES)
		                     , .NUM_HANDSHAKES_BITS            (B2B_NUM_HANDSHAKES_BITS)
		                     )
		b2b_hss_multiplexer_i( .CLK_IN                         (b2b_usrclk_i)
		                     , .RESET_IN                       (b2b_hss_reset_i[i])
		                       // Status Signals
		                     , .HANDSHAKE_COMPLETE_OUT         (b2b_handshake_complete_i[i])
		                     , .VERSION_MISMATCH_OUT           (b2b_version_mismatch_i[i])
		                       // High-Speed-Serial Interface
		                     , .RXDATA_IN                      (b2b_rxdata_i[i])
		                     , .RXCHARISCOMMA_IN               (b2b_rxchariscomma_i[i])
		                     , .RXCHARISK_IN                   (b2b_rxcharisk_i[i])
		                     , .RXLOSSOFSYNC_IN                (b2b_rxlossofsync_i[i])
		                     , .TXDATA_OUT                     (b2b_txdata_i[i])
		                     , .TXCHARISK_OUT                  (b2b_txcharisk_i[i])
		                       // Packet interface
		                     , .TX_PKT0_DATA_IN                (b2b_pkt_txdata_i[i][0])
		                     , .TX_PKT0_VLD_IN                 (b2b_pkt_txvld_i[i][0])
		                     , .TX_PKT0_RDY_OUT                (b2b_pkt_txrdy_i[i][0])
		                     , .TX_PKT1_DATA_IN                (b2b_pkt_txdata_i[i][1])
		                     , .TX_PKT1_VLD_IN                 (b2b_pkt_txvld_i[i][1])
		                     , .TX_PKT1_RDY_OUT                (b2b_pkt_txrdy_i[i][1])
		                     , .TX_PKT2_DATA_IN                (b2b_pkt_txdata_i[i][2])
		                     , .TX_PKT2_VLD_IN                 (b2b_pkt_txvld_i[i][2])
		                     , .TX_PKT2_RDY_OUT                (b2b_pkt_txrdy_i[i][2])
		                     , .TX_PKT3_DATA_IN                (b2b_pkt_txdata_i[i][3])
		                     , .TX_PKT3_VLD_IN                 (b2b_pkt_txvld_i[i][3])
		                     , .TX_PKT3_RDY_OUT                (b2b_pkt_txrdy_i[i][3])
		                     , .TX_PKT4_DATA_IN                (b2b_pkt_txdata_i[i][4])
		                     , .TX_PKT4_VLD_IN                 (b2b_pkt_txvld_i[i][4])
		                     , .TX_PKT4_RDY_OUT                (b2b_pkt_txrdy_i[i][4])
		                     , .TX_PKT5_DATA_IN                (b2b_pkt_txdata_i[i][5])
		                     , .TX_PKT5_VLD_IN                 (b2b_pkt_txvld_i[i][5])
		                     , .TX_PKT5_RDY_OUT                (b2b_pkt_txrdy_i[i][5])
		                     , .TX_PKT6_DATA_IN                (b2b_pkt_txdata_i[i][6])
		                     , .TX_PKT6_VLD_IN                 (b2b_pkt_txvld_i[i][6])
		                     , .TX_PKT6_RDY_OUT                (b2b_pkt_txrdy_i[i][6])
		                     , .TX_PKT7_DATA_IN                (b2b_pkt_txdata_i[i][7])
		                     , .TX_PKT7_VLD_IN                 (b2b_pkt_txvld_i[i][7])
		                     , .TX_PKT7_RDY_OUT                (b2b_pkt_txrdy_i[i][7])
		                     , .RX_PKT0_DATA_OUT               (b2b_pkt_rxdata_i[i][0])
		                     , .RX_PKT0_VLD_OUT                (b2b_pkt_rxvld_i[i][0])
		                     , .RX_PKT0_RDY_IN                 (b2b_pkt_rxrdy_i[i][0])
		                     , .RX_PKT1_DATA_OUT               (b2b_pkt_rxdata_i[i][1])
		                     , .RX_PKT1_VLD_OUT                (b2b_pkt_rxvld_i[i][1])
		                     , .RX_PKT1_RDY_IN                 (b2b_pkt_rxrdy_i[i][1])
		                     , .RX_PKT2_DATA_OUT               (b2b_pkt_rxdata_i[i][2])
		                     , .RX_PKT2_VLD_OUT                (b2b_pkt_rxvld_i[i][2])
		                     , .RX_PKT2_RDY_IN                 (b2b_pkt_rxrdy_i[i][2])
		                     , .RX_PKT3_DATA_OUT               (b2b_pkt_rxdata_i[i][3])
		                     , .RX_PKT3_VLD_OUT                (b2b_pkt_rxvld_i[i][3])
		                     , .RX_PKT3_RDY_IN                 (b2b_pkt_rxrdy_i[i][3])
		                     , .RX_PKT4_DATA_OUT               (b2b_pkt_rxdata_i[i][4])
		                     , .RX_PKT4_VLD_OUT                (b2b_pkt_rxvld_i[i][4])
		                     , .RX_PKT4_RDY_IN                 (b2b_pkt_rxrdy_i[i][4])
		                     , .RX_PKT5_DATA_OUT               (b2b_pkt_rxdata_i[i][5])
		                     , .RX_PKT5_VLD_OUT                (b2b_pkt_rxvld_i[i][5])
		                     , .RX_PKT5_RDY_IN                 (b2b_pkt_rxrdy_i[i][5])
		                     , .RX_PKT6_DATA_OUT               (b2b_pkt_rxdata_i[i][6])
		                     , .RX_PKT6_VLD_OUT                (b2b_pkt_rxvld_i[i][6])
		                     , .RX_PKT6_RDY_IN                 (b2b_pkt_rxrdy_i[i][6])
		                     , .RX_PKT7_DATA_OUT               (b2b_pkt_rxdata_i[i][7])
		                     , .RX_PKT7_VLD_OUT                (b2b_pkt_rxvld_i[i][7])
		                     , .RX_PKT7_RDY_IN                 (b2b_pkt_rxrdy_i[i][7])
		                       // High-level protocol performance counters (unused)
		                     , .REG_ADDR_IN                    (`VERS_REG)
		                     , .REG_DATA_OUT                   () // Ignored
		                     );
	end
endgenerate

// Connect to peripherals
spio_hss_multiplexer   #( .CLOCK_CORRECTION_INTERVAL      (PERIPH_CLOCK_CORRECTION_INTERVAL)
                        , .CLOCK_CORRECTION_INTERVAL_BITS (PERIPH_CLOCK_CORRECTION_INTERVAL_BITS)
                        , .NUM_HANDSHAKES                 (PERIPH_NUM_HANDSHAKES)
                        , .NUM_HANDSHAKES_BITS            (PERIPH_NUM_HANDSHAKES_BITS)
                        )
periph_hss_multiplexer_i( .CLK_IN                         (periph_usrclk_i)
                        , .RESET_IN                       (periph_hss_reset_i)
                          // Status Signals
                        , .HANDSHAKE_COMPLETE_OUT         (periph_handshake_complete_i)
                        , .VERSION_MISMATCH_OUT           (periph_version_mismatch_i)
                          // High-Speed-Serial Interface
                        , .RXDATA_IN                      (periph_rxdata_i)
                        , .RXCHARISCOMMA_IN               (periph_rxchariscomma_i)
                        , .RXCHARISK_IN                   (periph_rxcharisk_i)
                        , .RXLOSSOFSYNC_IN                (periph_rxlossofsync_i)
                        , .TXDATA_OUT                     (periph_txdata_i)
                        , .TXCHARISK_OUT                  (periph_txcharisk_i)
                          // Packet interface
                        , .TX_PKT0_DATA_IN                (periph_pkt_txdata_i[0])
                        , .TX_PKT0_VLD_IN                 (periph_pkt_txvld_i[0])
                        , .TX_PKT0_RDY_OUT                (periph_pkt_txrdy_i[0])
                        , .TX_PKT1_DATA_IN                (periph_pkt_txdata_i[1])
                        , .TX_PKT1_VLD_IN                 (periph_pkt_txvld_i[1])
                        , .TX_PKT1_RDY_OUT                (periph_pkt_txrdy_i[1])
                        , .TX_PKT2_DATA_IN                (periph_pkt_txdata_i[2])
                        , .TX_PKT2_VLD_IN                 (periph_pkt_txvld_i[2])
                        , .TX_PKT2_RDY_OUT                (periph_pkt_txrdy_i[2])
                        , .TX_PKT3_DATA_IN                (periph_pkt_txdata_i[3])
                        , .TX_PKT3_VLD_IN                 (periph_pkt_txvld_i[3])
                        , .TX_PKT3_RDY_OUT                (periph_pkt_txrdy_i[3])
                        , .TX_PKT4_DATA_IN                (periph_pkt_txdata_i[4])
                        , .TX_PKT4_VLD_IN                 (periph_pkt_txvld_i[4])
                        , .TX_PKT4_RDY_OUT                (periph_pkt_txrdy_i[4])
                        , .TX_PKT5_DATA_IN                (periph_pkt_txdata_i[5])
                        , .TX_PKT5_VLD_IN                 (periph_pkt_txvld_i[5])
                        , .TX_PKT5_RDY_OUT                (periph_pkt_txrdy_i[5])
                        , .TX_PKT6_DATA_IN                (periph_pkt_txdata_i[6])
                        , .TX_PKT6_VLD_IN                 (periph_pkt_txvld_i[6])
                        , .TX_PKT6_RDY_OUT                (periph_pkt_txrdy_i[6])
                        , .TX_PKT7_DATA_IN                (periph_pkt_txdata_i[7])
                        , .TX_PKT7_VLD_IN                 (periph_pkt_txvld_i[7])
                        , .TX_PKT7_RDY_OUT                (periph_pkt_txrdy_i[7])
                        , .RX_PKT0_DATA_OUT               (periph_pkt_rxdata_i[0])
                        , .RX_PKT0_VLD_OUT                (periph_pkt_rxvld_i[0])
                        , .RX_PKT0_RDY_IN                 (periph_pkt_rxrdy_i[0])
                        , .RX_PKT1_DATA_OUT               (periph_pkt_rxdata_i[1])
                        , .RX_PKT1_VLD_OUT                (periph_pkt_rxvld_i[1])
                        , .RX_PKT1_RDY_IN                 (periph_pkt_rxrdy_i[1])
                        , .RX_PKT2_DATA_OUT               (periph_pkt_rxdata_i[2])
                        , .RX_PKT2_VLD_OUT                (periph_pkt_rxvld_i[2])
                        , .RX_PKT2_RDY_IN                 (periph_pkt_rxrdy_i[2])
                        , .RX_PKT3_DATA_OUT               (periph_pkt_rxdata_i[3])
                        , .RX_PKT3_VLD_OUT                (periph_pkt_rxvld_i[3])
                        , .RX_PKT3_RDY_IN                 (periph_pkt_rxrdy_i[3])
                        , .RX_PKT4_DATA_OUT               (periph_pkt_rxdata_i[4])
                        , .RX_PKT4_VLD_OUT                (periph_pkt_rxvld_i[4])
                        , .RX_PKT4_RDY_IN                 (periph_pkt_rxrdy_i[4])
                        , .RX_PKT5_DATA_OUT               (periph_pkt_rxdata_i[5])
                        , .RX_PKT5_VLD_OUT                (periph_pkt_rxvld_i[5])
                        , .RX_PKT5_RDY_IN                 (periph_pkt_rxrdy_i[5])
                        , .RX_PKT6_DATA_OUT               (periph_pkt_rxdata_i[6])
                        , .RX_PKT6_VLD_OUT                (periph_pkt_rxvld_i[6])
                        , .RX_PKT6_RDY_IN                 (periph_pkt_rxrdy_i[6])
                        , .RX_PKT7_DATA_OUT               (periph_pkt_rxdata_i[7])
                        , .RX_PKT7_VLD_OUT                (periph_pkt_rxvld_i[7])
                        , .RX_PKT7_RDY_IN                 (periph_pkt_rxrdy_i[7])
                          // High-level protocol performance counters (unused)
                        , .REG_ADDR_IN                    (`VERS_REG)
                        , .REG_DATA_OUT                   () // Ignored
                        );


////////////////////////////////////////////////////////////////////////////////
// High-speed serial ring network
////////////////////////////////////////////////////////////////////////////////

// TODO: Develop ring-network protocol


////////////////////////////////////////////////////////////////////////////////
// SpiNNaker links
////////////////////////////////////////////////////////////////////////////////


// SpiNNaker link to packet interface conversion
generate for (i = 0; i < 16; i = i + 1)
	begin : spinnaker_link_interface
		// SpiNNaker link signals after passing through a synchroniser
		wire       synced_sl_out_ack_i;
		wire [6:0] synced_sl_in_data_i;
		
		// FPGA -> SpiNNaker
		spio_spinnaker_link_sender
		spio_spinnaker_link_sender_i( .CLK_IN           (spinnaker_link_clk1_i)
		                            , .RESET_IN         (spinnaker_link_reset_i)
		                              // Packet counter interface
		                            , .COUNT_PACKET_OUT () // Unused
		                              // Synchronous packet interface
		                            , .PKT_DATA_IN      (sl_pkt_txdata_i[i])
		                            , .PKT_VLD_IN       (sl_pkt_txvld_i[i])
		                            , .PKT_RDY_OUT      (sl_pkt_txrdy_i[i])
		                            // SpiNNaker link asynchronous interface
		                            , .SL_DATA_2OF7_OUT (sl_out_data_i)
		                            , .SL_ACK_IN        (synced_sl_out_ack_i)
		
		// SpiNNaker -> FPGA
		spio_spinnaker_link_receiver
		spio_spinnaker_link_receiver_i( .CLK_IN           (spinnaker_link_clk1_i)
		                              , .RESET_IN         (spinnaker_link_reset_i)
		                                // Packet counter interface
		                              , .COUNT_PACKET_OUT () // Unused
		                              // SpiNNaker link asynchronous interface
		                              , .SL_DATA_2OF7_IN  (synced_sl_in_data_i)
		                              , .SL_ACK_OUT       (sl_in_ack_i)
		                                // Synchronous packet interface
		                              , .PKT_DATA_OUT     (sl_pkt_rxdata_i[i])
		                              , .PKT_VLD_OUT      (sl_pkt_rxvld_i[i])
		                              , .PKT_RDY_IN       (sl_pkt_rxrdy_i[i])
		                              );
		
		// Synchronisers
		spio_spinnaker_link_sync#(.SIZE(1))
		spio_spinnaker_link_sync_i( .CLK (spinnaker_link_clk1_i)
		                          , .IN  (sl_out_ack_i)
		                          , .OUT (synced_sl_out_ack_i)
		                          );
		
		spio_spinnaker_link_sync2#(.SIZE(7))
		spio_spinnaker_link_sync2_i( .CLK0 (spinnaker_link_clk0_i)
		                           ( .CLK1 (spinnaker_link_clk1_i)
		                           , .IN   (sl_in_data_i)
		                           , .OUT  (synced_sl_in_data_i)
		                           );
		
	end
endgenerate


////////////////////////////////////////////////////////////////////////////////
// Routing of SpiNNaker packets from SpiNNaker chips
////////////////////////////////////////////////////////////////////////////////

// TODO
// XXX: Temporarily connect spinnaker links straight to board-to-board links
generate for (i = 0; i < `NUM_CHANS; i = i + 1)
	begin : xxx_spinnaker_rx_links
		assign b2b_pkt_txdata_i[0][i] = sl_pkt_rxdata_i[i];
		assign b2b_pkt_txvld_i[0][i]  = sl_pkt_rxvld_i[i];
		assign b2b_pkt_txrdy_i[0][i]  = sl_pkt_rxrdy_i[i];
		
		assign b2b_pkt_txdata_i[1][i] = sl_pkt_rxdata_i[8 + i];
		assign b2b_pkt_txvld_i[1][i]  = sl_pkt_rxvld_i[8 + i];
		assign b2b_pkt_txrdy_i[1][i]  = sl_pkt_rxrdy_i[8 + i];
	end
endgenerate

////////////////////////////////////////////////////////////////////////////////
// Arbitration of packets for SpiNNaker chip outputs
////////////////////////////////////////////////////////////////////////////////

// TODO
// XXX: Temporarily connect spinnaker links straight to board-to-board links
generate for (i = 0; i < `NUM_CHANS; i = i + 1)
	begin : xxx_spinnaker_tx_links
		assign b2b_pkt_rxdata_i[0][i] = sl_pkt_txdata_i[i];
		assign b2b_pkt_rxvld_i[0][i]  = sl_pkt_txvld_i[i];
		assign b2b_pkt_rxrdy_i[0][i]  = sl_pkt_txrdy_i[i];
		
		assign b2b_pkt_rxdata_i[1][i] = sl_pkt_txdata_i[8 + i];
		assign b2b_pkt_rxvld_i[1][i]  = sl_pkt_txvld_i[8 + i];
		assign b2b_pkt_rxrdy_i[1][i]  = sl_pkt_txrdy_i[8 + i];
	end
endgenerate


endmodule
