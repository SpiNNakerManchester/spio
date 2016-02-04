/**
 * SpiNNaker FPGA design providing board-to-board, peripheral and inter-FPGA
 * ring network connectivity for SpiNN-5 boards. Also exposes a set of
 * diagnostic/debug registers via SPI. Top level module.
 *
 * In this source:
 *   Nets named b2b_* are signals for logic driving the two board-to-board links.
 *   Nets named periph_* are signals for logic driving peripheral links
 *   Nets named ring_* are signals for logic driving the FPGA ring network links
 *
 * Missing features:
 *  * Ring network implementation.
 */

`include "../../modules/spinnaker_link/spio_spinnaker_link.h"
`include "../../modules/packet_counter/spio_packet_counter.h"

`include "../../modules/hss_multiplexer/spio_hss_multiplexer_common.h"
`include "../../modules/hss_multiplexer/spio_hss_multiplexer_reg_bank.h"


module spinnaker_fpgas_top #( // Version number for top-level design
                              parameter VERSION = 32'h28010900
                              // Enable simulation mode for GTP tile
                            , parameter SIMULATION = 0
                              // Speed up simulated reset of GTP tile
                            , parameter SIMULATION_GTPRESET_SPEEDUP = 0
                              // Enable chip-scope Virtual I/O interface (e.g.
                              // to access HSS multiplexer debug register banks)
                            , parameter DEBUG_CHIPSCOPE_VIO = 0
                              // include switch, arbiter and hss multiplexer modules
                              // for peripheral port (also doubler and halver modules)
                            , parameter INCLUDE_PERIPH_SUPPORT = 0
                              // include hss multiplexer module for FPGA ring
                            , parameter INCLUDE_RING_SUPPORT = 0
                              // Which FPGA should this module be compiled for
                            , parameter FPGA_ID = 2
                              // Should North and South connections be connected
                              // to 0: J6 and J8 on the back respectively or 1:
                              // J9 and J11 on the front respectively.
                              // (peripheral connections will be placed on the
                              // opposing side).
                            , parameter NORTH_SOUTH_ON_FRONT = 1
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
                            ( // Reset signal
                              input wire N_RESET_IN
                              
                              // Status LEDs
                            , output wire RED_LED_OUT
                            , output wire GRN_LED_OUT
                              
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
                              
                              // Wires for the 16 SpiNNaker 2-of-7 links. Since
                              // the three different FPGAs are connected to
                              // different configurations, these pins have their
                              // directions implied by the FPGA_ID parameter.
                            , inout wire [(16*16)-1:0] SL_INOUT
                            
                              // SPI interface pins
                            , input  wire SPI_NSS_IN
                            , input  wire SPI_SCLK_IN
                            , input  wire SPI_MOSI_IN
                            , output wire SPI_MISO_OUT
                            );

genvar i;

`include "spinnaker_fpgas_top.h"

// GTP internal loopback connectivity (for debugging)
localparam    B2B_GTP_LOOPBACK = 3'b000;
localparam PERIPH_GTP_LOOPBACK = 3'b000;
localparam   RING_GTP_LOOPBACK = 3'b000;

// GTP Analog signal generation settings (either found via IBERT or left as zeros)
localparam    B2B_RXEQMIX = 2'b10;   // 5.4 dB
localparam PERIPH_RXEQMIX = 2'b00;   // Default
localparam   RING_RXEQMIX = 2'b00;   // Default

// !!lap localparam    B2B_TXDIFFCTRL = 4'b0010; // 495 mV
localparam    B2B_TXDIFFCTRL = 4'b0110; // 762 mV
// !!lap localparam    B2B_TXDIFFCTRL = 4'b1010; // 1054 mV
localparam PERIPH_TXDIFFCTRL = 4'b0000; // Default
localparam   RING_TXDIFFCTRL = 4'b0000; // Default

localparam    B2B_TXPREEMPHASIS = 3'b010;  // 1.7 dB
localparam PERIPH_TXPREEMPHASIS = 3'b000;  // Default
localparam   RING_TXPREEMPHASIS = 3'b000;  // Default


////////////////////////////////////////////////////////////////////////////////
// Internal signals
////////////////////////////////////////////////////////////////////////////////

// External reset signal
wire n_reset_i;
wire reset_i;

// Reset GTP blocks
wire gtp_reset_i;

// Reset clock scaler
wire clk_reset_i;

// Reset for HSS link modules
wire    b2b_hss_reset_i [1:0];
wire periph_hss_reset_i;
//wire   ring_hss_reset_i;

// Reset for SpiNNaker link blocks, one per block. Bit 0 resets link 0
// SpiNN->FPGA, Bit 1 resets link 0 FPGA->SpiNN, and so on.
wire [31:0] spinnaker_link_reset_i;

// Reset for packet arbitration/routing blocks
wire arbiter_reset_i;

// Reset for LED control
wire led_reset_i;

// Reset for SPI interface
wire spi_reset_i;

// Reset for register bank
wire reg_bank_reset_i;

// Reset for packet counters
wire pkt_ctr_reset_i;

// LED signals
wire red_led_i;
wire grn_led_i;

// Input clock to the GTP modules from the external clock
wire gtpclkin_i;

// The GTP's external clock signal, exposed for general use
wire [1:0] unbuffered_gtpclkout_i;
wire gtpclkout_i;

// GTP tile PLL stability signal
wire plllkdet_i;

// GTP tile reset completion signals
wire    b2b_gtpresetdone_i [1:0];
wire periph_gtpresetdone_i;
wire   ring_gtpresetdone_i;

// User clocks, all of these are positive-edge aligned.
wire    b2b_usrclk_i;
wire periph_usrclk_i;
wire   ring_usrclk_i;

wire    b2b_usrclk2_i;
wire periph_usrclk2_i;
wire   ring_usrclk2_i;

// SpiNNaker link interface clocks
wire spinnaker_link_clk0_i;
wire spinnaker_link_clk1_i;

// LED flasher clock
wire led_clk_i;

// SPI interface clock
wire spi_clk_i;

// Register bank clock
wire reg_bank_clk_i;

// packet counters clock
wire pkt_ctr_clk_i;

// Are the user clocks stable?
wire usrclks_stable_i;

// Comma detection and byte alignment status
wire [1:0]    b2b_rxlossofsync_i [1:0];
wire [1:0] periph_rxlossofsync_i;
wire [1:0]   ring_rxlossofsync_i;

// Data being received [tile][block]
wire [31:0]    b2b_rxdata_i [1:0];
wire [31:0] periph_rxdata_i;
wire [31:0]   ring_rxdata_i;

wire  [3:0]    b2b_rxchariscomma_i [1:0];
wire  [3:0] periph_rxchariscomma_i;
wire  [3:0]   ring_rxchariscomma_i;

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
wire [`PKT_BITS-1:0] b2b_pkt_txdata_i [1:0][`NUM_CHANS-1:0];
wire                 b2b_pkt_txvld_i  [1:0][`NUM_CHANS-1:0];
wire                 b2b_pkt_txrdy_i  [1:0][`NUM_CHANS-1:0];
wire [`PKT_BITS-1:0] b2b_pkt_rxdata_i [1:0][`NUM_CHANS-1:0];
wire                 b2b_pkt_rxvld_i  [1:0][`NUM_CHANS-1:0];
wire                 b2b_pkt_rxrdy_i  [1:0][`NUM_CHANS-1:0];
wire b2b_pkt_txactivity_i;

// Peripheral HSS multiplexer packet interfaces
wire [`PKT_BITS-1:0] periph_pkt_txdata_i [`NUM_CHANS-1:0];
wire                 periph_pkt_txvld_i  [`NUM_CHANS-1:0];
wire                 periph_pkt_txrdy_i  [`NUM_CHANS-1:0];
wire [`PKT_BITS-1:0] periph_pkt_rxdata_i [`NUM_CHANS-1:0];
wire                 periph_pkt_rxvld_i  [`NUM_CHANS-1:0];
wire                 periph_pkt_rxrdy_i  [`NUM_CHANS-1:0];


// SpiNNaker link (synchronous) packet interfaces
wire [`PKT_BITS-1:0] sl_pkt_txdata_i [15:0];
wire                 sl_pkt_txvld_i  [15:0];
wire                 sl_pkt_txrdy_i  [15:0];
wire [`PKT_BITS-1:0] sl_pkt_rxdata_i [15:0];
wire                 sl_pkt_rxvld_i  [15:0];
wire                 sl_pkt_rxrdy_i  [15:0];

// A signal asserted whenever at least one packet link transfers a packet (used
// for status indication)
wire    b2b_activity_i [1:0];
wire periph_activity_i;
wire   ring_activity_i;

// Un-decoded debug register pins
wire        reg_read_i;
wire        reg_write_i;
wire [31:0] reg_addr_i;
wire [31:0] reg_read_data_i;
wire [31:0] reg_write_data_i;

// HSS Multiplexer debug register pins (for access by chipscope)
wire                     b2b_reg_write_i [1:0];
wire                  periph_reg_write_i;
wire                    ring_reg_write_i;
wire [`REGA_BITS-1:0]    b2b_reg_addr_i [1:0];
wire [`REGA_BITS-1:0] periph_reg_addr_i;
wire [`REGA_BITS-1:0]   ring_reg_addr_i;;
wire [`REGD_BITS-1:0]    b2b_reg_read_data_i [1:0];
wire [`REGD_BITS-1:0] periph_reg_read_data_i;
wire [`REGD_BITS-1:0]   ring_reg_read_data_i;
wire [`REGD_BITS-1:0]    b2b_reg_write_data_i [1:0];
wire [`REGD_BITS-1:0] periph_reg_write_data_i;
wire [`REGD_BITS-1:0]   ring_reg_write_data_i;

// Top-level register bank signals
wire        top_reg_write_i;
wire [13:0] top_reg_addr_i;
wire [31:0] top_reg_read_data_i;
wire [31:0] top_reg_write_data_i;

// packet counter signals
wire [`CTRA_BITS-1:0] pkt_ctr_addr_i [1:0];
wire [`CTRD_BITS-1:0] pkt_ctr_read_data_i [1:0];

// Routing (spio_switch) status signals
wire [1:0] switch_blocked_outputs_i  [`NUM_CHANS-1:0];
wire [1:0] switch_selected_outputs_i [`NUM_CHANS-1:0];

// Routing (spio_switch) packet dropping port
wire [`PKT_BITS-1:0]  switch_dropped_data_i    [`NUM_CHANS-1:0];
wire [1:0]            switch_dropped_outputs_i [`NUM_CHANS-1:0];
wire                  switch_dropped_vld_i     [`NUM_CHANS-1:0];

// SpiNNaker (2-of-7) link enable signals, used to tristate link IO buffers and
// hold SpiNNaker link blocks in reset. Bit 0 controls link 0 SpiNN->FPGA, Bit
// 1 controls link 0 FPGA->SpiNN, and so on.
wire [31:0] spinnaker_link_enable_i;

// Key/mask to match to forward MC packets to peripheral link
wire [31:0] periph_mc_key_i;
wire [31:0] periph_mc_mask_i;

// control wires from the top-level register bank
wire [31:0] scrmbl_idl_dat_i;


////////////////////////////////////////////////////////////////////////////////
// Reset
////////////////////////////////////////////////////////////////////////////////

IBUF reset_buf (.I (N_RESET_IN), .O (n_reset_i));
assign reset_i = !n_reset_i;

assign gtp_reset_i = reset_i;

assign clk_reset_i = reset_i | !plllkdet_i;

// HSS blocks are connected to the GTP blocks and so must wait until they have
// completely reset.
assign    b2b_hss_reset_i[0] =    !b2b_gtpresetdone_i[0] | !usrclks_stable_i;
assign    b2b_hss_reset_i[1] =    !b2b_gtpresetdone_i[1] | !usrclks_stable_i;
assign periph_hss_reset_i    = !periph_gtpresetdone_i    | !usrclks_stable_i;
//assign   ring_hss_reset_i    =   !ring_gtpresetdone_i    | !usrclks_stable_i;


// SpiNNaker link blocks can be individually reset
generate for (i = 0; i < 32; i = i + 1)
	begin : spinnaker_link_reset
		assign spinnaker_link_reset_i[i] = !(usrclks_stable_i && spinnaker_link_enable_i[i]);
	end
endgenerate

assign arbiter_reset_i = !usrclks_stable_i;

assign led_reset_i = !usrclks_stable_i;

assign spi_reset_i = !usrclks_stable_i;

assign reg_bank_reset_i = !usrclks_stable_i;

assign pkt_ctr_reset_i = !usrclks_stable_i;


////////////////////////////////////////////////////////////////////////////////
// LEDs
////////////////////////////////////////////////////////////////////////////////

// Since the FPGA can't provide the 3.3 volts required to light the LEDs, they
// are connected something like this:
//
//   FPGA Pin ----------------,
//                            |
//        3v3 ---[Resistor]---+
//                            |
//                          [LED]
//                            |
//        Gnd ----------------'
//
// If the FPGA pin is left floating, the LED is driven by the 3v3 line via the
// resistor. If the FPGA is connected and driven low, the potential across the
// LED becomes zero and it goes out. As a result, the LED's state can be set by
// changing the tristate signal of a tristate buffer driven low.
OBUFT red_led_buf_i (.I (1'b0), .O (RED_LED_OUT), .T(~red_led_i));
OBUFT grn_led_buf_i (.I (1'b0), .O (GRN_LED_OUT), .T(~grn_led_i));

wire animation_repeat_i;

wire [3:0] device_led_states_i;

// XXX: TODO: Somehow share these two LEDs between all the status indicatorS
assign red_led_i = device_led_states_i[0];
assign grn_led_i = device_led_states_i[1];

// Generate a LED status for each serial link
spio_status_led_generator #( // The number of devices (and thus LEDs)
                             .NUM_DEVICES(4)
                             // Animation period in clock cycles
                           , .ANIMATION_PERIOD_BITS(SIMULATION ? 10 : 27)
                             // Duration of brief pulses (cycles)
                           , .PULSE_DURATION(SIMULATION ? 2 : 7500000)
                             // Which bit of the period counter should be
                             // used to produce the activity blink
                           , .ACTIVITY_BLINK_BIT(SIMULATION ? 5 : 22)
                             // Number of bits PWM resolution
                           , .PWM_BITS(7)
                             // Timeout for non-activity before
                             // deasserting the activity status.
                           , .ACTIVITY_TIMEOUT(SIMULATION ? 2048 : 18750000)
                           , .ACTIVITY_TIMEOUT_BITS(25)
                           )
spio_status_led_generator_i( .CLK_IN               (led_clk_i)
                           , .RESET_IN             (led_reset_i)
                           , .ERROR_IN             ({   ring_version_mismatch_i
                                                    , periph_version_mismatch_i
                                                    ,    b2b_version_mismatch_i[1]
                                                    ,    b2b_version_mismatch_i[0]
                                                    })
                           , .CONNECTED_IN         ({   ring_handshake_complete_i
                                                    , periph_handshake_complete_i
                                                    ,    b2b_handshake_complete_i[1]
                                                    ,    b2b_handshake_complete_i[0]
                                                    })
                           , .ACTIVITY_IN          ({   ring_activity_i
                                                    , periph_activity_i
                                                    ,    b2b_activity_i[1]
                                                    ,    b2b_activity_i[0]
                                                    })
                           , .LED_OUT              (device_led_states_i)
                           , .ANIMATION_REPEAT_OUT () // Unused
                           );


// Generate the link activity signal for each link.
generate for (i = 0; i < 2; i = i + 1)
	begin : b2b_activity_signal
		assign b2b_activity_i[i] = (b2b_pkt_rxvld_i[i][0] & b2b_pkt_rxrdy_i[i][0])
		                         | (b2b_pkt_rxvld_i[i][1] & b2b_pkt_rxrdy_i[i][1])
		                         | (b2b_pkt_rxvld_i[i][2] & b2b_pkt_rxrdy_i[i][2])
		                         | (b2b_pkt_rxvld_i[i][3] & b2b_pkt_rxrdy_i[i][3])
		                         | (b2b_pkt_rxvld_i[i][4] & b2b_pkt_rxrdy_i[i][4])
		                         | (b2b_pkt_rxvld_i[i][5] & b2b_pkt_rxrdy_i[i][5])
		                         | (b2b_pkt_rxvld_i[i][6] & b2b_pkt_rxrdy_i[i][6])
		                         | (b2b_pkt_rxvld_i[i][7] & b2b_pkt_rxrdy_i[i][7])
		                         | (b2b_pkt_txvld_i[i][0] & b2b_pkt_txrdy_i[i][0])
		                         | (b2b_pkt_txvld_i[i][1] & b2b_pkt_txrdy_i[i][1])
		                         | (b2b_pkt_txvld_i[i][2] & b2b_pkt_txrdy_i[i][2])
		                         | (b2b_pkt_txvld_i[i][3] & b2b_pkt_txrdy_i[i][3])
		                         | (b2b_pkt_txvld_i[i][4] & b2b_pkt_txrdy_i[i][4])
		                         | (b2b_pkt_txvld_i[i][5] & b2b_pkt_txrdy_i[i][5])
		                         | (b2b_pkt_txvld_i[i][6] & b2b_pkt_txrdy_i[i][6])
		                         | (b2b_pkt_txvld_i[i][7] & b2b_pkt_txrdy_i[i][7])
		                         ;
	end
endgenerate

generate if (INCLUDE_PERIPH_SUPPORT)
	assign periph_activity_i = (periph_pkt_rxvld_i[0] & periph_pkt_rxrdy_i[0])
	                         | (periph_pkt_rxvld_i[1] & periph_pkt_rxrdy_i[1])
	                         | (periph_pkt_rxvld_i[2] & periph_pkt_rxrdy_i[2])
	                         | (periph_pkt_rxvld_i[3] & periph_pkt_rxrdy_i[3])
	                         | (periph_pkt_rxvld_i[4] & periph_pkt_rxrdy_i[4])
	                         | (periph_pkt_rxvld_i[5] & periph_pkt_rxrdy_i[5])
	                         | (periph_pkt_rxvld_i[6] & periph_pkt_rxrdy_i[6])
	                         | (periph_pkt_rxvld_i[7] & periph_pkt_rxrdy_i[7])
	                         | (periph_pkt_txvld_i[0] & periph_pkt_txrdy_i[0])
	                         | (periph_pkt_txvld_i[1] & periph_pkt_txrdy_i[1])
	                         | (periph_pkt_txvld_i[2] & periph_pkt_txrdy_i[2])
	                         | (periph_pkt_txvld_i[3] & periph_pkt_txrdy_i[3])
	                         | (periph_pkt_txvld_i[4] & periph_pkt_txrdy_i[4])
	                         | (periph_pkt_txvld_i[5] & periph_pkt_txrdy_i[5])
	                         | (periph_pkt_txvld_i[6] & periph_pkt_txrdy_i[6])
	                         | (periph_pkt_txvld_i[7] & periph_pkt_txrdy_i[7])
	                         ;
else
	assign periph_activity_i = 1'b0; 
endgenerate

assign ring_activity_i = 1'b0;



////////////////////////////////////////////////////////////////////////////////
// Unique-Per-FPGA SpiNNaker Link Wiring and Buffering
////////////////////////////////////////////////////////////////////////////////

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

// Buffer the incoming SpiNNaker link signals and split input and output parts.
// Note that outputs are tristated to facilitate soft-disabling of individual
// links.
generate for (i = 0; i < 16; i = i + 1)
	begin : sl_buffers
		case (FPGA_SL_TYPES[(32*FPGA_ID) + (2*i)+:2])
			HIGH_SL:
				begin
					// SpiNNaker -> FPGA
					IBUF sl_in_data_buf_i [6:0] ( .I(SL_INOUT[(i*16)+1 +: 7]), .O(sl_in_data_i[i]));
					IOBUF sl_in_ack_buf_i       ( .IO(SL_INOUT[(i*16)+0])
					                            , .I(sl_in_ack_i[i])
					                            , .T(spinnaker_link_enable_i[(i*2) + 0])
					                            );
					// FPGA -> SpiNNaker
					IOBUF sl_out_data_buf_i [6:0] ( .IO(SL_INOUT[(i*16)+9 +: 7])
					                              , .I(sl_out_data_i[i])
					                              , .T({7{spinnaker_link_enable_i[(i*2) + 1]}})
					                              );
					IBUF sl_out_ack_buf_i         (.I(SL_INOUT[(i*16)+8]), .O(sl_out_ack_i[i]));
				end
			
			LOW_SL:
				begin
					// SpiNNaker -> FPGA
					IBUF sl_in_data_buf_i [6:0] (.I(SL_INOUT[(i*16)+9 +: 7]), .O(sl_in_data_i[i]));
					IOBUF sl_in_ack_buf_i       ( .IO(SL_INOUT[(i*16)+8])
					                            , .I(sl_in_ack_i[i])
					                            , .T(spinnaker_link_enable_i[(i*2) + 0])
					                            );
					// FPGA -> SpiNNaker
					IOBUF sl_out_data_buf_i [6:0] ( .IO(SL_INOUT[(i*16)+1 +: 7])
					                              , .I(sl_out_data_i[i])
					                              , .T({7{spinnaker_link_enable_i[(i*2) + 1]}})
					                              );
					IBUF sl_out_ack_buf_i         (.I(SL_INOUT[(i*16)+0]), .O(sl_out_ack_i[i]));
				end
		endcase
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
gtpclkout_0_pll_bufio2_i ( .I            (unbuffered_gtpclkout_i[0])
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
               , .CLK_OUT3 (clk_150_i)   // 150  MHz (Output)
               , .CLK_OUT4 (clk_37_5_i)  // 37.5 MHz (Output)
               , .RESET    (clk_reset_i)
               , .LOCKED   (usrclks_stable_i)
               );

assign    b2b_usrclk_i  = clk_300_i;
assign    b2b_usrclk2_i = clk_75_i;

generate if (INCLUDE_PERIPH_SUPPORT)
	begin
		assign periph_usrclk_i  = clk_150_i;
		assign periph_usrclk2_i = clk_37_5_i;
	end
else
	begin
		assign periph_usrclk_i  = 1'b0;
		assign periph_usrclk2_i = 1'b0;
	end
endgenerate

generate if (INCLUDE_RING_SUPPORT)
	begin
		assign   ring_usrclk_i  = clk_300_i;
		assign   ring_usrclk2_i = clk_75_i;
	end
else
	begin
		assign   ring_usrclk_i  = 1'b0;
		assign   ring_usrclk2_i = 1'b0;
	end
endgenerate

assign spinnaker_link_clk0_i = clk_150_i;
assign spinnaker_link_clk1_i = b2b_usrclk2_i;

assign led_clk_i = clk_75_i;

assign spi_clk_i = clk_75_i;

assign reg_bank_clk_i = clk_75_i;

assign pkt_ctr_clk_i = clk_75_i;


////////////////////////////////////////////////////////////////////////////////
// GTP Block
////////////////////////////////////////////////////////////////////////////////

// The transceiver IP blocks (split into two to allow complete, independent
// control of the link speeds while still using the wizard).
//
// Different versions of these blocks are provided to support the different
// speeds required by board-to-board connections and peripheral connections. The
// versions required are instantiated as gtp_x0_y0_i and gtp_x1_y0_i and
// connected up appropriately.
//
// Design         Tile   Block 0   Block 1
// -------------- ------ --------- ---------
// gtp_x0_y0_bb   0,0    b2b       b2b
// gtp_x0_y0_bp   0,0    b2b       periph
// gtp_x0_y0_pb   0,0    periph    b2b
// gtp_x1_y0_pr   1,0    periph    ring
// gtp_x1_y0_br   1,0    b2b       ring

generate case ({NORTH_SOUTH_ON_FRONT, FPGA_ID})
	{0, 0},
	{0, 1},
	{0, 2},
	{1, 1}:
		gtp_x0_y0_bb#( .WRAPPER_SIM_GTPRESET_SPEEDUP (SIMULATION_GTPRESET_SPEEDUP)
		             , .WRAPPER_SIMULATION           (SIMULATION)
		             )
		gtp_x0_y0_i  ( // TILE0 (X0_Y0)
		                 // Loopback and Powerdown Ports
		                 .TILE0_LOOPBACK0_IN         (B2B_GTP_LOOPBACK)
		             ,   .TILE0_LOOPBACK1_IN         (B2B_GTP_LOOPBACK)
		                 // PLL Ports
		             ,   .TILE0_CLK00_IN             (gtpclkin_i)
		             ,   .TILE0_CLK01_IN             (1'b0) // Uses the first block's clock, just tie-off
		             ,   .TILE0_GTPRESET0_IN         (gtp_reset_i)
		             ,   .TILE0_GTPRESET1_IN         (gtp_reset_i)
		             ,   .TILE0_PLLLKDET0_OUT        (plllkdet_i)
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
		             ,   .TILE0_RXCLKCORCNT0_OUT     () // Unused
		             ,   .TILE0_RXCLKCORCNT1_OUT     () // Unused
		                 // Receive Ports - Comma Detection and Alignment
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
		             ,   .TILE0_RXN0_IN              (HSS_RXN_IN[0])
		             ,   .TILE0_RXN1_IN              (HSS_RXN_IN[1])
		             ,   .TILE0_RXP0_IN              (HSS_RXP_IN[0])
		             ,   .TILE0_RXP1_IN              (HSS_RXP_IN[1])
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
		             ,   .TILE0_TXN0_OUT             (HSS_TXN_OUT[0])
		             ,   .TILE0_TXN1_OUT             (HSS_TXN_OUT[1])
		             ,   .TILE0_TXP0_OUT             (HSS_TXP_OUT[0])
		             ,   .TILE0_TXP1_OUT             (HSS_TXP_OUT[1])
		                 // Analog signal generation settings
		             ,   .TILE0_RXEQMIX0_IN              (B2B_RXEQMIX)
		             ,   .TILE0_RXEQMIX1_IN              (B2B_RXEQMIX)
		             ,   .TILE0_TXDIFFCTRL0_IN           (B2B_TXDIFFCTRL)
		             ,   .TILE0_TXDIFFCTRL1_IN           (B2B_TXDIFFCTRL)
		             ,   .TILE0_TXPREEMPHASIS0_IN        (B2B_TXPREEMPHASIS)
		             ,   .TILE0_TXPREEMPHASIS1_IN        (B2B_TXPREEMPHASIS)
		             );
	
	{1, 0}:
		gtp_x0_y0_bp#( .WRAPPER_SIM_GTPRESET_SPEEDUP (SIMULATION_GTPRESET_SPEEDUP)
		             , .WRAPPER_SIMULATION           (SIMULATION)
		             )
		gtp_x0_y0_i  ( // TILE0 (X0_Y0)
		                 // Loopback and Powerdown Ports
		                 .TILE0_LOOPBACK0_IN         (   B2B_GTP_LOOPBACK)
		             ,   .TILE0_LOOPBACK1_IN         (PERIPH_GTP_LOOPBACK)
		                 // PLL Ports
		             ,   .TILE0_CLK00_IN             (gtpclkin_i)
		             ,   .TILE0_CLK01_IN             (1'b0) // Uses the first block's clock, just tie-off
		             ,   .TILE0_GTPRESET0_IN         (gtp_reset_i)
		             ,   .TILE0_GTPRESET1_IN         (gtp_reset_i)
		             ,   .TILE0_PLLLKDET0_OUT        (plllkdet_i)
		             ,   .TILE0_RESETDONE0_OUT       (   b2b_gtpresetdone_i[0])
		             ,   .TILE0_RESETDONE1_OUT       (periph_gtpresetdone_i)
		                 // Receive Ports - 8b10b Decoder
		             ,   .TILE0_RXCHARISCOMMA0_OUT   (   b2b_rxchariscomma_i[0])
		             ,   .TILE0_RXCHARISCOMMA1_OUT   (periph_rxchariscomma_i)
		             ,   .TILE0_RXCHARISK0_OUT       (   b2b_rxcharisk_i[0])
		             ,   .TILE0_RXCHARISK1_OUT       (periph_rxcharisk_i)
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
		             ,   .TILE0_RXDATA0_OUT          (   b2b_rxdata_i[0])
		             ,   .TILE0_RXDATA1_OUT          (periph_rxdata_i)
		             ,   .TILE0_RXUSRCLK0_IN         (   b2b_usrclk_i)
		             ,   .TILE0_RXUSRCLK1_IN         (periph_usrclk_i)
		             ,   .TILE0_RXUSRCLK20_IN        (   b2b_usrclk2_i)
		             ,   .TILE0_RXUSRCLK21_IN        (periph_usrclk2_i)
		                 // Receive Ports - RX Driver,OOB signalling,Coupling and Eq.,CDR
		             ,   .TILE0_RXN0_IN              (HSS_RXN_IN[0])
		             ,   .TILE0_RXN1_IN              (HSS_RXN_IN[1])
		             ,   .TILE0_RXP0_IN              (HSS_RXP_IN[0])
		             ,   .TILE0_RXP1_IN              (HSS_RXP_IN[1])
		                 // Receive Ports - RX Loss-of-sync State Machine
		             ,   .TILE0_RXLOSSOFSYNC0_OUT    (   b2b_rxlossofsync_i[0])
		             ,   .TILE0_RXLOSSOFSYNC1_OUT    (periph_rxlossofsync_i)
		                 // TX/RX Datapath Ports
		             ,   .TILE0_GTPCLKOUT0_OUT       (unbuffered_gtpclkout_i)
		             ,   .TILE0_GTPCLKOUT1_OUT       () // TILE0_GTPCLKOUT0_OUT used for everything
		                 // Transmit Ports - 8b10b Encoder Control
		             ,   .TILE0_TXCHARISK0_IN        (   b2b_txcharisk_i[0])
		             ,   .TILE0_TXCHARISK1_IN        (periph_txcharisk_i)
		                 // Transmit Ports - TX Data Path interface
		             ,   .TILE0_TXDATA0_IN           (   b2b_txdata_i[0])
		             ,   .TILE0_TXDATA1_IN           (periph_txdata_i)
		             ,   .TILE0_TXUSRCLK0_IN         (   b2b_usrclk_i)
		             ,   .TILE0_TXUSRCLK1_IN         (periph_usrclk_i)
		             ,   .TILE0_TXUSRCLK20_IN        (   b2b_usrclk2_i)
		             ,   .TILE0_TXUSRCLK21_IN        (periph_usrclk2_i)
		                 // Transmit Ports - TX Driver and OOB signalling
		             ,   .TILE0_TXN0_OUT             (HSS_TXN_OUT[0])
		             ,   .TILE0_TXN1_OUT             (HSS_TXN_OUT[1])
		             ,   .TILE0_TXP0_OUT             (HSS_TXP_OUT[0])
		             ,   .TILE0_TXP1_OUT             (HSS_TXP_OUT[1])
		                 // Analog signal generation settings
		             ,   .TILE0_RXEQMIX0_IN              (   B2B_RXEQMIX)
		             ,   .TILE0_RXEQMIX1_IN              (PERIPH_RXEQMIX)
		             ,   .TILE0_TXDIFFCTRL0_IN           (   B2B_TXDIFFCTRL)
		             ,   .TILE0_TXDIFFCTRL1_IN           (PERIPH_TXDIFFCTRL)
		             ,   .TILE0_TXPREEMPHASIS0_IN        (   B2B_TXPREEMPHASIS)
		             ,   .TILE0_TXPREEMPHASIS1_IN        (PERIPH_TXPREEMPHASIS)
		             );
	
	{1, 2}:
		gtp_x0_y0_pb#( .WRAPPER_SIM_GTPRESET_SPEEDUP (SIMULATION_GTPRESET_SPEEDUP)
		             , .WRAPPER_SIMULATION           (SIMULATION)
		             )
		gtp_x0_y0_i  ( // TILE0 (X0_Y0)
		                 // Loopback and Powerdown Ports
		                 .TILE0_LOOPBACK0_IN         (PERIPH_GTP_LOOPBACK)
		             ,   .TILE0_LOOPBACK1_IN         (   B2B_GTP_LOOPBACK)
		                 // PLL Ports
		             ,   .TILE0_CLK00_IN             (gtpclkin_i)
		             ,   .TILE0_CLK01_IN             (1'b0) // Uses the first block's clock, just tie-off
		             ,   .TILE0_GTPRESET0_IN         (gtp_reset_i)
		             ,   .TILE0_GTPRESET1_IN         (gtp_reset_i)
		             ,   .TILE0_PLLLKDET0_OUT        (plllkdet_i)
		             ,   .TILE0_RESETDONE0_OUT       (periph_gtpresetdone_i)
		             ,   .TILE0_RESETDONE1_OUT       (   b2b_gtpresetdone_i[1])
		                 // Receive Ports - 8b10b Decoder
		             ,   .TILE0_RXCHARISCOMMA0_OUT   (periph_rxchariscomma_i)
		             ,   .TILE0_RXCHARISCOMMA1_OUT   (   b2b_rxchariscomma_i[1])
		             ,   .TILE0_RXCHARISK0_OUT       (periph_rxcharisk_i)
		             ,   .TILE0_RXCHARISK1_OUT       (   b2b_rxcharisk_i[1])
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
		             ,   .TILE0_RXDATA0_OUT          (periph_rxdata_i)
		             ,   .TILE0_RXDATA1_OUT          (   b2b_rxdata_i[1])
		             ,   .TILE0_RXUSRCLK0_IN         (periph_usrclk_i)
		             ,   .TILE0_RXUSRCLK1_IN         (   b2b_usrclk_i)
		             ,   .TILE0_RXUSRCLK20_IN        (periph_usrclk2_i)
		             ,   .TILE0_RXUSRCLK21_IN        (   b2b_usrclk2_i)
		                 // Receive Ports - RX Driver,OOB signalling,Coupling and Eq.,CDR
		             ,   .TILE0_RXN0_IN              (HSS_RXN_IN[0])
		             ,   .TILE0_RXN1_IN              (HSS_RXN_IN[1])
		             ,   .TILE0_RXP0_IN              (HSS_RXP_IN[0])
		             ,   .TILE0_RXP1_IN              (HSS_RXP_IN[1])
		                 // Receive Ports - RX Loss-of-sync State Machine
		             ,   .TILE0_RXLOSSOFSYNC0_OUT    (periph_rxlossofsync_i)
		             ,   .TILE0_RXLOSSOFSYNC1_OUT    (   b2b_rxlossofsync_i[1])
		                 // TX/RX Datapath Ports
		             ,   .TILE0_GTPCLKOUT0_OUT       (unbuffered_gtpclkout_i)
		             ,   .TILE0_GTPCLKOUT1_OUT       () // TILE0_GTPCLKOUT0_OUT used for everything
		                 // Transmit Ports - 8b10b Encoder Control
		             ,   .TILE0_TXCHARISK0_IN        (periph_txcharisk_i)
		             ,   .TILE0_TXCHARISK1_IN        (   b2b_txcharisk_i[1])
		                 // Transmit Ports - TX Data Path interface
		             ,   .TILE0_TXDATA0_IN           (periph_txdata_i)
		             ,   .TILE0_TXDATA1_IN           (   b2b_txdata_i[1])
		             ,   .TILE0_TXUSRCLK0_IN         (periph_usrclk_i)
		             ,   .TILE0_TXUSRCLK1_IN         (   b2b_usrclk_i)
		             ,   .TILE0_TXUSRCLK20_IN        (periph_usrclk2_i)
		             ,   .TILE0_TXUSRCLK21_IN        (   b2b_usrclk2_i)
		                 // Transmit Ports - TX Driver and OOB signalling
		             ,   .TILE0_TXN0_OUT             (HSS_TXN_OUT[0])
		             ,   .TILE0_TXN1_OUT             (HSS_TXN_OUT[1])
		             ,   .TILE0_TXP0_OUT             (HSS_TXP_OUT[0])
		             ,   .TILE0_TXP1_OUT             (HSS_TXP_OUT[1])
		                 // Analog signal generation settings
		             ,   .TILE0_RXEQMIX0_IN              (PERIPH_RXEQMIX)
		             ,   .TILE0_RXEQMIX1_IN              (   B2B_RXEQMIX)
		             ,   .TILE0_TXDIFFCTRL0_IN           (PERIPH_TXDIFFCTRL)
		             ,   .TILE0_TXDIFFCTRL1_IN           (   B2B_TXDIFFCTRL)
		             ,   .TILE0_TXPREEMPHASIS0_IN        (PERIPH_TXPREEMPHASIS)
		             ,   .TILE0_TXPREEMPHASIS1_IN        (   B2B_TXPREEMPHASIS)
		             );
endcase endgenerate

generate case ({NORTH_SOUTH_ON_FRONT, FPGA_ID})
	{0, 0},
	{0, 1},
	{0, 2},
	{1, 1}:
		gtp_x1_y0_pr#( .WRAPPER_SIM_GTPRESET_SPEEDUP (SIMULATION_GTPRESET_SPEEDUP)
		             , .WRAPPER_SIMULATION           (SIMULATION)
		             )
		gtp_x1_y0_i  ( // TILE0 (X0_Y0)
		                 // Loopback and Powerdown Ports
		                 .TILE0_LOOPBACK0_IN         (PERIPH_GTP_LOOPBACK) // Not loopback mode
		             ,   .TILE0_LOOPBACK1_IN         (  RING_GTP_LOOPBACK) // Not loopback mode
		                 // PLL Ports
		             ,   .TILE0_CLK00_IN             (gtpclkin_i)
		             ,   .TILE0_CLK01_IN             (1'b0) // Uses the first block's clock, just tie-off
		             ,   .TILE0_GTPRESET0_IN         (gtp_reset_i)
		             ,   .TILE0_GTPRESET1_IN         (gtp_reset_i)
		             ,   .TILE0_PLLLKDET0_OUT        ()
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
		             ,   .TILE0_RXCLKCORCNT0_OUT     () // Unused
		             ,   .TILE0_RXCLKCORCNT1_OUT     () // Unused
		                 // Receive Ports - Comma Detection and Alignment
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
		             ,   .TILE0_RXN0_IN              (HSS_RXN_IN[2])
		             ,   .TILE0_RXN1_IN              (HSS_RXN_IN[3])
		             ,   .TILE0_RXP0_IN              (HSS_RXP_IN[2])
		             ,   .TILE0_RXP1_IN              (HSS_RXP_IN[3])
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
		             ,   .TILE0_TXN0_OUT             (HSS_TXN_OUT[2])
		             ,   .TILE0_TXN1_OUT             (HSS_TXN_OUT[3])
		             ,   .TILE0_TXP0_OUT             (HSS_TXP_OUT[2])
		             ,   .TILE0_TXP1_OUT             (HSS_TXP_OUT[3])
		                 // Analog signal generation settings
		             ,   .TILE0_RXEQMIX0_IN              (PERIPH_RXEQMIX)
		             ,   .TILE0_RXEQMIX1_IN              (  RING_RXEQMIX)
		             ,   .TILE0_TXDIFFCTRL0_IN           (PERIPH_TXDIFFCTRL)
		             ,   .TILE0_TXDIFFCTRL1_IN           (  RING_TXDIFFCTRL)
		             ,   .TILE0_TXPREEMPHASIS0_IN        (PERIPH_TXPREEMPHASIS)
		             ,   .TILE0_TXPREEMPHASIS1_IN        (  RING_TXPREEMPHASIS)
		            );
	
	{1, 0}:
		gtp_x1_y0_br#( .WRAPPER_SIM_GTPRESET_SPEEDUP (SIMULATION_GTPRESET_SPEEDUP)
		             , .WRAPPER_SIMULATION           (SIMULATION)
		             )
		gtp_x1_y0_i  ( // TILE0 (X0_Y0)
		                 // Loopback and Powerdown Ports
		                 .TILE0_LOOPBACK0_IN         ( B2B_GTP_LOOPBACK) // Not loopback mode
		             ,   .TILE0_LOOPBACK1_IN         (RING_GTP_LOOPBACK) // Not loopback mode
		                 // PLL Ports
		             ,   .TILE0_CLK00_IN             (gtpclkin_i)
		             ,   .TILE0_CLK01_IN             (1'b0) // Uses the first block's clock, just tie-off
		             ,   .TILE0_GTPRESET0_IN         (gtp_reset_i)
		             ,   .TILE0_GTPRESET1_IN         (gtp_reset_i)
		             ,   .TILE0_PLLLKDET0_OUT        ()
		             ,   .TILE0_RESETDONE0_OUT       ( b2b_gtpresetdone_i[1])
		             ,   .TILE0_RESETDONE1_OUT       (ring_gtpresetdone_i)
		                 // Receive Ports - 8b10b Decoder
		             ,   .TILE0_RXCHARISCOMMA0_OUT   ( b2b_rxchariscomma_i[1])
		             ,   .TILE0_RXCHARISCOMMA1_OUT   (ring_rxchariscomma_i)
		             ,   .TILE0_RXCHARISK0_OUT       ( b2b_rxcharisk_i[1])
		             ,   .TILE0_RXCHARISK1_OUT       (ring_rxcharisk_i)
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
		             ,   .TILE0_RXDATA0_OUT          ( b2b_rxdata_i[1])
		             ,   .TILE0_RXDATA1_OUT          (ring_rxdata_i)
		             ,   .TILE0_RXUSRCLK0_IN         ( b2b_usrclk_i)
		             ,   .TILE0_RXUSRCLK1_IN         (ring_usrclk_i)
		             ,   .TILE0_RXUSRCLK20_IN        ( b2b_usrclk2_i)
		             ,   .TILE0_RXUSRCLK21_IN        (ring_usrclk2_i)
		                 // Receive Ports - RX Driver,OOB signalling,Coupling and Eq.,CDR
		             ,   .TILE0_RXN0_IN              (HSS_RXN_IN[2])
		             ,   .TILE0_RXN1_IN              (HSS_RXN_IN[3])
		             ,   .TILE0_RXP0_IN              (HSS_RXP_IN[2])
		             ,   .TILE0_RXP1_IN              (HSS_RXP_IN[3])
		                 // Receive Ports - RX Loss-of-sync State Machine
		             ,   .TILE0_RXLOSSOFSYNC0_OUT    ( b2b_rxlossofsync_i[1])
		             ,   .TILE0_RXLOSSOFSYNC1_OUT    (ring_rxlossofsync_i)
		                 // TX/RX Datapath Ports
		             ,   .TILE0_GTPCLKOUT0_OUT       () // TILE0_GTPCLKOUT0_OUT used for everything
		             ,   .TILE0_GTPCLKOUT1_OUT       () // TILE0_GTPCLKOUT0_OUT used for everything
		                 // Transmit Ports - 8b10b Encoder Control
		             ,   .TILE0_TXCHARISK0_IN        ( b2b_txcharisk_i[1])
		             ,   .TILE0_TXCHARISK1_IN        (ring_txcharisk_i)
		                 // Transmit Ports - TX Data Path interface
		             ,   .TILE0_TXDATA0_IN           ( b2b_txdata_i[1])
		             ,   .TILE0_TXDATA1_IN           (ring_txdata_i)
		             ,   .TILE0_TXUSRCLK0_IN         ( b2b_usrclk_i)
		             ,   .TILE0_TXUSRCLK1_IN         (ring_usrclk_i)
		             ,   .TILE0_TXUSRCLK20_IN        ( b2b_usrclk2_i)
		             ,   .TILE0_TXUSRCLK21_IN        (ring_usrclk2_i)
		                 // Transmit Ports - TX Driver and OOB signalling
		             ,   .TILE0_TXN0_OUT             (HSS_TXN_OUT[2])
		             ,   .TILE0_TXN1_OUT             (HSS_TXN_OUT[3])
		             ,   .TILE0_TXP0_OUT             (HSS_TXP_OUT[2])
		             ,   .TILE0_TXP1_OUT             (HSS_TXP_OUT[3])
		                 // Analog signal generation settings
		             ,   .TILE0_RXEQMIX0_IN              ( B2B_RXEQMIX)
		             ,   .TILE0_RXEQMIX1_IN              (RING_RXEQMIX)
		             ,   .TILE0_TXDIFFCTRL0_IN           ( B2B_TXDIFFCTRL)
		             ,   .TILE0_TXDIFFCTRL1_IN           (RING_TXDIFFCTRL)
		             ,   .TILE0_TXPREEMPHASIS0_IN        ( B2B_TXPREEMPHASIS)
		             ,   .TILE0_TXPREEMPHASIS1_IN        (RING_TXPREEMPHASIS)
		            );
	
	{1, 2}:
		gtp_x1_y0_br#( .WRAPPER_SIM_GTPRESET_SPEEDUP (SIMULATION_GTPRESET_SPEEDUP)
		             , .WRAPPER_SIMULATION           (SIMULATION)
		             )
		gtp_x1_y0_i  ( // TILE0 (X0_Y0)
		                 // Loopback and Powerdown Ports
		                 .TILE0_LOOPBACK0_IN         ( B2B_GTP_LOOPBACK) // Not loopback mode
		             ,   .TILE0_LOOPBACK1_IN         (RING_GTP_LOOPBACK) // Not loopback mode
		                 // PLL Ports
		             ,   .TILE0_CLK00_IN             (gtpclkin_i)
		             ,   .TILE0_CLK01_IN             (1'b0) // Uses the first block's clock, just tie-off
		             ,   .TILE0_GTPRESET0_IN         (gtp_reset_i)
		             ,   .TILE0_GTPRESET1_IN         (gtp_reset_i)
		             ,   .TILE0_PLLLKDET0_OUT        ()
		             ,   .TILE0_RESETDONE0_OUT       ( b2b_gtpresetdone_i[0])
		             ,   .TILE0_RESETDONE1_OUT       (ring_gtpresetdone_i)
		                 // Receive Ports - 8b10b Decoder
		             ,   .TILE0_RXCHARISCOMMA0_OUT   ( b2b_rxchariscomma_i[0])
		             ,   .TILE0_RXCHARISCOMMA1_OUT   (ring_rxchariscomma_i)
		             ,   .TILE0_RXCHARISK0_OUT       ( b2b_rxcharisk_i[0])
		             ,   .TILE0_RXCHARISK1_OUT       (ring_rxcharisk_i)
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
		             ,   .TILE0_RXDATA0_OUT          ( b2b_rxdata_i[0])
		             ,   .TILE0_RXDATA1_OUT          (ring_rxdata_i)
		             ,   .TILE0_RXUSRCLK0_IN         ( b2b_usrclk_i)
		             ,   .TILE0_RXUSRCLK1_IN         (ring_usrclk_i)
		             ,   .TILE0_RXUSRCLK20_IN        ( b2b_usrclk2_i)
		             ,   .TILE0_RXUSRCLK21_IN        (ring_usrclk2_i)
		                 // Receive Ports - RX Driver,OOB signalling,Coupling and Eq.,CDR
		             ,   .TILE0_RXN0_IN              (HSS_RXN_IN[2])
		             ,   .TILE0_RXN1_IN              (HSS_RXN_IN[3])
		             ,   .TILE0_RXP0_IN              (HSS_RXP_IN[2])
		             ,   .TILE0_RXP1_IN              (HSS_RXP_IN[3])
		                 // Receive Ports - RX Loss-of-sync State Machine
		             ,   .TILE0_RXLOSSOFSYNC0_OUT    ( b2b_rxlossofsync_i[0])
		             ,   .TILE0_RXLOSSOFSYNC1_OUT    (ring_rxlossofsync_i)
		                 // TX/RX Datapath Ports
		             ,   .TILE0_GTPCLKOUT0_OUT       () // TILE0_GTPCLKOUT0_OUT used for everything
		             ,   .TILE0_GTPCLKOUT1_OUT       () // TILE0_GTPCLKOUT0_OUT used for everything
		                 // Transmit Ports - 8b10b Encoder Control
		             ,   .TILE0_TXCHARISK0_IN        ( b2b_txcharisk_i[0])
		             ,   .TILE0_TXCHARISK1_IN        (ring_txcharisk_i)
		                 // Transmit Ports - TX Data Path interface
		             ,   .TILE0_TXDATA0_IN           ( b2b_txdata_i[0])
		             ,   .TILE0_TXDATA1_IN           (ring_txdata_i)
		             ,   .TILE0_TXUSRCLK0_IN         ( b2b_usrclk_i)
		             ,   .TILE0_TXUSRCLK1_IN         (ring_usrclk_i)
		             ,   .TILE0_TXUSRCLK20_IN        ( b2b_usrclk2_i)
		             ,   .TILE0_TXUSRCLK21_IN        (ring_usrclk2_i)
		                 // Transmit Ports - TX Driver and OOB signalling
		             ,   .TILE0_TXN0_OUT             (HSS_TXN_OUT[2])
		             ,   .TILE0_TXN1_OUT             (HSS_TXN_OUT[3])
		             ,   .TILE0_TXP0_OUT             (HSS_TXP_OUT[2])
		             ,   .TILE0_TXP1_OUT             (HSS_TXP_OUT[3])
		                 // Analog signal generation settings
		             ,   .TILE0_RXEQMIX0_IN              ( B2B_RXEQMIX)
		             ,   .TILE0_RXEQMIX1_IN              (RING_RXEQMIX)
		             ,   .TILE0_TXDIFFCTRL0_IN           ( B2B_TXDIFFCTRL)
		             ,   .TILE0_TXDIFFCTRL1_IN           (RING_TXDIFFCTRL)
		             ,   .TILE0_TXPREEMPHASIS0_IN        ( B2B_TXPREEMPHASIS)
		             ,   .TILE0_TXPREEMPHASIS1_IN        (RING_TXPREEMPHASIS)
		            );
endcase endgenerate

////////////////////////////////////////////////////////////////////////////////
// High-speed serial multiplexers for board-to-board connections
////////////////////////////////////////////////////////////////////////////////

// Connect boards together in the system.
generate for (i = 0; i < 2; i = i + 1)
	begin : b2b_hss_multiplexers
		spio_hss_multiplexer#( .CLOCK_CORRECTION_INTERVAL      (B2B_CLOCK_CORRECTION_INTERVAL)
		                     , .CLOCK_CORRECTION_INTERVAL_BITS (B2B_CLOCK_CORRECTION_INTERVAL_BITS)
		                     , .NUM_HANDSHAKES                 (B2B_NUM_HANDSHAKES)
		                     , .NUM_HANDSHAKES_BITS            (B2B_NUM_HANDSHAKES_BITS)
		                     )
		b2b_hss_multiplexer_i( .CLK_IN                         (b2b_usrclk2_i)
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
		                     , .TX_PKT0_DATA_IN                (b2b_pkt_txdata_i[i][i ? 7 : 0])
		                     , .TX_PKT0_VLD_IN                 (b2b_pkt_txvld_i[i][i ? 7 : 0])
		                     , .TX_PKT0_RDY_OUT                (b2b_pkt_txrdy_i[i][i ? 7 : 0])
		                     , .TX_PKT1_DATA_IN                (b2b_pkt_txdata_i[i][i ? 6 : 1])
		                     , .TX_PKT1_VLD_IN                 (b2b_pkt_txvld_i[i][i ? 6 : 1])
		                     , .TX_PKT1_RDY_OUT                (b2b_pkt_txrdy_i[i][i ? 6 : 1])
		                     , .TX_PKT2_DATA_IN                (b2b_pkt_txdata_i[i][i ? 5 : 2])
		                     , .TX_PKT2_VLD_IN                 (b2b_pkt_txvld_i[i][i ? 5 : 2])
		                     , .TX_PKT2_RDY_OUT                (b2b_pkt_txrdy_i[i][i ? 5 : 2])
		                     , .TX_PKT3_DATA_IN                (b2b_pkt_txdata_i[i][i ? 4 : 3])
		                     , .TX_PKT3_VLD_IN                 (b2b_pkt_txvld_i[i][i ? 4 : 3])
		                     , .TX_PKT3_RDY_OUT                (b2b_pkt_txrdy_i[i][i ? 4 : 3])
		                     , .TX_PKT4_DATA_IN                (b2b_pkt_txdata_i[i][i ? 3 : 4])
		                     , .TX_PKT4_VLD_IN                 (b2b_pkt_txvld_i[i][i ? 3 : 4])
		                     , .TX_PKT4_RDY_OUT                (b2b_pkt_txrdy_i[i][i ? 3 : 4])
		                     , .TX_PKT5_DATA_IN                (b2b_pkt_txdata_i[i][i ? 2 : 5])
		                     , .TX_PKT5_VLD_IN                 (b2b_pkt_txvld_i[i][i ? 2 : 5])
		                     , .TX_PKT5_RDY_OUT                (b2b_pkt_txrdy_i[i][i ? 2 : 5])
		                     , .TX_PKT6_DATA_IN                (b2b_pkt_txdata_i[i][i ? 1 : 6])
		                     , .TX_PKT6_VLD_IN                 (b2b_pkt_txvld_i[i][i ? 1 : 6])
		                     , .TX_PKT6_RDY_OUT                (b2b_pkt_txrdy_i[i][i ? 1 : 6])
		                     , .TX_PKT7_DATA_IN                (b2b_pkt_txdata_i[i][i ? 0 : 7])
		                     , .TX_PKT7_VLD_IN                 (b2b_pkt_txvld_i[i][i ? 0 : 7])
		                     , .TX_PKT7_RDY_OUT                (b2b_pkt_txrdy_i[i][i ? 0 : 7])
		                     , .RX_PKT0_DATA_OUT               (b2b_pkt_rxdata_i[i][i ? 7 : 0])
		                     , .RX_PKT0_VLD_OUT                (b2b_pkt_rxvld_i[i][i ? 7 : 0])
		                     , .RX_PKT0_RDY_IN                 (b2b_pkt_rxrdy_i[i][i ? 7 : 0])
		                     , .RX_PKT1_DATA_OUT               (b2b_pkt_rxdata_i[i][i ? 6 : 1])
		                     , .RX_PKT1_VLD_OUT                (b2b_pkt_rxvld_i[i][i ? 6 : 1])
		                     , .RX_PKT1_RDY_IN                 (b2b_pkt_rxrdy_i[i][i ? 6 : 1])
		                     , .RX_PKT2_DATA_OUT               (b2b_pkt_rxdata_i[i][i ? 5 : 2])
		                     , .RX_PKT2_VLD_OUT                (b2b_pkt_rxvld_i[i][i ? 5 : 2])
		                     , .RX_PKT2_RDY_IN                 (b2b_pkt_rxrdy_i[i][i ? 5 : 2])
		                     , .RX_PKT3_DATA_OUT               (b2b_pkt_rxdata_i[i][i ? 4 : 3])
		                     , .RX_PKT3_VLD_OUT                (b2b_pkt_rxvld_i[i][i ? 4 : 3])
		                     , .RX_PKT3_RDY_IN                 (b2b_pkt_rxrdy_i[i][i ? 4 : 3])
		                     , .RX_PKT4_DATA_OUT               (b2b_pkt_rxdata_i[i][i ? 3 : 4])
		                     , .RX_PKT4_VLD_OUT                (b2b_pkt_rxvld_i[i][i ? 3 : 4])
		                     , .RX_PKT4_RDY_IN                 (b2b_pkt_rxrdy_i[i][i ? 3 : 4])
		                     , .RX_PKT5_DATA_OUT               (b2b_pkt_rxdata_i[i][i ? 2 : 5])
		                     , .RX_PKT5_VLD_OUT                (b2b_pkt_rxvld_i[i][i ? 2 : 5])
		                     , .RX_PKT5_RDY_IN                 (b2b_pkt_rxrdy_i[i][i ? 2 : 5])
		                     , .RX_PKT6_DATA_OUT               (b2b_pkt_rxdata_i[i][i ? 1 : 6])
		                     , .RX_PKT6_VLD_OUT                (b2b_pkt_rxvld_i[i][i ? 1 : 6])
		                     , .RX_PKT6_RDY_IN                 (b2b_pkt_rxrdy_i[i][i ? 1 : 6])
		                     , .RX_PKT7_DATA_OUT               (b2b_pkt_rxdata_i[i][i ? 0 : 7])
		                     , .RX_PKT7_VLD_OUT                (b2b_pkt_rxvld_i[i][i ? 0 : 7])
		                     , .RX_PKT7_RDY_IN                 (b2b_pkt_rxrdy_i[i][i ? 0 : 7])
		                       // top-level control inputs
		                     , .SCRMBL_IDL_DAT                 (scrmbl_idl_dat_i[i])
		                       // High-level protocol performance counters
		                     , .REG_WRITE_IN                   (b2b_reg_write_i[i])
		                     , .REG_ADDR_IN                    (b2b_reg_addr_i[i])
		                     , .REG_READ_DATA_OUT              (b2b_reg_read_data_i[i])
		                     , .REG_WRITE_DATA_IN              (b2b_reg_write_data_i[i])
		                     );
	end
endgenerate

////////////////////////////////////////////////////////////////////////////////
// High-speed serial multiplexers for peripheral connections
////////////////////////////////////////////////////////////////////////////////

// Connect to peripherals
generate if (INCLUDE_PERIPH_SUPPORT)
	spio_hss_multiplexer   #( .CLOCK_CORRECTION_INTERVAL      (PERIPH_CLOCK_CORRECTION_INTERVAL)
        	                , .CLOCK_CORRECTION_INTERVAL_BITS (PERIPH_CLOCK_CORRECTION_INTERVAL_BITS)
                	        , .NUM_HANDSHAKES                 (PERIPH_NUM_HANDSHAKES)
                        	, .NUM_HANDSHAKES_BITS            (PERIPH_NUM_HANDSHAKES_BITS)
	                        )
	periph_hss_multiplexer_i( .CLK_IN                         (periph_usrclk2_i)
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
	                          // top-level control inputs
		                , .SCRMBL_IDL_DAT                 (scrmbl_idl_dat_i[2])
	                          // High-level protocol performance counters
                                , .REG_WRITE_IN                   (periph_reg_write_i)
                                , .REG_ADDR_IN                    (periph_reg_addr_i)
                                , .REG_READ_DATA_OUT              (periph_reg_read_data_i)
	                        , .REG_WRITE_DATA_IN              (periph_reg_write_data_i)
	                        );
endgenerate


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
		
		// SpiNNaker link (synchronous) fast clock packet interfaces
		wire [`PKT_BITS-1:0] slfc_pkt_rxdata_i;
		wire                 slfc_pkt_rxvld_i;
		wire                 slfc_pkt_rxrdy_i;
		wire [`PKT_BITS-1:0] slfc_pkt_txdata_i;
		wire                 slfc_pkt_txvld_i;
		wire                 slfc_pkt_txrdy_i;



		// FPGA -> SpiNNaker
		spio_spinnaker_link_sender
		spio_spinnaker_link_sender_i( .CLK_IN           (spinnaker_link_clk0_i)
		                            , .RESET_IN         (spinnaker_link_reset_i[(i*2) + 1])
		                              // Synchronous packet interface
		                            , .PKT_DATA_IN      (slfc_pkt_txdata_i)
		                            , .PKT_VLD_IN       (slfc_pkt_txvld_i)
		                            , .PKT_RDY_OUT      (slfc_pkt_txrdy_i)
		                            // SpiNNaker link asynchronous interface
		                            , .SL_DATA_2OF7_OUT (sl_out_data_i[i])
		                            , .SL_ACK_IN        (synced_sl_out_ack_i)
		                            );
		
		// SpiNNaker -> FPGA
		spio_spinnaker_link_receiver
		spio_spinnaker_link_receiver_i( .CLK_IN           (spinnaker_link_clk0_i)
		                              , .RESET_IN         (spinnaker_link_reset_i[(i*2) + 0])
		                              // SpiNNaker link asynchronous interface
		                              , .SL_DATA_2OF7_IN  (synced_sl_in_data_i)
		                              , .SL_ACK_OUT       (sl_in_ack_i[i])
		                                // Synchronous packet interface
		                              , .PKT_DATA_OUT     (slfc_pkt_rxdata_i)
		                              , .PKT_VLD_OUT      (slfc_pkt_rxvld_i)
		                              , .PKT_RDY_IN       (slfc_pkt_rxrdy_i)
		                              );
		
		// packet synchronisers (fast clock <-> normal clock)
		spio_hss_multiplexer_pkt_fifo_sync
		spio_hss_multiplexer_pkt_fifo_sync_i ( .WCLK_IN          (spinnaker_link_clk0_i)
		                                     , .RCLK_IN          (spinnaker_link_clk1_i)
		                                     , .RESET_IN         (spinnaker_link_reset_i[(i*2) + 0])
						     // fast clock (write) packet interface
 						     , .SFI_DATA_IN      (slfc_pkt_rxdata_i)
						     , .SFI_VLD_IN       (slfc_pkt_rxvld_i)
						     , .SFI_RDY_OUT      (slfc_pkt_rxrdy_i)
						     // normal clock (read) packet interface
 						     , .SFO_DATA_OUT     (sl_pkt_rxdata_i[i])
						     , .SFO_VLD_OUT      (sl_pkt_rxvld_i[i])
						     , .SFO_RDY_IN       (sl_pkt_rxrdy_i[i])
						     );

		spio_hss_multiplexer_pkt_fifo_sync
		spio_hss_multiplexer_pkt_fifo_sync2_i ( .WCLK_IN          (spinnaker_link_clk1_i)
		                                      , .RCLK_IN          (spinnaker_link_clk0_i)
		                                      , .RESET_IN         (spinnaker_link_reset_i[(i*2) + 1])

						      // normal clock (write) packet interface
 				     		      , .SFI_DATA_IN      (sl_pkt_txdata_i[i])
						      , .SFI_VLD_IN       (sl_pkt_txvld_i[i])
						      , .SFI_RDY_OUT      (sl_pkt_txrdy_i[i])
						      // fast clock (read) packet interface
 						      , .SFO_DATA_OUT     (slfc_pkt_txdata_i)
						      , .SFO_VLD_OUT      (slfc_pkt_txvld_i)
						      , .SFO_RDY_IN       (slfc_pkt_txrdy_i)
						      );

		// async flit synchronisers
		spio_spinnaker_link_sync#(.SIZE(1))
		spio_spinnaker_link_sync_i( .CLK_IN (spinnaker_link_clk0_i)
		                          , .IN     (sl_out_ack_i[i])
		                          , .OUT    (synced_sl_out_ack_i)
		                          );
		
		spio_spinnaker_link_sync#(.SIZE(7))
		spio_spinnaker_link_sync2_i( .CLK_IN (spinnaker_link_clk0_i)
		                           , .IN      (sl_in_data_i[i])
		                           , .OUT     (synced_sl_in_data_i)
		                           );
		
	end
endgenerate


////////////////////////////////////////////////////////////////////////////////
// Routing of SpiNNaker packets from SpiNNaker chips
////////////////////////////////////////////////////////////////////////////////

// Route all packets from chips to the first board-to-board link rather than
// peripheral except those with a certain key when a peripheral is connected.
generate if (INCLUDE_PERIPH_SUPPORT)
	for (i = 0; i < `NUM_CHANS; i = i + 1)
		begin : spinnaker_rx_link_routing
			// Add an interface between the 75 MHz board-to-board links and 37.5 MHz
			// peripheral links 
			wire [`PKT_BITS-1:0] fast_periph_pkt_txdata_i;
			wire                 fast_periph_pkt_txvld_i;
			wire                 fast_periph_pkt_txrdy_i;
			spio_link_speed_halver #( .PKT_BITS(`PKT_BITS))
			spio_link_speed_halver_i( .RESET_IN(arbiter_reset_i)
			                        , .SCLK_IN(periph_usrclk2_i)
			                        , .FCLK_IN(   b2b_usrclk2_i)
			                          // Incoming signals (on CLK_IN)
			                        , .DATA_IN(fast_periph_pkt_txdata_i)
			                        , .VLD_IN( fast_periph_pkt_txvld_i)
			                        , .RDY_OUT(fast_periph_pkt_txrdy_i)
			                          // Outgoing signals (on CLK2_IN)
			                        , .DATA_OUT(periph_pkt_txdata_i[i])
			                        , .VLD_OUT( periph_pkt_txvld_i[i])
			                        , .RDY_IN(  periph_pkt_txrdy_i[i])
			                        );
			
			// Only match non-emergency routed multicast packets
			wire is_mc_packet_i = (sl_pkt_rxdata_i[i][0+:8]  & 8'b11110000) == 8'b00000000;
			wire key_matches_i  = (sl_pkt_rxdata_i[i][8+:32] & periph_mc_mask_i) == periph_mc_key_i;
			
			// The output ports from the switch (which must be broken onto their various
			// buses)
			wire [(`PKT_BITS*2)-1:0] switch_out_data_i;
			wire [1:0]               switch_out_vld_i;
			wire [1:0]               switch_out_rdy_i;
			
			// Cause the switch to drop the current packet.
			wire drop_i;
			
			// Route packets arriving from the first bank of spinnaker chips.
			spio_switch #( .PKT_BITS(`PKT_BITS)
			             , .NUM_PORTS(2)
			             )
			spio_switch_i( .CLK_IN(b2b_usrclk2_i)
			             , .RESET_IN(arbiter_reset_i)
			             // Input port (from SpiNNaker chips)
			             , .IN_DATA_IN           (sl_pkt_rxdata_i[i])
			             , .IN_OUTPUT_SELECT_IN  ( ( is_mc_packet_i
			                                       & key_matches_i
			                                       & periph_handshake_complete_i
			                                       ) ? 2'b10 : 2'b01 
			                                     )
			             , .IN_VLD_IN            (sl_pkt_rxvld_i[i])
			             , .IN_RDY_OUT           (sl_pkt_rxrdy_i[i])
			             // Output ports (to b2b and periph links)
			             , .OUT_DATA_OUT         (switch_out_data_i)
			             , .OUT_VLD_OUT          (switch_out_vld_i)
			             , .OUT_RDY_IN           (switch_out_rdy_i)
			             // Output blocking status
			             , .BLOCKED_OUTPUTS_OUT  (switch_blocked_outputs_i[i])
			             , .SELECTED_OUTPUTS_OUT (switch_selected_outputs_i[i])
			             // Force packet drop (disabled, for now)
			             , .DROP_IN              (drop_i)
			             // Dropped packet port
			             , .DROPPED_DATA_OUT     (switch_dropped_data_i[i])
			             , .DROPPED_OUTPUTS_OUT  (switch_dropped_outputs_i[i])
			             , .DROPPED_VLD_OUT      (switch_dropped_vld_i[i])
			             );
			
			
			// XXX: Temporary solution to preventing blocked streams due to disconnected
			// devices: drop packets whenever blocked while also being known to be
			// disconnected.
			assign drop_i = |(switch_blocked_outputs_i[i] & { !periph_handshake_complete_i
			                                                , !b2b_handshake_complete_i[0]
			                                                });
			
			
			// Connect switch's first output port to b2b link
			assign b2b_pkt_txdata_i[0][i] = switch_out_data_i[0*`PKT_BITS+:`PKT_BITS];
			assign b2b_pkt_txvld_i[0][i]  = switch_out_vld_i[0];
			assign switch_out_rdy_i[0]    = b2b_pkt_txrdy_i[0][i];
			
			// Connect switch's second output port to peripheral link
			assign fast_periph_pkt_txdata_i = switch_out_data_i[1*`PKT_BITS+:`PKT_BITS];
			assign fast_periph_pkt_txvld_i  = switch_out_vld_i[1];
			assign switch_out_rdy_i[1]      = fast_periph_pkt_txrdy_i;
			
			// Connect the second bank of spinnaker chips straight back to the
			// second board-to-board link.
			assign b2b_pkt_txdata_i[1][i] = sl_pkt_rxdata_i[i+8];
			assign b2b_pkt_txvld_i[1][i]  = sl_pkt_rxvld_i[i+8];
			assign sl_pkt_rxrdy_i[i+8]    = b2b_pkt_txrdy_i[1][i];
		end
else
	for (i = 0; i < `NUM_CHANS; i = i + 1)
		begin : spinnaker_rx_link_bypass_routing
			// Connect the spinnaker chips straight to the board-to-board link
			assign b2b_pkt_txdata_i[0][i] = sl_pkt_rxdata_i[i];
			assign b2b_pkt_txvld_i[0][i]  = sl_pkt_rxvld_i[i];
			assign sl_pkt_rxrdy_i[i]      = b2b_pkt_txrdy_i[0][i];
			assign b2b_pkt_txdata_i[1][i] = sl_pkt_rxdata_i[i+8];
			assign b2b_pkt_txvld_i[1][i]  = sl_pkt_rxvld_i[i+8];
			assign sl_pkt_rxrdy_i[i+8]    = b2b_pkt_txrdy_i[1][i];
		
			// and invalidate the periph links
			assign periph_pkt_txvld_i[i]  = 1'b0;
		end
endgenerate


////////////////////////////////////////////////////////////////////////////////
// Arbitration of packets for SpiNNaker chip outputs
////////////////////////////////////////////////////////////////////////////////

generate if (INCLUDE_PERIPH_SUPPORT)
	for (i = 0; i < `NUM_CHANS; i = i + 1)
		begin : spinnaker_tx_link_arbitration
			// Add an interface between the 37.5 MHz peripheral links and 75 MHz
			// board-to-board links
			wire [`PKT_BITS-1:0] fast_periph_pkt_rxdata_i;
			wire                 fast_periph_pkt_rxvld_i;
			wire                 fast_periph_pkt_rxrdy_i;
			spio_link_speed_doubler #( .PKT_BITS(`PKT_BITS))
			spio_link_speed_doubler_i( .RESET_IN(arbiter_reset_i)
			                         , .SCLK_IN(periph_usrclk2_i)
			                         , .FCLK_IN(   b2b_usrclk2_i)
			                           // Incoming signals (on CLK_IN)
			                         , .DATA_IN(periph_pkt_rxdata_i[i])
			                         , .VLD_IN( periph_pkt_rxvld_i[i])
			                         , .RDY_OUT(periph_pkt_rxrdy_i[i])
			                           // Outgoing signals (on CLK2_IN)
			                         , .DATA_OUT(fast_periph_pkt_rxdata_i)
			                         , .VLD_OUT( fast_periph_pkt_rxvld_i)
			                         , .RDY_IN(  fast_periph_pkt_rxrdy_i)
			                         );
			
			// Arbitrate the first board-to-board link with the peripheral link
			spio_rr_arbiter #( .PKT_BITS(`PKT_BITS))
			spio_rr_arbiter_i( .CLK_IN(b2b_usrclk2_i)
			                 , .RESET_IN(arbiter_reset_i)
			                   // Input ports
			                 , .DATA0_IN(   b2b_pkt_rxdata_i[0][i])
			                 , .VLD0_IN(    b2b_pkt_rxvld_i[0][i])
			                 , .RDY0_OUT(   b2b_pkt_rxrdy_i[0][i])
			                 , .DATA1_IN(fast_periph_pkt_rxdata_i)
			                 , .VLD1_IN( fast_periph_pkt_rxvld_i)
			                 , .RDY1_OUT(fast_periph_pkt_rxrdy_i)
			                   // Output port where the merged stream will be sent
			                 , .DATA_OUT(sl_pkt_txdata_i[i])
			                 , .VLD_OUT(sl_pkt_txvld_i[i])
			                 , .RDY_IN(sl_pkt_txrdy_i[i])
			                 );
			
			// The second board-to-board connection worth of links should be directly
			// connected since there is currently no contention for these link.
			assign sl_pkt_txdata_i[i+8]  = b2b_pkt_rxdata_i[1][i];
			assign sl_pkt_txvld_i[i+8]   = b2b_pkt_rxvld_i[1][i];
			assign b2b_pkt_rxrdy_i[1][i] = sl_pkt_txrdy_i[i+8];
		end
else
	for (i = 0; i < `NUM_CHANS; i = i + 1)
		begin : spinnaker_tx_link_bypass_arbitration
			// Connect the spinnaker chips straight to the board-to-board link
			assign sl_pkt_txdata_i[i]    = b2b_pkt_rxdata_i[0][i];
			assign sl_pkt_txvld_i[i]     = b2b_pkt_rxvld_i[0][i];
			assign b2b_pkt_rxrdy_i[0][i] = sl_pkt_txrdy_i[i];
			assign sl_pkt_txdata_i[i+8]  = b2b_pkt_rxdata_i[1][i];
			assign sl_pkt_txvld_i[i+8]   = b2b_pkt_rxvld_i[1][i];
			assign b2b_pkt_rxrdy_i[1][i] = sl_pkt_txrdy_i[i+8];
		
			// and invalidate the periph links
			assign periph_pkt_rxrdy_i[i] = 1'b0;
		end
endgenerate


////////////////////////////////////////////////////////////////////////////////
// SPI Interface and register address-bank decoding
////////////////////////////////////////////////////////////////////////////////

// Buffered SPI Signals
wire spi_sclk_i;
wire spi_mosi_i;
wire spi_miso_i;
wire spi_nss_i;

// Synchronised SPI signals
wire synced_spi_sclk_i;
wire synced_spi_mosi_i;
wire synced_spi_nss_i;

// Decoded address
wire [15:2] all_reg_addr_i;

// Buffer (and tristate) the SPI signals
IBUF  spi_nss_buf  (.I  (SPI_NSS_IN),   .O (spi_nss_i));
IBUF  spi_sclk_buf (.I  (SPI_SCLK_IN),  .O (spi_sclk_i));
IBUF  spi_mosi_buf (.I  (SPI_MOSI_IN),  .O (spi_mosi_i));
IOBUF spi_miso_buf (.IO (SPI_MISO_OUT), .I (spi_miso_i), .T (spi_nss_i), .O());

// Synchronising flip-flops
spinnaker_fpgas_sync #( .SIZE(1)
                      )
sync_nss_i            ( .CLK_IN(spi_clk_i)
                      , .IN(spi_nss_i)
                      , .OUT(synced_spi_nss_i)
                      );
spinnaker_fpgas_sync #( .SIZE(1)
                      )
sync_sclk_i           ( .CLK_IN(spi_clk_i)
                      , .IN(spi_sclk_i)
                      , .OUT(synced_spi_sclk_i)
                      );
spinnaker_fpgas_sync #( .SIZE(1)
                      )
sync_mosi_i           ( .CLK_IN(spi_clk_i)
                      , .IN(spi_mosi_i)
                      , .OUT(synced_spi_mosi_i)
                      );

// SPI Interface
spinnaker_fpgas_spi #( .ADDR_BITS(32)
                     , .VAL_BITS(32)
                     )
spinnaker_fpgas_spi_i( // System reset and clock
                       .RESET_IN (spi_reset_i)
                     , .CLK_IN   (spi_clk_i)
                       // SPI Signals
                     , .SCLK_IN  (synced_spi_sclk_i)
                     , .MOSI_IN  (synced_spi_mosi_i)
                     , .MISO_OUT (spi_miso_i)
                     , .NSS_IN   (synced_spi_nss_i)
                       
                       // Peek/Poke interface.
                     , .ADDRESS_OUT     (reg_addr_i)
                     , .READ_OUT        (reg_read_i)
                     , .WRITE_OUT       (reg_write_i)
                     , .WRITE_VALUE_OUT (reg_write_data_i)
                     , .READ_VALUE_IN   (reg_read_data_i)
                     );

// Address decode
spinnaker_fpgas_spi_address_decode #( .SPI_ADDR_BITS(32)
                                    , .VAL_BITS(32)
                                    )
spinnaker_fpgas_spi_address_decode_i( // Un-decoded interface
                                      .SPI_ADDR_IN        (reg_addr_i)
                                    , .SPI_READ_IN        (reg_read_i)
                                    , .SPI_WRITE_IN       (reg_write_i)
                                    , .SPI_READ_VALUE_OUT (reg_read_data_i)
                                      // Device signals
                                    , .ADDR_OUT(all_reg_addr_i)
                                      // Per-device signals
                                    ,    .B2B_READ_OUT() // Unused
                                    , .PERIPH_READ_OUT() // Unused
                                    ,   .RING_READ_OUT() // Unused
                                    ,    .TOP_READ_OUT() // Unused
                                    ,    .CTR_READ_OUT() // Unused
                                    ,    .B2B_WRITE_OUT({ b2b_reg_write_i[1]
                                                        , b2b_reg_write_i[0]
                                                        })
                                    , .PERIPH_WRITE_OUT(periph_reg_write_i)
                                    ,   .RING_WRITE_OUT(ring_reg_write_i)
                                    ,    .TOP_WRITE_OUT(top_reg_write_i)
                                    ,    .CTR_WRITE_OUT() // Unused
                                    ,    .B2B_READ_VALUE_IN({ b2b_reg_read_data_i[1]
                                                            , b2b_reg_read_data_i[0]
                                                            })
                                    , .PERIPH_READ_VALUE_IN(periph_reg_read_data_i)
                                    ,   .RING_READ_VALUE_IN(ring_reg_read_data_i)
                                    ,    .TOP_READ_VALUE_IN(top_reg_read_data_i)
                                    ,    .CTR_READ_VALUE_IN({ pkt_ctr_read_data_i[1]
                                                            , pkt_ctr_read_data_i[0]
                                                            })
                                    );

// Truncate decoded addresses and distribute to all devices
assign    b2b_reg_addr_i[1] = all_reg_addr_i[`REGA_BITS-1+2:2];
assign    b2b_reg_addr_i[0] = all_reg_addr_i[`REGA_BITS-1+2:2];
assign periph_reg_addr_i    = all_reg_addr_i[`REGA_BITS-1+2:2];
assign   ring_reg_addr_i    = all_reg_addr_i[`REGA_BITS-1+2:2];
assign    top_reg_addr_i    = all_reg_addr_i;
assign    pkt_ctr_addr_i[1] = all_reg_addr_i[`CTRA_BITS-1+2:2];
assign    pkt_ctr_addr_i[0] = all_reg_addr_i[`CTRA_BITS-1+2:2];


// Distribute write data to all devices
assign    b2b_reg_write_data_i[1] = reg_write_data_i;
assign    b2b_reg_write_data_i[0] = reg_write_data_i;
assign periph_reg_write_data_i    = reg_write_data_i;
assign   ring_reg_write_data_i    = reg_write_data_i;
assign    top_reg_write_data_i    = reg_write_data_i;


////////////////////////////////////////////////////////////////////////////////
// Top-level register bank
////////////////////////////////////////////////////////////////////////////////

spinnaker_fpgas_reg_bank
spinnaker_fpgas_reg_bank_i( .CLK_IN         (reg_bank_clk_i)
                          , .RESET_IN       (reg_bank_reset_i)
                            // Register bank interface
                          , .WRITE_IN       (top_reg_write_i)
                          , .ADDR_IN        (top_reg_addr_i)
                          , .WRITE_DATA_IN  (top_reg_write_data_i)
                          , .READ_DATA_OUT  (top_reg_read_data_i)
                            // Regsiters
                          , .VERSION_IN     (VERSION)
                          , .FLAGS_IN       ({ DEBUG_CHIPSCOPE_VIO[0]
                                             , INCLUDE_PERIPH_SUPPORT[0]
                                             , INCLUDE_RING_SUPPORT[0]
                                             , NORTH_SOUTH_ON_FRONT[0]
                                             , FPGA_ID[1:0]
                                             }
                                            )
                          , .SPINNAKER_LINK_ENABLE(spinnaker_link_enable_i)
                          , .PERIPH_MC_KEY  (periph_mc_key_i)
                          , .PERIPH_MC_MASK (periph_mc_mask_i)
                          , .SCRMBL_IDL_DAT (scrmbl_idl_dat_i)
                          );


////////////////////////////////////////////////////////////////////////////////
// packet counters
////////////////////////////////////////////////////////////////////////////////

spio_packet_counter
spio_pkt_ctr_tx( .CLK_IN    (pkt_ctr_clk_i)
               , .RESET_IN  (pkt_ctr_reset_i)
               , .pkt_vld0  (sl_pkt_txvld_i[0])
               , .pkt_rdy0  (sl_pkt_txrdy_i[0])
               , .pkt_vld1  (sl_pkt_txvld_i[1])
               , .pkt_rdy1  (sl_pkt_txrdy_i[1])
               , .pkt_vld2  (sl_pkt_txvld_i[2])
               , .pkt_rdy2  (sl_pkt_txrdy_i[2])
               , .pkt_vld3  (sl_pkt_txvld_i[3])
               , .pkt_rdy3  (sl_pkt_txrdy_i[3])
               , .pkt_vld4  (sl_pkt_txvld_i[4])
               , .pkt_rdy4  (sl_pkt_txrdy_i[4])
               , .pkt_vld5  (sl_pkt_txvld_i[5])
               , .pkt_rdy5  (sl_pkt_txrdy_i[5])
               , .pkt_vld6  (sl_pkt_txvld_i[6])
               , .pkt_rdy6  (sl_pkt_txrdy_i[6])
               , .pkt_vld7  (sl_pkt_txvld_i[7])
               , .pkt_rdy7  (sl_pkt_txrdy_i[7])
               , .pkt_vld8  (sl_pkt_txvld_i[8])
               , .pkt_rdy8  (sl_pkt_txrdy_i[8])
               , .pkt_vld9  (sl_pkt_txvld_i[9])
               , .pkt_rdy9  (sl_pkt_txrdy_i[9])
               , .pkt_vld10 (sl_pkt_txvld_i[10])
               , .pkt_rdy10 (sl_pkt_txrdy_i[10])
               , .pkt_vld11 (sl_pkt_txvld_i[11])
               , .pkt_rdy11 (sl_pkt_txrdy_i[11])
               , .pkt_vld12 (sl_pkt_txvld_i[12])
               , .pkt_rdy12 (sl_pkt_txrdy_i[12])
               , .pkt_vld13 (sl_pkt_txvld_i[13])
               , .pkt_rdy13 (sl_pkt_txrdy_i[13])
               , .pkt_vld14 (sl_pkt_txvld_i[14])
               , .pkt_rdy14 (sl_pkt_txrdy_i[14])
               , .pkt_vld15 (sl_pkt_txvld_i[15])
               , .pkt_rdy15 (sl_pkt_txrdy_i[15])
               , .ctr_addr  (pkt_ctr_addr_i[0])
               , .ctr_data  (pkt_ctr_read_data_i[0])
               );

spio_packet_counter
spio_pkt_ctr_rx( .CLK_IN    (pkt_ctr_clk_i)
               , .RESET_IN  (pkt_ctr_reset_i)
               , .pkt_vld0  (sl_pkt_rxvld_i[0])
               , .pkt_rdy0  (sl_pkt_rxrdy_i[0])
               , .pkt_vld1  (sl_pkt_rxvld_i[1])
               , .pkt_rdy1  (sl_pkt_rxrdy_i[1])
               , .pkt_vld2  (sl_pkt_rxvld_i[2])
               , .pkt_rdy2  (sl_pkt_rxrdy_i[2])
               , .pkt_vld3  (sl_pkt_rxvld_i[3])
               , .pkt_rdy3  (sl_pkt_rxrdy_i[3])
               , .pkt_vld4  (sl_pkt_rxvld_i[4])
               , .pkt_rdy4  (sl_pkt_rxrdy_i[4])
               , .pkt_vld5  (sl_pkt_rxvld_i[5])
               , .pkt_rdy5  (sl_pkt_rxrdy_i[5])
               , .pkt_vld6  (sl_pkt_rxvld_i[6])
               , .pkt_rdy6  (sl_pkt_rxrdy_i[6])
               , .pkt_vld7  (sl_pkt_rxvld_i[7])
               , .pkt_rdy7  (sl_pkt_rxrdy_i[7])
               , .pkt_vld8  (sl_pkt_rxvld_i[8])
               , .pkt_rdy8  (sl_pkt_rxrdy_i[8])
               , .pkt_vld9  (sl_pkt_rxvld_i[9])
               , .pkt_rdy9  (sl_pkt_rxrdy_i[9])
               , .pkt_vld10 (sl_pkt_rxvld_i[10])
               , .pkt_rdy10 (sl_pkt_rxrdy_i[10])
               , .pkt_vld11 (sl_pkt_rxvld_i[11])
               , .pkt_rdy11 (sl_pkt_rxrdy_i[11])
               , .pkt_vld12 (sl_pkt_rxvld_i[12])
               , .pkt_rdy12 (sl_pkt_rxrdy_i[12])
               , .pkt_vld13 (sl_pkt_rxvld_i[13])
               , .pkt_rdy13 (sl_pkt_rxrdy_i[13])
               , .pkt_vld14 (sl_pkt_rxvld_i[14])
               , .pkt_rdy14 (sl_pkt_rxrdy_i[14])
               , .pkt_vld15 (sl_pkt_rxvld_i[15])
               , .pkt_rdy15 (sl_pkt_rxrdy_i[15])
               , .ctr_addr  (pkt_ctr_addr_i[1])
               , .ctr_data  (pkt_ctr_read_data_i[1])
               );


////////////////////////////////////////////////////////////////////////////////
// Chipscope virtual I/O
////////////////////////////////////////////////////////////////////////////////

// Virtual I/O connections to allow observation of useful values

generate if (DEBUG_CHIPSCOPE_VIO)
	begin : chipscope
		
		wire [35:0] chipscope_control_i;
		
		wire [63:0] chipscope_sync_in_i;
		
		// Chipscope Integrated CONtroller (ICON)
		chipscope_icon chipscope_icon_i (.CONTROL0 (chipscope_control_i));
		
		// Chipscope Virtual I/O (VIO) Port
		chipscope_vio chipscope_vio_i ( .CONTROL (chipscope_control_i)
		                              , .CLK     (b2b_usrclk2_i)
		                              , .SYNC_IN (chipscope_sync_in_i)
		                              );
		
		// SpiNNaker link RX ready/vld
		assign chipscope_sync_in_i[ 0+:16] = { sl_pkt_rxrdy_i[15]
		                                     , sl_pkt_rxrdy_i[14]
		                                     , sl_pkt_rxrdy_i[13]
		                                     , sl_pkt_rxrdy_i[12]
		                                     , sl_pkt_rxrdy_i[11]
		                                     , sl_pkt_rxrdy_i[10]
		                                     , sl_pkt_rxrdy_i[9]
		                                     , sl_pkt_rxrdy_i[8]
		                                     , sl_pkt_rxrdy_i[7]
		                                     , sl_pkt_rxrdy_i[6]
		                                     , sl_pkt_rxrdy_i[5]
		                                     , sl_pkt_rxrdy_i[4]
		                                     , sl_pkt_rxrdy_i[3]
		                                     , sl_pkt_rxrdy_i[2]
		                                     , sl_pkt_rxrdy_i[1]
		                                     , sl_pkt_rxrdy_i[0]
		                                     };
		assign chipscope_sync_in_i[16+:16] = { sl_pkt_rxvld_i[15]
		                                     , sl_pkt_rxvld_i[14]
		                                     , sl_pkt_rxvld_i[13]
		                                     , sl_pkt_rxvld_i[12]
		                                     , sl_pkt_rxvld_i[11]
		                                     , sl_pkt_rxvld_i[10]
		                                     , sl_pkt_rxvld_i[9]
		                                     , sl_pkt_rxvld_i[8]
		                                     , sl_pkt_rxvld_i[7]
		                                     , sl_pkt_rxvld_i[6]
		                                     , sl_pkt_rxvld_i[5]
		                                     , sl_pkt_rxvld_i[4]
		                                     , sl_pkt_rxvld_i[3]
		                                     , sl_pkt_rxvld_i[2]
		                                     , sl_pkt_rxvld_i[1]
		                                     , sl_pkt_rxvld_i[0]
		                                     };
		// SpiNNaker link TX ready/
		assign chipscope_sync_in_i[32+:16] = { sl_pkt_txrdy_i[15]
		                                     , sl_pkt_txrdy_i[14]
		                                     , sl_pkt_txrdy_i[13]
		                                     , sl_pkt_txrdy_i[12]
		                                     , sl_pkt_txrdy_i[11]
		                                     , sl_pkt_txrdy_i[10]
		                                     , sl_pkt_txrdy_i[9]
		                                     , sl_pkt_txrdy_i[8]
		                                     , sl_pkt_txrdy_i[7]
		                                     , sl_pkt_txrdy_i[6]
		                                     , sl_pkt_txrdy_i[5]
		                                     , sl_pkt_txrdy_i[4]
		                                     , sl_pkt_txrdy_i[3]
		                                     , sl_pkt_txrdy_i[2]
		                                     , sl_pkt_txrdy_i[1]
		                                     , sl_pkt_txrdy_i[0]
		                                     };
		assign chipscope_sync_in_i[48+:16] = { sl_pkt_txvld_i[15]
		                                     , sl_pkt_txvld_i[14]
		                                     , sl_pkt_txvld_i[13]
		                                     , sl_pkt_txvld_i[12]
		                                     , sl_pkt_txvld_i[11]
		                                     , sl_pkt_txvld_i[10]
		                                     , sl_pkt_txvld_i[9]
		                                     , sl_pkt_txvld_i[8]
		                                     , sl_pkt_txvld_i[7]
		                                     , sl_pkt_txvld_i[6]
		                                     , sl_pkt_txvld_i[5]
		                                     , sl_pkt_txvld_i[4]
		                                     , sl_pkt_txvld_i[3]
		                                     , sl_pkt_txvld_i[2]
		                                     , sl_pkt_txvld_i[1]
		                                     , sl_pkt_txvld_i[0]
		                                     };
		
		
	end
endgenerate

endmodule
