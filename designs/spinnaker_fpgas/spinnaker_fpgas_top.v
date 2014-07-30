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

`include "../../modules/spinnaker_link/spio_spinnaker_link.h"

`include "../../modules/hss_multiplexer/spio_hss_multiplexer_common.h"
`include "../../modules/hss_multiplexer/spio_hss_multiplexer_reg_bank.h"


module spinnaker_fpgas_top #( // Enable simulation mode for GTP tile
                              parameter SIMULATION = 0
                              // Speed up simulated reset of GTP tile
                            , parameter SIMULATION_GTPRESET_SPEEDUP = 0
                              // Enable chip-scope Virtual I/O interface (e.g.
                              // to access HSS multiplexer debug register banks)
                            , parameter DEBUG_CHIPSCOPE_VIO = 1
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
                              // Mask and key to select MC packets to forward to
                              // a connected peripheral.
                            , parameter PERIPH_MC_MASK = 32'h00000000
                            , parameter PERIPH_MC_KEY  = 32'hffffffff
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
                            , inout wire [15:0] SL0_INOUT
                            , inout wire [15:0] SL1_INOUT
                            , inout wire [15:0] SL2_INOUT
                            , inout wire [15:0] SL3_INOUT
                            , inout wire [15:0] SL4_INOUT
                            , inout wire [15:0] SL5_INOUT
                            , inout wire [15:0] SL6_INOUT
                            , inout wire [15:0] SL7_INOUT
                            , inout wire [15:0] SL8_INOUT
                            , inout wire [15:0] SL9_INOUT
                            , inout wire [15:0] SL10_INOUT
                            , inout wire [15:0] SL11_INOUT
                            , inout wire [15:0] SL12_INOUT
                            , inout wire [15:0] SL13_INOUT
                            , inout wire [15:0] SL14_INOUT
                            , inout wire [15:0] SL15_INOUT
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

localparam    B2B_TXDIFFCTRL = 4'b0010; // 495 mV
localparam PERIPH_TXDIFFCTRL = 4'b0000; // Default
localparam   RING_TXDIFFCTRL = 4'b0000; // Default

localparam    B2B_TXPREEMPHASIS = 3'b010;  // 1.7 dB
localparam PERIPH_TXPREEMPHASIS = 3'b000;  // Default
localparam   RING_TXPREEMPHASIS = 3'b000;  // Default


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
wire    b2b_hss_reset_i [1:0];
wire periph_hss_reset_i;
//wire   ring_hss_reset_i;

// Reset for SpiNNaker link blocks
wire spinnaker_link_reset_i;

// Reset for packet arbitration/routing blocks
wire arbiter_reset_i;

// Reset for LED control
wire led_reset_i;

// LED signals
wire red_led_i;
wire grn_led_i;

wire [1:0] fpga_id_i;

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

// HSS Multiplexer debug registers (for access by chipscope)
wire [`REGA_BITS-1:0]    b2b_reg_addr_i [1:0];
wire [`REGA_BITS-1:0] periph_reg_addr_i;
wire [`REGA_BITS-1:0]   ring_reg_addr_i;
wire [`REGD_BITS-1:0]    b2b_reg_data_i [1:0];
wire [`REGD_BITS-1:0] periph_reg_data_i;
wire [`REGD_BITS-1:0]   ring_reg_data_i;

// Routing (spio_switch) status signals
wire [1:0] switch_blocked_outputs_i  [`NUM_CHANS-1:0];
wire [1:0] switch_selected_outputs_i [`NUM_CHANS-1:0];

// Routing (spio_switch) packet dropping port
wire [`PKT_BITS-1:0]  switch_dropped_data_i    [`NUM_CHANS-1:0];
wire [1:0]            switch_dropped_outputs_i [`NUM_CHANS-1:0];
wire                  switch_dropped_vld_i     [`NUM_CHANS-1:0];


////////////////////////////////////////////////////////////////////////////////
// Reset
////////////////////////////////////////////////////////////////////////////////

IBUF reset_buf (.I (RESET_IN), .O (reset_i));

assign gtp_reset_i = reset_i;

assign clk_reset_i = reset_i | !plllkdet_i;

// HSS blocks are connected to the GTP blocks and so must wait until they have
// completely reset.
assign    b2b_hss_reset_i[0] =    !b2b_gtpresetdone_i[0] & !usrclks_stable_i;
assign    b2b_hss_reset_i[1] =    !b2b_gtpresetdone_i[1] & !usrclks_stable_i;
assign periph_hss_reset_i    = !periph_gtpresetdone_i    & !usrclks_stable_i;
//assign   ring_hss_reset_i    =   !ring_gtpresetdone_i    & !usrclks_stable_i;

assign spinnaker_link_reset_i = !usrclks_stable_i;

assign arbiter_reset_i = !usrclks_stable_i;

assign led_reset_i = !usrclks_stable_i;


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
assign ring_pkt_activity_i = 1'b0;



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
wire [15:0] sl_in_pins_i  [15:0];
wire [15:0] sl_out_pins_i [15:0];
IOBUF sl0_buf_i  [15:0] (.IO(SL0_INOUT),  .O(sl_in_pins_i[ 0]), .I(sl_out_pins_i[ 0]), .T(sl_tristate_i[ 0]));
IOBUF sl1_buf_i  [15:0] (.IO(SL1_INOUT),  .O(sl_in_pins_i[ 1]), .I(sl_out_pins_i[ 1]), .T(sl_tristate_i[ 1]));
IOBUF sl2_buf_i  [15:0] (.IO(SL2_INOUT),  .O(sl_in_pins_i[ 2]), .I(sl_out_pins_i[ 2]), .T(sl_tristate_i[ 2]));
IOBUF sl3_buf_i  [15:0] (.IO(SL3_INOUT),  .O(sl_in_pins_i[ 3]), .I(sl_out_pins_i[ 3]), .T(sl_tristate_i[ 3]));
IOBUF sl4_buf_i  [15:0] (.IO(SL4_INOUT),  .O(sl_in_pins_i[ 4]), .I(sl_out_pins_i[ 4]), .T(sl_tristate_i[ 4]));
IOBUF sl5_buf_i  [15:0] (.IO(SL5_INOUT),  .O(sl_in_pins_i[ 5]), .I(sl_out_pins_i[ 5]), .T(sl_tristate_i[ 5]));
IOBUF sl6_buf_i  [15:0] (.IO(SL6_INOUT),  .O(sl_in_pins_i[ 6]), .I(sl_out_pins_i[ 6]), .T(sl_tristate_i[ 6]));
IOBUF sl7_buf_i  [15:0] (.IO(SL7_INOUT),  .O(sl_in_pins_i[ 7]), .I(sl_out_pins_i[ 7]), .T(sl_tristate_i[ 7]));
IOBUF sl8_buf_i  [15:0] (.IO(SL8_INOUT),  .O(sl_in_pins_i[ 8]), .I(sl_out_pins_i[ 8]), .T(sl_tristate_i[ 8]));
IOBUF sl9_buf_i  [15:0] (.IO(SL9_INOUT),  .O(sl_in_pins_i[ 9]), .I(sl_out_pins_i[ 9]), .T(sl_tristate_i[ 9]));
IOBUF sl10_buf_i [15:0] (.IO(SL10_INOUT), .O(sl_in_pins_i[10]), .I(sl_out_pins_i[10]), .T(sl_tristate_i[10]));
IOBUF sl11_buf_i [15:0] (.IO(SL11_INOUT), .O(sl_in_pins_i[11]), .I(sl_out_pins_i[11]), .T(sl_tristate_i[11]));
IOBUF sl12_buf_i [15:0] (.IO(SL12_INOUT), .O(sl_in_pins_i[12]), .I(sl_out_pins_i[12]), .T(sl_tristate_i[12]));
IOBUF sl13_buf_i [15:0] (.IO(SL13_INOUT), .O(sl_in_pins_i[13]), .I(sl_out_pins_i[13]), .T(sl_tristate_i[13]));
IOBUF sl14_buf_i [15:0] (.IO(SL14_INOUT), .O(sl_in_pins_i[14]), .I(sl_out_pins_i[14]), .T(sl_tristate_i[14]));
IOBUF sl15_buf_i [15:0] (.IO(SL15_INOUT), .O(sl_in_pins_i[15]), .I(sl_out_pins_i[15]), .T(sl_tristate_i[15]));

generate for (i = 0; i < 16; i = i + 1)
	begin : spinnaker_link_buffering
		// Select the appropriate input signals given the link type
		assign sl_in_data_i[i] = (sl_types_i[2*i+:2] == HIGH_SL) ?  sl_in_pins_i[i][7:1] : sl_in_pins_i[i][15:9];
		assign sl_out_ack_i[i] = (sl_types_i[2*i+:2] == HIGH_SL) ?  sl_in_pins_i[i][8]   : sl_in_pins_i[i][0];
		
		// Duplicate the output signals since the tristate will ensure that they get
		// to the right pins
		assign sl_out_pins_i[i] = {2{sl_out_data_i[i], sl_in_ack_i[i]}};
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
assign periph_usrclk_i  = clk_150_i;
assign   ring_usrclk_i  = clk_300_i;

assign    b2b_usrclk2_i = clk_75_i;
assign periph_usrclk2_i = clk_37_5_i;
assign   ring_usrclk2_i = clk_75_i;

assign spinnaker_link_clk0_i = clk_150_i;
assign spinnaker_link_clk1_i = b2b_usrclk2_i;

assign led_clk_i = clk_75_i;


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
            ,   .TILE0_RXEQMIX0_IN              (B2B_RXEQMIX)
            ,   .TILE0_RXEQMIX1_IN              (B2B_RXEQMIX)
            ,   .TILE0_TXDIFFCTRL0_IN           (B2B_TXDIFFCTRL)
            ,   .TILE0_TXDIFFCTRL1_IN           (B2B_TXDIFFCTRL)
            ,   .TILE0_TXPREEMPHASIS0_IN        (B2B_TXPREEMPHASIS)
            ,   .TILE0_TXPREEMPHASIS1_IN        (B2B_TXPREEMPHASIS)
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
            ,   .TILE0_RXEQMIX0_IN              (PERIPH_RXEQMIX)
            ,   .TILE0_RXEQMIX1_IN              (  RING_RXEQMIX)
            ,   .TILE0_TXDIFFCTRL0_IN           (PERIPH_TXDIFFCTRL)
            ,   .TILE0_TXDIFFCTRL1_IN           (  RING_TXDIFFCTRL)
            ,   .TILE0_TXPREEMPHASIS0_IN        (PERIPH_TXPREEMPHASIS)
            ,   .TILE0_TXPREEMPHASIS1_IN        (  RING_TXPREEMPHASIS)
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
		                       // High-level protocol performance counters
		                     , .REG_ADDR_IN                    (b2b_reg_addr_i[i])
		                     , .REG_DATA_OUT                   (b2b_reg_data_i[i])
		                     );
	end
endgenerate

// Connect to peripherals
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
                          // High-level protocol performance counters
                        , .REG_ADDR_IN                    (periph_reg_addr_i)
                        , .REG_DATA_OUT                   (periph_reg_data_i)
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
		                            , .SL_DATA_2OF7_OUT (sl_out_data_i[i])
		                            , .SL_ACK_IN        (synced_sl_out_ack_i)
		                            );
		
		// SpiNNaker -> FPGA
		spio_spinnaker_link_receiver
		spio_spinnaker_link_receiver_i( .CLK_IN           (spinnaker_link_clk1_i)
		                              , .RESET_IN         (spinnaker_link_reset_i)
		                                // Packet counter interface
		                              , .COUNT_PACKET_OUT () // Unused
		                              // SpiNNaker link asynchronous interface
		                              , .SL_DATA_2OF7_IN  (synced_sl_in_data_i)
		                              , .SL_ACK_OUT       (sl_in_ack_i[i])
		                                // Synchronous packet interface
		                              , .PKT_DATA_OUT     (sl_pkt_rxdata_i[i])
		                              , .PKT_VLD_OUT      (sl_pkt_rxvld_i[i])
		                              , .PKT_RDY_IN       (sl_pkt_rxrdy_i[i])
		                              );
		
		// Synchronisers
		spio_spinnaker_link_sync#(.SIZE(1))
		spio_spinnaker_link_sync_i( .CLK_IN (spinnaker_link_clk1_i)
		                          , .IN     (sl_out_ack_i[i])
		                          , .OUT    (synced_sl_out_ack_i)
		                          );
		
		spio_spinnaker_link_sync2#(.SIZE(7))
		spio_spinnaker_link_sync2_i( .CLK0_IN (spinnaker_link_clk0_i)
		                           , .CLK1_IN (spinnaker_link_clk1_i)
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
generate for (i = 0; i < `NUM_CHANS; i = i + 1)
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
		wire key_matches_i  = (sl_pkt_rxdata_i[i][8+:32] & PERIPH_MC_MASK) == PERIPH_MC_KEY;
		
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
		
		
		// XXX: Tempoary solution to preventing blocked streams due to disconnected
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
		
		// Connect the second bank of spinnaker chips to straight back to the
		// second board-to-board link.
		assign b2b_pkt_txdata_i[1][i] = sl_pkt_rxdata_i[i+8];
		assign b2b_pkt_txvld_i[1][i]  = sl_pkt_rxvld_i[i+8];
		assign sl_pkt_rxrdy_i[i+8]    = b2b_pkt_txrdy_i[0][i];
	end
endgenerate


////////////////////////////////////////////////////////////////////////////////
// Arbitration of packets for SpiNNaker chip outputs
////////////////////////////////////////////////////////////////////////////////

generate for (i = 0; i < `NUM_CHANS; i = i + 1)
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
		assign sl_pkt_txdata_i[i+8]    = b2b_pkt_rxdata_i[1][i];
		assign sl_pkt_txvld_i[i+8]     = b2b_pkt_rxvld_i[1][i];
		assign b2b_pkt_rxrdy_i[1][i] = sl_pkt_txrdy_i[i+8];
	end
endgenerate


////////////////////////////////////////////////////////////////////////////////
// Chipscope virtual I/O
////////////////////////////////////////////////////////////////////////////////

// Virtual I/O connections to allow observation of useful values

generate if (DEBUG_CHIPSCOPE_VIO)
	begin : chipscope_enabled
		
		wire [35:0] chipscope_control_i;
		
		wire [191:0] chipscope_sync_in_i;
		wire [19:0]  chipscope_sync_out_i;
		
		// Chipscope Integrated CONtroller (ICON)
		chipscope_icon chipscope_icon_i (.CONTROL0 (chipscope_control_i));
		
		// Chipscope Virtual I/O (VIO) Port
		chipscope_vio chipscope_vio_i ( .CONTROL (chipscope_control_i)
		                              , .CLK     (b2b_usrclk2_i)
		                              , .SYNC_IN (chipscope_sync_in_i)
		                              , .SYNC_OUT(chipscope_sync_out_i)
		                              );
		
		// HSS Multiplexer debug registers
		assign    b2b_reg_addr_i[0] = chipscope_sync_out_i[0*`REGA_BITS+:`REGA_BITS];
		assign    b2b_reg_addr_i[1] = chipscope_sync_out_i[1*`REGA_BITS+:`REGA_BITS];
		assign periph_reg_addr_i    = chipscope_sync_out_i[2*`REGA_BITS+:`REGA_BITS];
		assign   ring_reg_addr_i    = chipscope_sync_out_i[3*`REGA_BITS+:`REGA_BITS];
		assign chipscope_sync_in_i[0*`REGD_BITS+:`REGD_BITS] =     b2b_reg_data_i[0];
		assign chipscope_sync_in_i[1*`REGD_BITS+:`REGD_BITS] =     b2b_reg_data_i[1];
		assign chipscope_sync_in_i[2*`REGD_BITS+:`REGD_BITS] = periph_reg_data_i;
		assign chipscope_sync_in_i[3*`REGD_BITS+:`REGD_BITS] =   ring_reg_data_i;
		
		// SpiNNaker link RX ready/vld
		assign chipscope_sync_in_i[4*`REGD_BITS+0 +:16] = { sl_pkt_rxrdy_i[15]
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
		assign chipscope_sync_in_i[4*`REGD_BITS+16+:16] = { sl_pkt_rxvld_i[15]
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
		// SpiNNaker link TX ready/vld
		assign chipscope_sync_in_i[4*`REGD_BITS+32+:16] = { sl_pkt_txrdy_i[15]
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
		assign chipscope_sync_in_i[4*`REGD_BITS+48+:16] = { sl_pkt_txvld_i[15]
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
else
	begin : chipscope_disabled
		
		// Tie off register addresses to an arbitrary register
		assign    b2b_reg_addr_i[0] = `VERS_REG;
		assign    b2b_reg_addr_i[1] = `VERS_REG;
		assign periph_reg_addr_i    = `VERS_REG;
		assign   ring_reg_addr_i    = `VERS_REG;
		
	end
endgenerate

endmodule
