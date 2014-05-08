/**
 * A module to multiplex eight bi-directional streams of packets over a single
 * high-speed-serial link on a Spartan-6 FPGA.
 */

module spio_hss_multiplexer #( // The width of each channel
                               parameter CHANNEL_WIDTH = 72
                               // The number of channels
                             , parameter NUM_CHANNELS = 8
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
                                 // TODO...
                             ,   input  wire [31:0] XXX_TXDATA_IN
                             ,   input  wire  [3:0] XXX_TXCHARISK_IN
                             ,   output wire        XXX_TXRDY_OUT
                             
                             ,   output wire [31:0] XXX_RXDATA_OUT
                             ,   output wire  [3:0] XXX_RXCHARISK_OUT
                             ,   output wire        XXX_RXVLD_OUT
                             );


////////////////////////////////////////////////////////////////////////////////
// Internal signals
////////////////////////////////////////////////////////////////////////////////

// Low-level serial transmission interface
wire [31:0] txdata_i;
wire  [3:0] txcharisk_i;
wire        txrdy_i;

wire [31:0] rxdata_i;
wire  [3:0] rxcharisk_i;
wire        rxvld_i;


////////////////////////////////////////////////////////////////////////////////
// Low-level serial TX/RX control blocks
////////////////////////////////////////////////////////////////////////////////

wire handshake_phase_i;

spio_hss_multiplexer_tx_control #( .CLOCK_CORRECTION_INTERVAL(CLOCK_CORRECTION_INTERVAL)
                                 , .CLOCK_CORRECTION_INTERVAL_BITS(CLOCK_CORRECTION_INTERVAL_BITS)
                                 )
spio_hss_multiplexer_tx_control_i( .CLK_IN                 (CLK_IN)
                                 , .RESET_IN               (RESET_IN)
                                 , .HANDSHAKE_COMPLETE_IN  (HANDSHAKE_COMPLETE_OUT)
                                 , .HANDSHAKE_PHASE_IN     (handshake_phase_i)
                                 , .TXDATA_OUT             (TXDATA_OUT)
                                 , .TXCHARISK_OUT          (TXCHARISK_OUT)
                                 , .TXDATA_IN              (txdata_i)
                                 , .TXCHARISK_IN           (txcharisk_i)
                                 , .TXRDY_OUT              (txrdy_i)
                                 );


spio_hss_multiplexer_rx_control #( .NUM_HANDSHAKES(NUM_HANDSHAKES)
                                 , .NUM_HANDSHAKES_BITS(NUM_HANDSHAKES_BITS)
                                 )
spio_hss_multiplexer_rx_control_i( .CLK_IN                 (CLK_IN)
                                 , .RESET_IN               (RESET_IN)
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


////////////////////////////////////////////////////////////////////////////////
// XXX: Tempoary data interface
////////////////////////////////////////////////////////////////////////////////

assign txdata_i      = XXX_TXDATA_IN;
assign txcharisk_i   = XXX_TXCHARISK_IN;
assign XXX_TXRDY_OUT = txrdy_i;

assign XXX_RXDATA_OUT    = rxdata_i;
assign XXX_RXCHARISK_OUT = rxcharisk_i;
assign XXX_RXVLD_OUT     = rxvld_i;


endmodule
