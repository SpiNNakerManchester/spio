/**
 * A module to multiplex eight bi-directional streams of packets over a single
 * high-speed-serial link on a Spartan-6 FPGA.
 */

module spio_hss_multiplexer #( // The width of each channel (here, one SpiNNaker packet)
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
                                 // Incoming data from NUM_CHANNELS streams each
                                 // CHANNEL_WIDTH wide. Since verilog does not
                                 // support array types for inputs or outputs,
                                 // the different channels are simply allocated
                                 // to ascending pins of the busses below.
                             ,   input  wire [(NUM_CHANNELS*CHANNEL_WIDTH)-1:0] TX_PKT_DATA_IN
                             ,   input  wire [NUM_CHANNELS-1:0]                 TX_PKT_VLD_IN
                             ,   output wire [NUM_CHANNELS-1:0]                 TX_PKT_RDY_OUT
                                 // Outgoing data from NUM_CHANNELS streams each
                                 // CHANNEL_WIDTH wide.
                             ,   output wire [(NUM_CHANNELS*CHANNEL_WIDTH)-1:0] RX_PKT_DATA_OUT
                             ,   output wire [NUM_CHANNELS-1:0]                 RX_PKT_VLD_OUT
                             ,   input  wire [NUM_CHANNELS-1:0]                 RX_PKT_RDY_IN
                             
                               // High-level protocol Performance counters
                                 // Accessible as a bank of read-only registers
                                 // whose addresses are given in
                                 // spio_hss_multiplexer_register_bank.h
                             ,   input  wire [`REGA_BITS - 1:0] REG_ADDR_IN
                             ,   output wire [`REGD_BITS - 1:0] REG_DATA_OUT
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
// High-level packet-framing protocol.
////////////////////////////////////////////////////////////////////////////////

// TODO: Check parameters selected above are compatible

spio_hss_multiplexer_spinnlink
spio_hss_multiplexer_spinnlink_i( .clk       (CLK_IN)
                                , .rst       (RESET_IN)
                                
                                  // Monitoring interface
                                , .reg_addr  (REG_ADDR_IN),
                                , .reg_data  (REG_DATA_OUT),
                                
                                  // To high-speed serial: assembled frames out
                                , .hsl_data  (txdata_i)
                                , .hsl_kchr  (txcharisk_i)
                                , .hsl_rdy   (txrdy_i)
                                
                                  // From high-speed serial: assembled frames in
                                , ihsl_data  (rxdata_i)
                                , ihsl_kchr  (rxcharisk_i)
                                , ihsl_vld   (rxvld_i)
                                
                                  // Incoming packet streams
                                , .pkt_data0 (TX_PKT_DATA_IN[(CHANNEL_WIDTH*1)-1+:CHANNEL_WIDTH])
                                , .pkt_vld0  (TX_PKT_VLD_IN[0])
                                , .pkt_rdy0  (TX_PKT_RDY_OUT[0])
                                
                                , .pkt_data1 (TX_PKT_DATA_IN[(CHANNEL_WIDTH*2)-1+:CHANNEL_WIDTH])
                                , .pkt_vld1  (TX_PKT_VLD_IN[1])
                                , .pkt_rdy1  (TX_PKT_RDY_OUT[1])
                                
                                , .pkt_data2 (TX_PKT_DATA_IN[(CHANNEL_WIDTH*3)-1+:CHANNEL_WIDTH])
                                , .pkt_vld2  (TX_PKT_VLD_IN[2])
                                , .pkt_rdy2  (TX_PKT_RDY_OUT[2])
                                
                                , .pkt_data3 (TX_PKT_DATA_IN[(CHANNEL_WIDTH*4)-1+:CHANNEL_WIDTH])
                                , .pkt_vld3  (TX_PKT_VLD_IN[3])
                                , .pkt_rdy3  (TX_PKT_RDY_OUT[3])
                                
                                , .pkt_data4 (TX_PKT_DATA_IN[(CHANNEL_WIDTH*5)-1+:CHANNEL_WIDTH])
                                , .pkt_vld4  (TX_PKT_VLD_IN[4])
                                , .pkt_rdy4  (TX_PKT_RDY_OUT[4])
                                
                                , .pkt_data5 (TX_PKT_DATA_IN[(CHANNEL_WIDTH*6)-1+:CHANNEL_WIDTH])
                                , .pkt_vld5  (TX_PKT_VLD_IN[5])
                                , .pkt_rdy5  (TX_PKT_RDY_OUT[5])
                                
                                , .pkt_data6 (TX_PKT_DATA_IN[(CHANNEL_WIDTH*7)-1+:CHANNEL_WIDTH])
                                , .pkt_vld6  (TX_PKT_VLD_IN[6])
                                , .pkt_rdy6  (TX_PKT_RDY_OUT[6])
                                
                                , .pkt_data7 (TX_PKT_DATA_IN[(CHANNEL_WIDTH*8)-1+:CHANNEL_WIDTH])
                                , .pkt_vld7  (TX_PKT_VLD_IN[7])
                                , .pkt_rdy7  (TX_PKT_RDY_OUT[7])
                                
                                  // Outgoing packet streams
                                , opkt_data0 (RX_PKT_DATA_OUT[(CHANNEL_WIDTH*1)-1+:CHANNEL_WIDTH])
                                , opkt_vld0  (RX_PKT_VLD_OUT[0])
                                , opkt_rdy0  (RX_PKT_RDY_IN[0])
                                
                                , opkt_data1 (RX_PKT_DATA_OUT[(CHANNEL_WIDTH*2)-1+:CHANNEL_WIDTH])
                                , opkt_vld1  (RX_PKT_VLD_OUT[1])
                                , opkt_rdy1  (RX_PKT_RDY_IN[1])
                                
                                , opkt_data2 (RX_PKT_DATA_OUT[(CHANNEL_WIDTH*3)-1+:CHANNEL_WIDTH])
                                , opkt_vld2  (RX_PKT_VLD_OUT[2])
                                , opkt_rdy2  (RX_PKT_RDY_IN[2])
                                
                                , opkt_data3 (RX_PKT_DATA_OUT[(CHANNEL_WIDTH*4)-1+:CHANNEL_WIDTH])
                                , opkt_vld3  (RX_PKT_VLD_OUT[3])
                                , opkt_rdy3  (RX_PKT_RDY_IN[3])
                                
                                , opkt_data4 (RX_PKT_DATA_OUT[(CHANNEL_WIDTH*5)-1+:CHANNEL_WIDTH])
                                , opkt_vld4  (RX_PKT_VLD_OUT[4])
                                , opkt_rdy4  (RX_PKT_RDY_IN[4])
                                
                                , opkt_data5 (RX_PKT_DATA_OUT[(CHANNEL_WIDTH*6)-1+:CHANNEL_WIDTH])
                                , opkt_vld5  (RX_PKT_VLD_OUT[5])
                                , opkt_rdy5  (RX_PKT_RDY_IN[5])
                                
                                , opkt_data6 (RX_PKT_DATA_OUT[(CHANNEL_WIDTH*7)-1+:CHANNEL_WIDTH])
                                , opkt_vld6  (RX_PKT_VLD_OUT[6])
                                , opkt_rdy6  (RX_PKT_RDY_IN[6])
                                
                                , opkt_data7 (RX_PKT_DATA_OUT[(CHANNEL_WIDTH*8)-1+:CHANNEL_WIDTH])
                                , opkt_vld7  (RX_PKT_VLD_OUT[7])
                                , opkt_rdy7  (RX_PKT_RDY_IN[7])
                                );

endmodule
