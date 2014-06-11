/**
 * A transmitter/receiver module which sends and receives SpiNNaker packets over
 * a UART link.
 */

`include "spio_uart_common.h"

module spio_uart#( // 1 if device is master, 0 if slave.
                   parameter IS_MASTER = 0
                   // The number of CLK_IN ticks per baud clock tick
                 , parameter BAUD_PERIOD = 100
                   // The number of bits to use for the baud clock timer
                   // (must be enough to represent BAUD_PERIOD-1)
                 , parameter BAUD_NUM_BITS = 7
                   // The number of bits required to address a buffer of packets
                   // received (i.e. the packet receive buffer will have size
                   // (1<<BUFFER_ADDR_BITS)-1). If set to zero, no buffer will
                   // be used.
                 , parameter RX_BUFFER_ADDR_BITS = 4
                   // The high water mark for RX packet buffer occupancy beyond
                   // which the clear-to-send signal is deasserted.
                 , parameter RX_HIGH_WATER_MARK = 8
                 )
                 ( // Common clock for synchronous signals
                   input wire CLK_IN
                   // Asynchronous active-high reset
                 , input wire RESET_IN
                   // "Asynchronous" signals (incoming signals must be
                   // appropriately synchronised externally)
                     // Incoming serial stream
                 ,   input  wire RX_IN
                     // Clear to send signal from the remote device. When low,
                     // only synchronisation sequences will be sent.
                 ,   output wire CTS_IN
                     // Outgoing serial stream
                 ,   output wire TX_OUT
                     // Clear to send signal (asserted when the internal packet
                     // FIFO passes the high water mark).
                 ,   output wire CTS_OUT
                   // Synchronous receive-side signals
                     // SpiNNaker packet stream arriving from the remote device
                     // using the usual rdy/vld protocol. If the
                     // BUFFER_ADDR_BITS has been set to 0 (i.e. no packet
                     // buffering is used), the RX_RDY_IN signal is ignored and
                     // RX_DATA_OUT and RX_VLD_OUT will appear for one cycle
                     // whenever a byte is received.
                 ,   output wire [`PKT_LEN-1:0] RX_DATA_OUT
                 ,   output wire                RX_VLD_OUT
                 ,   input  wire                RX_RDY_IN
                     // This signal pulses for one clock cycle whenever a byte
                     // is dropped due to the internal packet FIFO being full.
                     // This signal is always 0 when BUFFER_ADDR_BITS is 0.
                 ,   output wire RX_PACKET_DROPPED_OUT
                   // Synchronous transmit-side signals
                     // SpiNNaker packet stream to send to the remote device
                     // using the usual rdy/vld protocol.
                 ,   input  wire [`PKT_LEN-1:0] TX_DATA_IN
                 ,   input  wire                TX_VLD_IN
                 ,   output wire                TX_RDY_OUT
                     // This signal pulses for one clock cycle whenever the last packet
                     // arriving on TX_DATA_IN had a parity error and was
                     // dropped (i.e. won't be sent).
                 ,   output wire TX_PACKET_DROPPED_OUT
                   // Synchronisation signals
                     // A one-cycle pulse on this line will cause a
                     // synchronisation sequence to be sent after any
                     // in-transmission packets are sent. This signal will
                     // typically be used by a master to initiate
                     // synchronisation. Slaves will typically tie this low.
                 ,   input wire SYNC_TRIGGER_IN
                     // This signal is high whenever synchronisation is taking
                     // place. For slaves, this signal will be high after reset
                     // and remain high until it has received a synchronisation
                     // sequence from a master. Note that for masters, this
                     // signal may tempoarily falsely report synchronisation
                     // completion too early due to previous data arriving.
                 ,   output wire SYNCHRONISING_OUT
                 );


// Baud clock signals
wire baud_pulse_i;
wire subsample_pulse_i;

// TX control packet interface readiness/validity (which is conditional on CTS)
wire tx_control_pkt_vld_i;
wire tx_control_pkt_rdy_i;

// Cause the TX to send a sync sequence
wire tx_sync_trigger_i;

// TX synchronisation state
wire tx_synchronising_i;

// TX byte stream
wire [7:0] tx_byte_data_i;
wire       tx_byte_vld_i;
wire       tx_byte_rdy_i;

// RX byte stream
wire [7:0] rx_byte_data_i;
wire       rx_byte_vld_i;

// Unbuffered RX packet stream
wire [`PKT_LEN-1:0] rx_unbuf_pkt_data_i;
wire                rx_unbuf_pkt_vld_i;

// RX synchronisation status
wire rx_synchronising_i;

// RX buffer state
wire [RX_BUFFER_ADDR_BITS-1:0] rx_buffer_occupancy;
wire                           rx_buffer_not_full;

// Signal to explicitly block further packets from being transmitted
wire tx_block_i;

////////////////////////////////////////////////////////////////////////////////
// Baud clock generation
////////////////////////////////////////////////////////////////////////////////

spio_uart_baud_gen #( .PERIOD              (BAUD_PERIOD)
                    , .NUM_BITS            (BAUD_NUM_BITS)
                    )
spio_uart_baud_gen_i( .CLK_IN              (CLK_IN)
                    , .RESET_IN            (RESET_IN)
                    , .BAUD_PULSE_OUT      (baud_pulse_i)
                    , .SUBSAMPLE_PULSE_OUT (subsample_pulse_i)
                    );

////////////////////////////////////////////////////////////////////////////////
// TX Side
////////////////////////////////////////////////////////////////////////////////

// UART byte transmitter
spio_uart_tx
spio_uart_tx_i( .CLK_IN        (CLK_IN)
              , .RESET_IN      (RESET_IN)
                // Byte stream to transmit
              , .DATA_IN       (tx_byte_data_i)
              , .VLD_IN        (tx_byte_vld_i)
              , .RDY_OUT       (tx_byte_rdy_i)
                // Baud clock
              , .BAUD_PULSE_IN (baud_pulse_i)
                // "Asynchronous" serial stream
              , .TX_OUT        (TX_OUT)
              );

// Synchronisation sequence generation and packet transmission
spio_uart_tx_control
spio_uart_tx_control_i ( .CLK_IN            (CLK_IN)
                       , .RESET_IN          (RESET_IN)
                       , .SYNC_TRIGGER_IN   (tx_sync_trigger_i)
                       , .SYNCHRONISING_OUT (tx_synchronising_i)
                         // Incoming packet stream
                       , .PKT_DATA_IN       (TX_DATA_IN)
                       , .PKT_VLD_IN        (tx_control_pkt_vld_i)
                       , .PKT_RDY_OUT       (tx_control_pkt_rdy_i)
                       , .PKT_DROPPED_OUT   (TX_PACKET_DROPPED_OUT)
                         // Outgoing byte stream
                       , .BYTE_DATA_OUT     (tx_byte_data_i)
                       , .BYTE_VLD_OUT      (tx_byte_vld_i)
                       , .BYTE_RDY_IN       (tx_byte_rdy_i)
                       );

// Prevent packets being consumed while blocked
assign tx_control_pkt_vld_i = TX_VLD_IN            && !tx_block_i;
assign TX_RDY_OUT           = tx_control_pkt_rdy_i && !tx_block_i;

////////////////////////////////////////////////////////////////////////////////
// RX Side
////////////////////////////////////////////////////////////////////////////////

// UART byte receiver
spio_uart_rx
spio_uart_rx_i( .CLK_IN             (CLK_IN)
              , .RESET_IN           (RESET_IN)
                // Incoming serial stream
              , .RX_IN              (RX_IN)
                // Incoming bytes
              , .DATA_OUT           (rx_byte_data_i)
              , .VLD_OUT            (rx_byte_vld_i)
                // Baud clock
              , .SUBSAMPLE_PULSE_IN (subsample_pulse_i)
              );


// Synchronisation and packet reception
spio_uart_rx_control
spio_uart_rx_control_i( .CLK_IN            (CLK_IN)
                      , .RESET_IN          (RESET_IN)
                      , .SYNCHRONISING_OUT (rx_synchronising_i)
                        // Incoming byte stream.
                      , .BYTE_DATA_IN      (rx_byte_data_i)
                      , .BYTE_VLD_IN       (rx_byte_vld_i)
                        // Outgoing (unbuffered) packet stream.
                      , .PKT_DATA_OUT      (rx_unbuf_pkt_data_i)
                      , .PKT_VLD_OUT       (rx_unbuf_pkt_vld_i)
                      );


// FIFO to hold incoming packets
spio_uart_fifo #( .BUFFER_ADDR_BITS (RX_BUFFER_ADDR_BITS)
                , .WORD_SIZE        (`PKT_LEN)
                )
spio_uart_fifo_i( .CLK_IN           (CLK_IN)
                , .RESET_IN         (RESET_IN)
                  // FIFO occupancy
                , .OCCUPANCY_OUT    (rx_buffer_occupancy)
                  // Incoming unbuffered packet stream
                , .IN_DATA_IN       (rx_unbuf_pkt_data_i)
                , .IN_VLD_IN        (rx_unbuf_pkt_vld_i)
                , .IN_RDY_OUT       (rx_buffer_not_full)
                  // Buffered packet stream
                , .OUT_DATA_OUT      (RX_DATA_OUT)
                , .OUT_VLD_OUT       (RX_VLD_OUT)
                , .OUT_RDY_IN        (RX_RDY_IN)
                );

// Remove CTS when FIFO getting full
assign CTS_OUT = rx_buffer_occupancy < RX_HIGH_WATER_MARK;

// Drop a packet whenever one arrives while the FIFO is full
assign RX_PACKET_DROPPED_OUT = rx_unbuf_pkt_vld_i && !rx_buffer_not_full;


////////////////////////////////////////////////////////////////////////////////
// Master/slave specific behaviour
////////////////////////////////////////////////////////////////////////////////

generate if (IS_MASTER)
	begin : master_behaviour
		// Start a synchronisation sequence on demand
		assign tx_sync_trigger_i = SYNC_TRIGGER_IN;
		
		// Only block when CTS isn't high
		// XXX: Will deassert ready when not transferring!
		assign tx_block_i = !CTS_IN;
		
		assign SYNCHRONISING_OUT = rx_synchronising_i || tx_synchronising_i;
	end
else
	begin : slave_behaviour
		// Value of rx_synchronising_i last cycle
		reg rx_last_synchronising_i;
		always @ (posedge CLK_IN, posedge RESET_IN)
			if (RESET_IN)
				rx_last_synchronising_i <= 1'b1;
			else
				rx_last_synchronising_i <= rx_synchronising_i;
		
		// A signal which is high for one cycle when the RX becomes synchronised.
		wire rx_just_synced_i =  (rx_last_synchronising_i == 1'b1)
		                      && (rx_synchronising_i      == 1'b0);
		
		// Slaves must respond to recipt of a sync sequence (or manual intervention)
		// with their own sync sequence.
		assign tx_sync_trigger_i = SYNC_TRIGGER_IN || rx_just_synced_i;
		
		// Slaves must not transmit until synchronised (or when CTS isn't high).
		// XXX: Will deassert ready when not transferring!
		assign tx_block_i = rx_synchronising_i || tx_sync_trigger_i || !CTS_IN;
		
		assign SYNCHRONISING_OUT =  rx_synchronising_i || tx_sync_trigger_i
		                         || tx_synchronising_i;
	end
endgenerate


endmodule
