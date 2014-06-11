/**
 * Simple testbench for the top-level UART module: connect a master and a slave
 * to eachother.
 *
 * Not tested: Filtering of packets with incorrect parity at transmitter.
 */

`include "spio_uart_common.h"

module spio_uart_tb();

localparam CLK_FREQ  = 37_500_000;
localparam BAUD_RATE =    115_200;
localparam RX_BUFFER_ADDR_BITS = 2;

genvar i;

// Master and slave, resp.
reg clk_i   [1:0];
reg reset_i [1:0];

// Packet input stream (master and slave, resp.)
wire [`PKT_LEN-1:0] tx_data_i [1:0];
reg                 tx_vld_i  [1:0];
wire                tx_rdy_i  [1:0];

// Packet output stream (master and slave, resp.)
wire [`PKT_LEN-1:0] rx_data_expected_i [1:0];
wire [`PKT_LEN-1:0] rx_data_i [1:0];
wire                rx_vld_i  [1:0];
reg                 rx_rdy_i  [1:0];


// Serial signals (master and slave, resp.)
wire uart_data_i        [1:0];
wire uart_data_synced_i [1:0];


// CTS signals (master and slave, resp.)
wire cts_i        [1:0];
wire cts_synced_i [1:0];

// Sync trigger port (master and slave, resp.)
reg sync_trigger_i [1:0];

// Synchronisation status (master and slave, resp.)
wire synchronising_i [1:0];

// Packet drop indicators (master and slave, resp.)
wire tx_packet_dropped_i [1:0];
wire rx_packet_dropped_i [1:0];

////////////////////////////////////////////////////////////////////////////////
// Devices and Synchroniser instanciation
////////////////////////////////////////////////////////////////////////////////

generate for (i = 0; i < 2; i = i + 1)
	begin : instances
		// Synchroniser for the serial signal
		spio_uart_sync         #( .NUM_BITS      (1)
		                        , .NUM_STAGES    (2)
		                        , .INITIAL_VALUE (1'b1)
		                        )
		spio_uart_sync_tx_data_i( .CLK_IN   (clk_i[i])
		                        , .RESET_IN (reset_i[i])
		                        , .DATA_IN  (uart_data_i[i])
		                        , .DATA_OUT (uart_data_synced_i[i])
		                        );
		
		// Synchroniser for the cts signal
		spio_uart_sync     #( .NUM_BITS      (1)
		                    , .NUM_STAGES    (2)
		                    , .INITIAL_VALUE (1'b0)
		                    )
		spio_uart_sync_cts_i( .CLK_IN   (clk_i[i])
		                    , .RESET_IN (reset_i[i])
		                    , .DATA_IN  (cts_i[i])
		                    , .DATA_OUT (cts_synced_i[i])
		                    );
		
		// UART module itself
		spio_uart #( .IS_MASTER             (i==0)
		           , .BAUD_PERIOD           (CLK_FREQ/BAUD_RATE)
		           , .BAUD_NUM_BITS         ($clog2(CLK_FREQ/BAUD_RATE))
		           , .RX_BUFFER_ADDR_BITS   (RX_BUFFER_ADDR_BITS)
		           , .RX_HIGH_WATER_MARK    ((1<<RX_BUFFER_ADDR_BITS)/2)
		           )
		spio_uart_i( .CLK_IN                (clk_i[i])
		           , .RESET_IN              (reset_i[i])
		             // UART signals
		           , .RX_IN                 (uart_data_synced_i[(i+1)%2])
		           , .CTS_IN                (cts_synced_i[(i+1)%2])
		           , .TX_OUT                (uart_data_i[i])
		           , .CTS_OUT               (cts_i[i])
		             // RX Packets
		           , .RX_DATA_OUT           (rx_data_i[i])
		           , .RX_VLD_OUT            (rx_vld_i[i])
		           , .RX_RDY_IN             (rx_rdy_i[i])
		           , .RX_PACKET_DROPPED_OUT (rx_packet_dropped_i[i])
		             // TX Packets
		           , .TX_DATA_IN            (tx_data_i[i])
		           , .TX_VLD_IN             (tx_vld_i[i])
		           , .TX_RDY_OUT            (tx_rdy_i[i])
		           , .TX_PACKET_DROPPED_OUT (tx_packet_dropped_i[i])
		             // Synchronisation signals
		           , .SYNC_TRIGGER_IN       (sync_trigger_i[i])
		           , .SYNCHRONISING_OUT     (synchronising_i[i])
		           );
	end
endgenerate

////////////////////////////////////////////////////////////////////////////////
// Input Stimulus
////////////////////////////////////////////////////////////////////////////////

localparam WAIT_TIME = (1000000000/CLK_FREQ)*(CLK_FREQ/BAUD_RATE)*1000;

`define  MTICK              @(posedge clk_i[0])
`define MMTICK #(WAIT_TIME) @(posedge clk_i[0])

`define  STICK              @(posedge clk_i[1])
`define MSTICK #(WAIT_TIME) @(posedge clk_i[1])

`define MTXV tx_vld_i[0]
`define STXV tx_vld_i[1]

`define MRXR rx_rdy_i[0]
`define SRXR rx_rdy_i[1]

`define MST sync_trigger_i[0]
`define SST sync_trigger_i[1]

integer sim_phase;

initial
	begin
	sim_phase = 0;
	
	// Initially nothing should happen
	`MTXV<=1'b0; `MRXR<=1'b0; `STXV<=1'b0; `SRXR<=1'b0; `MST<=1'b0; `SST<=1'b0;
	@(negedge reset_i[1]);
	`MMTICK;
	
	sim_phase = sim_phase + 1;
	$display("Trigger synchronisation via the master");
	`MMTICK;
	`MTXV<=1'b0; `MRXR<=1'b0; `STXV<=1'b0; `SRXR<=1'b0; `MST<=1'b1; `SST<=1'b0;
	`MTICK;
	`MTXV<=1'b0; `MRXR<=1'b0; `STXV<=1'b0; `SRXR<=1'b0; `MST<=1'b0; `SST<=1'b0;
	
	sim_phase = sim_phase + 1;
	$display("Should now be in sync");
	`MMTICK;
	if (synchronising_i[0])
		$display("Time %d: ERROR: Master should be synchronised.", $time);
	if (synchronising_i[1])
		$display("Time %d: ERROR: Slave should be synchronised.", $time);
	
	sim_phase = sim_phase + 1;
	$display("Should be able to send to eachother without backpressure");
		`MMTICK;
		// Send
		`MTXV<=1'b1; `MRXR<=1'b1; `STXV<=1'b0; `SRXR<=1'b0; `MST<=1'b0; `SST<=1'b0;
		`STICK;
		`MTXV<=1'b1; `MRXR<=1'b1; `STXV<=1'b1; `SRXR<=1'b1; `MST<=1'b0; `SST<=1'b0;
		
		`MMTICK;
		// Stop sending
		`MTXV<=1'b0; `MRXR<=1'b1; `STXV<=1'b0; `SRXR<=1'b1; `MST<=1'b0; `SST<=1'b0;
		`STICK;
		`MTXV<=1'b0; `MRXR<=1'b1; `STXV<=1'b0; `SRXR<=1'b1; `MST<=1'b0; `SST<=1'b0;
		
		`MMTICK;
		// Drain
		`MTXV<=1'b0; `MRXR<=1'b0; `STXV<=1'b0; `SRXR<=1'b1; `MST<=1'b0; `SST<=1'b0;
		`STICK;
		`MTXV<=1'b0; `MRXR<=1'b0; `STXV<=1'b0; `SRXR<=1'b0; `MST<=1'b0; `SST<=1'b0;
	
	sim_phase = sim_phase + 1;
	$display("Should be able to block, unblock and drain");
		`MMTICK;
		// Send
		`MTXV<=1'b1; `MRXR<=1'b1; `STXV<=1'b0; `SRXR<=1'b0; `MST<=1'b0; `SST<=1'b0;
		`STICK;
		`MTXV<=1'b1; `MRXR<=1'b1; `STXV<=1'b1; `SRXR<=1'b1; `MST<=1'b0; `SST<=1'b0;
		
		`MMTICK;
		// Block
		`MTXV<=1'b1; `MRXR<=1'b0; `STXV<=1'b1; `SRXR<=1'b1; `MST<=1'b0; `SST<=1'b0;
		`STICK;
		`MTXV<=1'b1; `MRXR<=1'b0; `STXV<=1'b1; `SRXR<=1'b0; `MST<=1'b0; `SST<=1'b0;
		
		`MMTICK;
		// Unblock
		`MTXV<=1'b1; `MRXR<=1'b1; `STXV<=1'b1; `SRXR<=1'b0; `MST<=1'b0; `SST<=1'b0;
		`STICK;
		`MTXV<=1'b1; `MRXR<=1'b1; `STXV<=1'b1; `SRXR<=1'b1; `MST<=1'b0; `SST<=1'b0;
		
		`MMTICK;
		// Stop sending
		`MTXV<=1'b0; `MRXR<=1'b1; `STXV<=1'b0; `SRXR<=1'b1; `MST<=1'b0; `SST<=1'b0;
		`STICK;
		`MTXV<=1'b0; `MRXR<=1'b1; `STXV<=1'b0; `SRXR<=1'b1; `MST<=1'b0; `SST<=1'b0;
		
		`MMTICK;
		// Drain
		`MTXV<=1'b0; `MRXR<=1'b0; `STXV<=1'b0; `SRXR<=1'b1; `MST<=1'b0; `SST<=1'b0;
		`STICK;
		`MTXV<=1'b0; `MRXR<=1'b0; `STXV<=1'b0; `SRXR<=1'b0; `MST<=1'b0; `SST<=1'b0;
	
	sim_phase = sim_phase + 1;
	$display("Should be able to block and drain immediately");
		`MMTICK;
		// Send
		`MTXV<=1'b1; `MRXR<=1'b1; `STXV<=1'b0; `SRXR<=1'b0; `MST<=1'b0; `SST<=1'b0;
		`STICK;
		`MTXV<=1'b1; `MRXR<=1'b1; `STXV<=1'b1; `SRXR<=1'b1; `MST<=1'b0; `SST<=1'b0;
		
		`MMTICK;
		// Block
		`MTXV<=1'b1; `MRXR<=1'b0; `STXV<=1'b1; `SRXR<=1'b1; `MST<=1'b0; `SST<=1'b0;
		`STICK;
		`MTXV<=1'b1; `MRXR<=1'b0; `STXV<=1'b1; `SRXR<=1'b0; `MST<=1'b0; `SST<=1'b0;
		
		`MMTICK;
		// Stop sending
		`MTXV<=1'b0; `MRXR<=1'b0; `STXV<=1'b1; `SRXR<=1'b0; `MST<=1'b0; `SST<=1'b0;
		`STICK;
		`MTXV<=1'b0; `MRXR<=1'b0; `STXV<=1'b0; `SRXR<=1'b0; `MST<=1'b0; `SST<=1'b0;
		
		`MMTICK;
		// Drain
		`MTXV<=1'b0; `MRXR<=1'b1; `STXV<=1'b0; `SRXR<=1'b0; `MST<=1'b0; `SST<=1'b0;
		`STICK;
		`MTXV<=1'b0; `MRXR<=1'b1; `STXV<=1'b0; `SRXR<=1'b1; `MST<=1'b0; `SST<=1'b0;
	
	sim_phase = sim_phase + 1;
	$display("Should be able to resync both ends while idle");
	`MMTICK;
	`MTXV<=1'b0; `MRXR<=1'b0; `STXV<=1'b0; `SRXR<=1'b0; `MST<=1'b1; `SST<=1'b0;
	`MTICK;
	`MTXV<=1'b0; `MRXR<=1'b0; `STXV<=1'b0; `SRXR<=1'b0; `MST<=1'b0; `SST<=1'b0;
	`MSTICK;
	`MTXV<=1'b0; `MRXR<=1'b0; `STXV<=1'b0; `SRXR<=1'b0; `MST<=1'b0; `SST<=1'b1;
	`STICK;
	`MTXV<=1'b0; `MRXR<=1'b0; `STXV<=1'b0; `SRXR<=1'b0; `MST<=1'b0; `SST<=1'b0;
	
	sim_phase = sim_phase + 1;
	$display("Should be able to resync both ends while transmitting");
		`MMTICK;
		// Send
		`MTXV<=1'b1; `MRXR<=1'b1; `STXV<=1'b0; `SRXR<=1'b0; `MST<=1'b0; `SST<=1'b0;
		`STICK;
		`MTXV<=1'b1; `MRXR<=1'b1; `STXV<=1'b1; `SRXR<=1'b1; `MST<=1'b0; `SST<=1'b0;
		
		`MMTICK;
		// Trigger resyncs
		`MTXV<=1'b1; `MRXR<=1'b1; `STXV<=1'b1; `SRXR<=1'b1; `MST<=1'b1; `SST<=1'b0;
		`MTICK;
		`MTXV<=1'b1; `MRXR<=1'b1; `STXV<=1'b1; `SRXR<=1'b1; `MST<=1'b0; `SST<=1'b0;
		`MSTICK;
		`MTXV<=1'b1; `MRXR<=1'b1; `STXV<=1'b1; `SRXR<=1'b1; `MST<=1'b0; `SST<=1'b1;
		`STICK;
		`MTXV<=1'b1; `MRXR<=1'b1; `STXV<=1'b1; `SRXR<=1'b1; `MST<=1'b0; `SST<=1'b0;
		
		`MMTICK;
		// Stop sending
		`MTXV<=1'b0; `MRXR<=1'b1; `STXV<=1'b0; `SRXR<=1'b1; `MST<=1'b0; `SST<=1'b0;
		`STICK;
		`MTXV<=1'b0; `MRXR<=1'b1; `STXV<=1'b0; `SRXR<=1'b1; `MST<=1'b0; `SST<=1'b0;
		
		`MMTICK;
		// Drain
		`MTXV<=1'b0; `MRXR<=1'b0; `STXV<=1'b0; `SRXR<=1'b1; `MST<=1'b0; `SST<=1'b0;
		`STICK;
		`MTXV<=1'b0; `MRXR<=1'b0; `STXV<=1'b0; `SRXR<=1'b0; `MST<=1'b0; `SST<=1'b0;
	
	sim_phase = sim_phase + 1;
	$display("Should be able to resync while blocked");
		`MMTICK;
		// Send
		`MTXV<=1'b1; `MRXR<=1'b1; `STXV<=1'b0; `SRXR<=1'b0; `MST<=1'b0; `SST<=1'b0;
		`STICK;
		`MTXV<=1'b1; `MRXR<=1'b1; `STXV<=1'b1; `SRXR<=1'b1; `MST<=1'b0; `SST<=1'b0;
		
		`MMTICK;
		// Block
		`MTXV<=1'b1; `MRXR<=1'b0; `STXV<=1'b1; `SRXR<=1'b1; `MST<=1'b0; `SST<=1'b0;
		`STICK;
		`MTXV<=1'b1; `MRXR<=1'b0; `STXV<=1'b1; `SRXR<=1'b0; `MST<=1'b0; `SST<=1'b0;
		
		`MMTICK;
		// Trigger resyncs
		`MTXV<=1'b1; `MRXR<=1'b0; `STXV<=1'b1; `SRXR<=1'b0; `MST<=1'b1; `SST<=1'b0;
		`MTICK;
		`MTXV<=1'b1; `MRXR<=1'b0; `STXV<=1'b1; `SRXR<=1'b0; `MST<=1'b0; `SST<=1'b0;
		`MSTICK;
		`MTXV<=1'b1; `MRXR<=1'b0; `STXV<=1'b1; `SRXR<=1'b0; `MST<=1'b0; `SST<=1'b1;
		`STICK;
		`MTXV<=1'b1; `MRXR<=1'b0; `STXV<=1'b1; `SRXR<=1'b0; `MST<=1'b0; `SST<=1'b0;
		
		`MMTICK;
		// Unblock
		`MTXV<=1'b1; `MRXR<=1'b1; `STXV<=1'b1; `SRXR<=1'b0; `MST<=1'b0; `SST<=1'b0;
		`STICK;
		`MTXV<=1'b1; `MRXR<=1'b1; `STXV<=1'b1; `SRXR<=1'b1; `MST<=1'b0; `SST<=1'b0;
		
		`MMTICK;
		// Stop sending
		`MTXV<=1'b0; `MRXR<=1'b1; `STXV<=1'b0; `SRXR<=1'b1; `MST<=1'b0; `SST<=1'b0;
		`STICK;
		`MTXV<=1'b0; `MRXR<=1'b1; `STXV<=1'b0; `SRXR<=1'b1; `MST<=1'b0; `SST<=1'b0;
		
		`MMTICK;
		// Drain
		`MTXV<=1'b0; `MRXR<=1'b0; `STXV<=1'b0; `SRXR<=1'b1; `MST<=1'b0; `SST<=1'b0;
		`STICK;
		`MTXV<=1'b0; `MRXR<=1'b0; `STXV<=1'b0; `SRXR<=1'b0; `MST<=1'b0; `SST<=1'b0;
	
	
	`MSTICK;
	// Job done! Check everything sent also arrived.
	if (tx_counter_i[0] != rx_counter_i[1])
		$display( "Time %d: ERROR: Some values from the master never arrived: last ID sent = %x, last ID arrived = %x."
		        , $time
		        , tx_counter_i[0]-1
		        , rx_counter_i[1]-1
		        );
	if (tx_counter_i[1] != rx_counter_i[0])
		$display( "Time %d: ERROR: Some values from the slave never arrived: last ID sent = %x, last ID arrived = %x."
		        , $time
		        , tx_counter_i[1]-1
		        , rx_counter_i[0]-1
		        );
	#0
	$finish;
	end

// Value of next packet to transmit (master and slave, resp.)
reg [31:0] tx_counter_i [1:0];

// Generate multicast packets with alternating lengths and ascending values
generate for (i = 0; i < 2; i = i + 1)
	begin : packet_generation
		always @ (posedge clk_i[i], posedge reset_i[i])
			if (reset_i[i])
				tx_counter_i[i] <= 32'd0;
			else
				if (tx_vld_i[i] && tx_rdy_i[i])
					tx_counter_i[i] <= tx_counter_i[i] + 1;
		
		assign tx_data_i[i] = (tx_counter_i[i][0] == 1'b0)
		                    ? {32'bX          , tx_counter_i[i], 7'h00, !(^tx_counter_i[i])}
		                    : {tx_counter_i[i], tx_counter_i[i], 8'h02}
		                    ;
	end
endgenerate

////////////////////////////////////////////////////////////////////////////////
// Behaviour validation
////////////////////////////////////////////////////////////////////////////////

// Value of next packet to receive (master and slave, resp.)
reg [31:0] rx_counter_i [1:0];

// Generate multicast packets with alternating lengths and ascending values
generate for (i = 0; i < 2; i = i + 1)
	begin : packet_consumption
		assign rx_data_expected_i[i] = (rx_counter_i[i][0] == 1'b0)
		                             ? {32'bX          , rx_counter_i[i], 7'h00, ~^rx_counter_i[i]}
		                             : {rx_counter_i[i], rx_counter_i[i], 8'h02}
		                             ;
		
		always @ (posedge clk_i[i], posedge reset_i[i])
			if (reset_i[i])
				rx_counter_i[i] <= 32'b0;
			else
				if (rx_vld_i[i] && rx_rdy_i[i])
					begin
						if (rx_data_i[i] != rx_data_expected_i[i])
							$display( "Time %d: ERROR: Value out of sequence for %d: got %x, expected %x."
							        , $time
							        , i
							        , rx_data_i[i]
							        , rx_data_expected_i[i]
							        );
						rx_counter_i[i] <= rx_data_i[i][8+:32] + 1;
					end
	end
endgenerate


////////////////////////////////////////////////////////////////////////////////
// Testbench signals
////////////////////////////////////////////////////////////////////////////////

generate for (i = 0; i < 2; i = i + 1)
	begin : testbench_signals
		// Clock
		initial
			begin
				clk_i[i] = 1'b0;
				forever
					#((1_000_000_000/CLK_FREQ)/2) clk_i[i] = ~clk_i[i];
			end
		
		// Reset
		initial
			begin
				reset_i[i] = 1'b1;
				#(100 + i*05) reset_i[i] = 1'b0;
			end
	end
endgenerate


endmodule


