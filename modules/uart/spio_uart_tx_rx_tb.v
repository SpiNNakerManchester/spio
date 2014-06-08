/**
 * Simple testbench for the UART TX/RX modules: connect a TX to an RX.
 */

module spio_uart_tx_rx_tb;

localparam TX_CLK_FREQ  = 37500000; // Hz
localparam TX_BAUD_RATE =   115200; // Hz

localparam RX_CLK_FREQ  = 37500005; // Hz
localparam RX_BAUD_RATE =   115205; // Hz

genvar i;

reg tx_clk_i;
reg rx_clk_i;

reg tx_reset_i;
reg rx_reset_i;

// Byte input stream
reg  [7:0] in_data_i;
reg        in_vld_i;
wire       in_rdy_i;

// Byte output stream
wire [7:0] out_data_i;
wire       out_vld_i;


// Baudrate generator signals
wire tx_baud_pulse_i;
wire rx_subsample_pulse_i;

// Serial signal
wire tx_data_i;
wire tx_data_synced_i;


////////////////////////////////////////////////////////////////////////////////
// TX Devices under test
////////////////////////////////////////////////////////////////////////////////

// Generate baud timing for transmitter
spio_uart_baud_gen    #( .PERIOD   (TX_CLK_FREQ/TX_BAUD_RATE)
                       , .NUM_BITS ($clog2(TX_CLK_FREQ/TX_BAUD_RATE))
                       )
spio_uart_baud_gen_tx_i( .CLK_IN              (tx_clk_i)
                       , .RESET_IN            (tx_reset_i)
                       , .BAUD_PULSE_OUT      (tx_baud_pulse_i)
                       , .SUBSAMPLE_PULSE_OUT () // Unused
                       );

// UART Transmitter
spio_uart_tx spio_uart_tx_i ( .CLK_IN        (tx_clk_i)
                            , .RESET_IN      (tx_reset_i)
                            , .DATA_IN       (in_data_i)
                            , .VLD_IN        (in_vld_i)
                            , .RDY_OUT       (in_rdy_i)
                            , .BAUD_PULSE_IN (tx_baud_pulse_i)
                            , .TX_OUT        (tx_data_i)
                            );


////////////////////////////////////////////////////////////////////////////////
// RX Devices under test
////////////////////////////////////////////////////////////////////////////////

// Generate baud timing for receiver
spio_uart_baud_gen    #( .PERIOD   (RX_CLK_FREQ/RX_BAUD_RATE)
                       , .NUM_BITS ($clog2(RX_CLK_FREQ/RX_BAUD_RATE))
                       )
spio_uart_baud_gen_rx_i( .CLK_IN              (rx_clk_i)
                       , .RESET_IN            (rx_reset_i)
                       , .BAUD_PULSE_OUT      () // Unused
                       , .SUBSAMPLE_PULSE_OUT (rx_subsample_pulse_i)
                       );


// Synchroniser for the serial signal
spio_uart_sync         #( .NUM_BITS      (1)
                        , .NUM_STAGES    (2)
                        , .INITIAL_VALUE (1'b1)
                        )
spio_uart_sync_tx_data_i( .CLK_IN   (rx_clk_i)
                        , .RESET_IN (rx_reset_i)
                        , .DATA_IN  (tx_data_i)
                        , .DATA_OUT (tx_data_synced_i)
                        );

// UART Receiver
spio_uart_rx
spio_uart_rx_i( .CLK_IN             (rx_clk_i)
              , .RESET_IN           (rx_reset_i)
              , .RX_IN              (tx_data_synced_i)
              , .DATA_OUT           (out_data_i)
              , .VLD_OUT            (out_vld_i)
              , .SUBSAMPLE_PULSE_IN (rx_subsample_pulse_i)
              );

////////////////////////////////////////////////////////////////////////////////
// Input Stimulus
////////////////////////////////////////////////////////////////////////////////

localparam TX_WAIT_TIME = (1000000000/TX_CLK_FREQ)*(TX_CLK_FREQ/TX_BAUD_RATE)*100;
localparam RX_WAIT_TIME = (1000000000/RX_CLK_FREQ)*(RX_CLK_FREQ/RX_BAUD_RATE)*100;

`define  TXTICK                 @(posedge tx_clk_i)
`define MTXTICK #(TX_WAIT_TIME) @(posedge tx_clk_i)

`define  RXTICK                 @(posedge rx_clk_i)
`define MRXTICK #(RX_WAIT_TIME) @(posedge rx_clk_i)

initial
	begin
	// Check nothing arrives while both input invalid and output not ready
	in_vld_i <= 1'b0; @(negedge rx_reset_i)
	
	`MRXTICK;
	// Check nothing arrives while output ready
	in_vld_i <= 1'b0;
	
	`MTXTICK;
	// Check single bytes can be sent
	in_vld_i <= 1'b1;
	`TXTICK;
	in_vld_i <= 1'b0;
	
	`MTXTICK;
	// Check bytes flow freely and can be stopped
	in_vld_i <= 1'b1;
	`MTXTICK;
	in_vld_i <= 1'b0;
	
	`MRXTICK
	// Job done! Check everything sent also arrived.
	if (in_data_i != next_arrival_i)
		$display( "Time %d: ERROR: Some values never arrived: last ID sent = %x, last ID arrived = %x."
		        , $time
		        , in_data_i-1
		        , next_arrival_i-1
		        );
	#0
	$finish;
	end


// Generate bytes with ascending values
always @ (posedge tx_clk_i, posedge tx_reset_i)
	if (tx_reset_i)
		in_data_i <= 8'h55;
	else
		if (in_vld_i && in_rdy_i)
			in_data_i <= in_data_i + 1;


////////////////////////////////////////////////////////////////////////////////
// Behaviour validation
////////////////////////////////////////////////////////////////////////////////

// Check incoming bytes have the right value
reg [7:0] next_arrival_i;
always @ (posedge rx_clk_i, posedge rx_reset_i)
	if (rx_reset_i)
		next_arrival_i <= 8'h55;
	else
		if (out_vld_i)
			begin
			if (out_data_i != next_arrival_i)
				$display( "Time %d: ERROR: Value out of sequence: got %x, expected %x."
				        , $time
				        , out_data_i
				        , next_arrival_i
				        );
				next_arrival_i <= out_data_i + 1;
			end


////////////////////////////////////////////////////////////////////////////////
// Testbench signals
////////////////////////////////////////////////////////////////////////////////

// Clock generation
initial
	begin
	tx_clk_i = 1'b0;
	forever
		#(1000000000.0/TX_CLK_FREQ) tx_clk_i = ~tx_clk_i;
	end

initial
	begin
	rx_clk_i = 1'b0;
	forever
		#(1000000000.0/RX_CLK_FREQ) rx_clk_i = ~rx_clk_i;
	end


// Reset generation
initial
	begin
		tx_reset_i <= 1'b1;
		rx_reset_i <= 1'b1;
		#100
		tx_reset_i <= 1'b0;
		#30
		rx_reset_i <= 1'b0;
	end


endmodule


