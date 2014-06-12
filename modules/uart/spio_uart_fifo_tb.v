/**
 * Simple testbench for the UART FIFO module.
 */

module spio_uart_fifo_tb;

localparam BUFFER_ADDR_BITS = 4;
localparam BUFFER_SIZE      = 1<<BUFFER_ADDR_BITS;
localparam WORD_SIZE        = 8;

genvar i;

reg clk_i;
reg reset_i;

// Input stream
reg [WORD_SIZE-1:0] in_data_i;
reg                 in_vld_i;
wire                in_rdy_i;

// Output stream
wire [WORD_SIZE-1:0] out_data_i;
wire                 out_vld_i;
reg                  out_rdy_i;

wire [BUFFER_ADDR_BITS-1:0] fifo_occupancy_i;


////////////////////////////////////////////////////////////////////////////////
// Device under test
////////////////////////////////////////////////////////////////////////////////

spio_uart_fifo #( .BUFFER_ADDR_BITS(BUFFER_ADDR_BITS)
                , .WORD_SIZE(WORD_SIZE)
                )
spio_uart_fifo_i( .CLK_IN       (clk_i)
                , .RESET_IN     (reset_i)
                , .OCCUPANCY_OUT(fifo_occupancy_i)
                , .IN_DATA_IN   (in_data_i)
                , .IN_VLD_IN    (in_vld_i)
                , .IN_RDY_OUT   (in_rdy_i)
                , .OUT_DATA_OUT (out_data_i)
                , .OUT_VLD_OUT  (out_vld_i)
                , .OUT_RDY_IN   (out_rdy_i)
                );


////////////////////////////////////////////////////////////////////////////////
// Input Stimulus
////////////////////////////////////////////////////////////////////////////////

`define  TICK      @(posedge clk_i)
`define MTICK #500 @(posedge clk_i)

initial
	begin
	// Check nothing is sent while input is invalid
	in_vld_i <= 1'b0;   out_rdy_i <= 1'b1; @(negedge reset_i) `MTICK;
	
	// Check a single packet goes in then out
	in_vld_i <= 1'b1;   out_rdy_i <= 1'b1; `TICK;
	in_vld_i <= 1'b0;   out_rdy_i <= 1'b1; `MTICK;
	
	// Check many packets go in
	in_vld_i <= 1'b1;   out_rdy_i <= 1'b1; `MTICK;
	in_vld_i <= 1'b0;   out_rdy_i <= 1'b1; `MTICK;
	
	// Check many packets go in and the FIFO fills up when blocked and resumes
	// when unblocked
	in_vld_i <= 1'b1;   out_rdy_i <= 1'b1; `MTICK;
	in_vld_i <= 1'b1;   out_rdy_i <= 1'b0; `MTICK;
	in_vld_i <= 1'b1;   out_rdy_i <= 1'b1; `MTICK;
	in_vld_i <= 1'b0;   out_rdy_i <= 1'b1; `MTICK;
	
	// Check many packets go in and the FIFO fills up when blocked and drains
	// when unblocked with idle inputs
	in_vld_i <= 1'b1;   out_rdy_i <= 1'b1; `MTICK;
	in_vld_i <= 1'b1;   out_rdy_i <= 1'b0; `MTICK;
	in_vld_i <= 1'b0;   out_rdy_i <= 1'b1; `MTICK;
	
	
	// Job done! Check everything sent also arrived.
	if (in_data_i != next_arrival_i)
		$display( "Time %d: ERROR: Some values never arrived: last ID sent = %x, last ID arrived = %x."
		        , $time
		        , in_data_i-1
		        , out_data_i-1
		        );
	#0
	$finish;
	end


// Generate words with ascending values
always @ (posedge clk_i, posedge reset_i)
	if (reset_i)
		in_data_i <= {WORD_SIZE{1'b0}};
	else
		if (in_vld_i && in_rdy_i)
			in_data_i <= in_data_i + 1;


// Check incoming packets have the right ID
reg [WORD_SIZE-1:0] next_arrival_i;
always @ (posedge clk_i, posedge reset_i)
	if (reset_i)
		next_arrival_i <= {WORD_SIZE{1'b0}};
	else
		if (out_vld_i && out_rdy_i)
			begin
			if (out_data_i != next_arrival_i)
				$display( "Time %d: ERROR: Value out of sequence: got %x, expected %x."
				        , $time
				        , out_data_i
				        , next_arrival_i
				        );
				next_arrival_i <= out_data_i + 1;
			end


// Check occupancy is correct
wire [BUFFER_ADDR_BITS-1:0] expected_occupancy_i = in_data_i - next_arrival_i;
always @ (posedge clk_i, posedge reset_i)
	if (!reset_i)
		if (fifo_occupancy_i != expected_occupancy_i)
			$display( "Time %d: ERROR: Occupancy reported as %d, should be %d."
			        , $time
			        , fifo_occupancy_i
			        , expected_occupancy_i
			        );


////////////////////////////////////////////////////////////////////////////////
// Testbench signals
////////////////////////////////////////////////////////////////////////////////

// Clock generation
initial
	begin
	clk_i = 1'b0;
	forever
		#5 clk_i = ~clk_i;
	end


// Reset generation
initial
	begin
		reset_i <= 1'b1;
		#100
		reset_i <= 1'b0;
	end


endmodule
