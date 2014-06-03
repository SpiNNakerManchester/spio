/**
 * Simple testbench for the switch.
 *
 * If everything is working, no errors should be printed out in the console.
 */


module spio_switch_tb;

localparam PKT_BITS = 72;

genvar i;

reg clk_i;
reg reset_i;

// Input
wire [PKT_BITS-1:0] in_data_i;
reg  [3:0]          in_output_select_i;
reg                 in_vld_i;
wire                in_rdy_i;

// Outputs
wire [(4*PKT_BITS)-1:0] out_data_bus_i;
wire [PKT_BITS-1:0]     out_data_i [3:0];
wire [3:0]              out_vld_i;
reg  [3:0]              out_rdy_i;

// Break out output data buses into an array
assign out_data_i[0] = out_data_bus_i[0*PKT_BITS+:PKT_BITS];
assign out_data_i[1] = out_data_bus_i[1*PKT_BITS+:PKT_BITS];
assign out_data_i[2] = out_data_bus_i[2*PKT_BITS+:PKT_BITS];
assign out_data_i[3] = out_data_bus_i[3*PKT_BITS+:PKT_BITS];

// Blockage status
wire [3:0] blocked_outputs_i;
wire [3:0] selected_outputs_i;

// Force packet drop signal
reg drop_i;

// Drop port
wire [PKT_BITS-1:0] drop_data_i;
wire [3:0]          drop_outputs_i;
wire                drop_vld_i;


////////////////////////////////////////////////////////////////////////////////
// Device under test
////////////////////////////////////////////////////////////////////////////////

spio_switch #( .PKT_BITS(PKT_BITS)
             , .NUM_PORTS(4)
             )
spio_switch_i( .CLK_IN              (clk_i)
             , .RESET_IN            (reset_i)
             // Input
             , .IN_DATA_IN          (in_data_i)
             , .IN_OUTPUT_SELECT_IN (in_output_select_i)
             , .IN_VLD_IN           (in_vld_i)
             , .IN_RDY_OUT          (in_rdy_i)
             // Outputs
             , .OUT_DATA_OUT        (out_data_bus_i)
             , .OUT_VLD_OUT         (out_vld_i)
             , .OUT_RDY_IN          (out_rdy_i)
             // Blockage status
             , .BLOCKED_OUTPUTS_OUT (blocked_outputs_i)
             , .SELECTED_OUTPUTS_OUT(selected_outputs_i)
             , .DROP_IN             (drop_i)
             // Drop port
             , .DROPPED_DATA_OUT    (drop_data_i)
             , .DROPPED_OUTPUTS_OUT (drop_outputs_i)
             , .DROPPED_VLD_OUT     (drop_vld_i)
             );


////////////////////////////////////////////////////////////////////////////////
// Input Stimulus
////////////////////////////////////////////////////////////////////////////////

localparam CNTR_WIDTH = 16;

// The incrementing counter which gives the packet number
reg [CNTR_WIDTH-1:0] input_counter_i;

// Increment the counter automatically whenever a packet is accepted
always @ (posedge clk_i, posedge reset_i)
	if (reset_i)
		input_counter_i <= 0;
	else
		if (in_vld_i && in_rdy_i)
				input_counter_i <= input_counter_i + 1;

// If something should be dropped, which outputs should be dropped
reg       should_drop_i;
reg [3:0] outputs_to_drop_i;

// Produce packets which contain the packet number, selected outputs and
// where it is expected to be dropped.
assign in_data_i = { {(PKT_BITS-4-CNTR_WIDTH-4-1){1'b0}}
                   , should_drop_i
                   , outputs_to_drop_i
                   , input_counter_i
                   , in_output_select_i
                   };

// Short aliases
`define OSEL  in_output_select_i
`define VLD   in_vld_i
`define RDY   out_rdy_i
`define DRP   drop_i
`define SHDRP should_drop_i
`define TODRP outputs_to_drop_i

// Run for a single cycle
`define TICK @(posedge clk_i)

// Run for a number of cycles
`define MTICK #100 @(posedge clk_i)

// Event triggered just before the simulation terminates.
event stimulus_ended;

initial
	begin
	// Initially make sure nothing happens when the input isn't valid
	`OSEL<=4'b0000; `VLD<=1'b0; `RDY<=4'b1111; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000;
	@(negedge reset_i)
	`MTICK;
	
	// Unicast packets to unblocked outputs
	`OSEL<=4'b0001; `VLD<=1'b1; `RDY<=4'b1111; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000;`MTICK;
	`OSEL<=4'b0010;                                                                      `MTICK;
	`OSEL<=4'b0100;                                                                      `MTICK;
	`OSEL<=4'b1000;                                                                      `MTICK;
	
	// Immediately drop packets with no destinations, even with blocked outputs
	`OSEL<=4'b0000; `VLD<=1'b1; `RDY<=4'b1111; `DRP<=1'b0; `SHDRP<=1'b1; `TODRP<=4'b0000;`MTICK;
	`OSEL<=4'b0000; `VLD<=1'b1; `RDY<=4'b0000; `DRP<=1'b0; `SHDRP<=1'b1; `TODRP<=4'b0000;`MTICK;
	
	// Multicast packets to unblocked outputs
	`OSEL<=4'b0001; `VLD<=1'b1; `RDY<=4'b1111; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000;`MTICK;
	`OSEL<=4'b0011;                                                                      `MTICK;
	`OSEL<=4'b0111;                                                                      `MTICK;
	`OSEL<=4'b1111;                                                                      `MTICK;
	`OSEL<=4'b1110;                                                                      `MTICK;
	`OSEL<=4'b1100;                                                                      `MTICK;
	`OSEL<=4'b1000;                                                                      `MTICK;
	
	// Unicast packets with blocking
	`OSEL<=4'b0001; `VLD<=1'b1; `RDY<=4'b0001; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000;`MTICK;
	                            `RDY<=4'b0000;                                           `MTICK;
	                            `RDY<=4'b0001;                                           `MTICK;
	`OSEL<=4'b0010;             `RDY<=4'b0010;                                           `MTICK;
	                            `RDY<=4'b0000;                                           `MTICK;
	                            `RDY<=4'b0010;                                           `MTICK;
	`OSEL<=4'b0100;             `RDY<=4'b0100;                                           `MTICK;
	                            `RDY<=4'b0000;                                           `MTICK;
	                            `RDY<=4'b0100;                                           `MTICK;
	`OSEL<=4'b1000;             `RDY<=4'b1000;                                           `MTICK;
	                            `RDY<=4'b0000;                                           `MTICK;
	                            `RDY<=4'b1000;                                           `MTICK;
	
	// Multicast packets with blocking of all ports
	`OSEL<=4'b1111; `VLD<=1'b1; `RDY<=4'b1111; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000;`MTICK;
	                            `RDY<=4'b0000;                                           `MTICK;
	                            `RDY<=4'b1111;                                           `MTICK;
	
	// Multicast packets with blocking of single ports
	`OSEL<=4'b1111; `VLD<=1'b1; `RDY<=4'b1111; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000;`MTICK;
	                            `RDY<=4'b1110;                                           `MTICK;
	                            `RDY<=4'b1111;                                           `MTICK;
	`OSEL<=4'b1111;             `RDY<=4'b1111;                                           `MTICK;
	                            `RDY<=4'b1101;                                           `MTICK;
	                            `RDY<=4'b1111;                                           `MTICK;
	`OSEL<=4'b1111;             `RDY<=4'b1111;                                           `MTICK;
	                            `RDY<=4'b1011;                                           `MTICK;
	                            `RDY<=4'b1111;                                           `MTICK;
	`OSEL<=4'b1111;             `RDY<=4'b1111;                                           `MTICK;
	                            `RDY<=4'b0111;                                           `MTICK;
	                            `RDY<=4'b1111;                                           `MTICK;
	
	// Dropping a blocking unicast packet: drain then inject a pair of packets into
	// a blocked network. The first packet should eventually get through (once
	// unblocked) but the second packet should be dropped.
	`OSEL<=4'b0000; `VLD<=1'b0; `RDY<=4'b1111; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000;`MTICK;
	`OSEL<=4'b0001; `VLD<=1'b1; `RDY<=4'b0000; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000; `TICK;
	`OSEL<=4'b0001; `VLD<=1'b1; `RDY<=4'b0000; `DRP<=1'b0; `SHDRP<=1'b1; `TODRP<=4'b0001; `TICK;
	`OSEL<=4'b0000; `VLD<=1'b0; `RDY<=4'b0000; `DRP<=1'b1; `SHDRP<=1'b0; `TODRP<=4'b0000; `TICK;
	`OSEL<=4'b0000; `VLD<=1'b0; `RDY<=4'b0000; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000; `TICK;
	`OSEL<=4'b0000; `VLD<=1'b0; `RDY<=4'b1111; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000; `TICK;
	
	// Dropping a partly-blocking unicast packet: drain then inject a pair of
	// packets into a blocked network. The first packet should eventually get
	// through, along with part of the second multicast packet (once unblocked)
	// but at least one output of the multicast packet will be dropped.
	`OSEL<=4'b0000; `VLD<=1'b0; `RDY<=4'b1111; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000;`MTICK;
	`OSEL<=4'b0001; `VLD<=1'b1; `RDY<=4'b0000; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000; `TICK;
	`OSEL<=4'b1111; `VLD<=1'b1; `RDY<=4'b0000; `DRP<=1'b0; `SHDRP<=1'b1; `TODRP<=4'b0001; `TICK;
	`OSEL<=4'b0000; `VLD<=1'b0; `RDY<=4'b0000; `DRP<=1'b1; `SHDRP<=1'b0; `TODRP<=4'b0000; `TICK;
	`OSEL<=4'b0000; `VLD<=1'b0; `RDY<=4'b0000; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000; `TICK;
	`OSEL<=4'b0000; `VLD<=1'b0; `RDY<=4'b1111; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000; `TICK;
	
	// Drain the network
	`OSEL<=4'b0000; `VLD<=1'b0; `RDY<=4'b1111; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000;`MTICK;
	
	// Job done!
	->stimulus_ended;
	#0
	$finish;
	end

////////////////////////////////////////////////////////////////////////////////
// Output checking
////////////////////////////////////////////////////////////////////////////////

// Validate that no packets go missing: This memory holds a bit for each output
// port which is set to 1 if a packet with the value at that address is
// expected.
reg packet_expected [3:0][(1<<CNTR_WIDTH)-1:0];

// Validate that all expected packets are dropped exactly once. A lookup from
// packet number to 5'b10000 if expecting to drop a destination-less packet, and
// 5'b0____ for a packet expecting to be dropped from certain outputs.
reg [4:0] drop_expected [(1<<CNTR_WIDTH)-1:0];

// Initialise with 0 for every bit
initial
	begin : packet_expected_init
		integer j; integer k;
		
		// Output expected lookup
		for (k = 0; k < 4; k = k + 1)
			for (j = 0; j < (1<<CNTR_WIDTH); j = j + 1)
				packet_expected[k][j] = 0;
		
		// Drop expected lookup
		for (j = 0; j < (1<<CNTR_WIDTH); j = j + 1)
			drop_expected[j] = 0;
		
		
	end

// Check every packet arrived at the end of the simulation
initial
	begin : packet_expected_final_check
		integer j; integer k;
		
		@(stimulus_ended);
		
		// Output expected lookup
		for (k = 0; k < 4; k = k + 1)
			for (j = 0; j < (1<<CNTR_WIDTH); j = j + 1)
				if (packet_expected[k][j] != 0)
					$display( "Time=%08d: ERROR: Packet with number %x was never delivered to output %x."
					        , $time, j, k);
		
		// Drop expected lookup
		for (j = 0; j < (1<<CNTR_WIDTH); j = j + 1)
			drop_expected[j] = 0;
			if (drop_expected[j][4] != 0)
				$display( "Time=%08d: ERROR: Packet with number %x was never dropped from output %x."
				        , $time, j, drop_expected[j][3:0]);
			else if (drop_expected[j][3:0] != 0)
				$display( "Time=%08d: ERROR: Packet with number %x was never dropped from some outputs %x."
				        , $time, j, drop_expected[j][3:0]);
		
		
	end

// Check the output port values
generate for (i = 0; i < 4; i = i + 1)
	begin : output_checker
		// Record packet IDs that are sent to this port
		always @ (posedge clk_i, posedge reset_i)
			if (!reset_i && in_vld_i && in_rdy_i && ( in_output_select_i[i]
			                                        & !outputs_to_drop_i[i]
			                                        & !(should_drop_i && outputs_to_drop_i == 4'h0)
			                                        ))
				packet_expected[i][input_counter_i] <= 1'b1;
		
		// Check packets arriving at the output
		always @ (posedge clk_i, posedge reset_i)
			if (!reset_i && out_vld_i[i] && out_rdy_i[i])
				begin
					// Check output port was intended as destination
					if (out_data_i[i][i] != 1'b1)
						$display( "Time=%08d: ERROR: Packet %018x unexpected on output %x."
						        , $time
						        , out_data_i[i]
						        , i
						        );
					
					// Check that packet wasn't supposed to be dropped
					if (out_data_i[i][i+CNTR_WIDTH+4] == 1'b1)
						$display( "Time=%08d: ERROR: Packet %018x at output %x should have been dropped."
						        , $time
						        , out_data_i[i]
						        , i
						        );
					
					// Check that packet was expected to arrive here
					if (packet_expected[i][out_data_i[i][4+:CNTR_WIDTH]] != 1'b1)
						$display( "Time=%08d: ERROR: Unexpected packet %018x with number %x at output %x."
						        , $time
						        , out_data_i[i]
						        , out_data_i[i][4+:CNTR_WIDTH]
						        , i
						        );
					
					// Clear the flag that the packet was due to arrive
					packet_expected[i][out_data_i[i][4+:CNTR_WIDTH]] <= 1'b0;
				end
	end
endgenerate

// Record packet IDs that are due to be dropped
always @ (posedge clk_i, posedge reset_i)
	if (!reset_i && in_vld_i && in_rdy_i && should_drop_i)
		drop_expected[input_counter_i] <= outputs_to_drop_i ? outputs_to_drop_i : 5'b10000;

// Check the dropping port values
always @ (posedge clk_i, posedge reset_i)
	if (!reset_i && drop_vld_i)
		begin
			// Check that the packet was supposed to be dropped
			if (drop_data_i[4+CNTR_WIDTH+4] != 1'b1)
				$display( "Time=%08d: ERROR: Packet %018x should not have been dropped."
				        , $time
				        , drop_data_i
				        );
			
			// Check that the packet was dropped due to the right outputs being
			// blocked.
			if (drop_data_i[CNTR_WIDTH+4+:4] != drop_outputs_i)
				$display( "Time=%08d: ERROR: Packet %018x dropped due to outputs %x, should have been %x."
				        , $time
				        , drop_data_i
				        , drop_outputs_i
				        , drop_data_i[CNTR_WIDTH+4+:4]
				        );
			
			// Check that a dropped packet has the expected ID
			if (!( ( (drop_expected[drop_data_i[4+:CNTR_WIDTH]] == 5'b10000)
			         && drop_outputs_i == 4'h0)
			     ||( (drop_expected[drop_data_i[4+:CNTR_WIDTH]][3:0] == drop_outputs_i)
			         && drop_outputs_i != 4'h0)
			     ))
				$display( "Time=%08d: ERROR: Unexpected packet dropped %018x with number %x."
				        , $time
				        , drop_data_i
				        , drop_data_i[4+:CNTR_WIDTH]
				        );
			
			// Clear the flag that the packet was due to be dropped
			drop_expected[drop_data_i[4+:CNTR_WIDTH]][4] <= 1'b0;
			drop_expected[drop_data_i[4+:CNTR_WIDTH]][3:0] <=
				drop_expected[drop_data_i[4+:CNTR_WIDTH]][3:0] & ~drop_outputs_i;
		end


// TODO: Validate BLOCKED_OUTPUTS_OUT

// TODO: Validate SELECTED_OUTPUTS_OUT

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
