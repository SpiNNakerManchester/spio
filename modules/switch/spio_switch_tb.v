/**
 * Simple testbench for the switch.
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

// The incrementing counter which gives the packet number
reg [31:0] input_counter_i;

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
assign in_data_i = { {(PKT_BITS-4-32-4-1){1'b0}}
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

initial
	begin
	// Initially make sure nothing happens when the input isn't valid
	`OSEL<=4'b0000; `VLD<=1'b0; `RDY<=4'b1111; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000;
	@(negedge reset_i)
	#100 @(posedge clk_i)
	
	// Unicast packets to unblocked outputs
	`OSEL<=4'b0001; `VLD<=1'b1; `RDY<=4'b1111; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000;
	#100 @(posedge clk_i)
	`OSEL<=4'b0010; `VLD<=1'b1; `RDY<=4'b1111; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000;
	#100 @(posedge clk_i)
	`OSEL<=4'b0100; `VLD<=1'b1; `RDY<=4'b1111; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000;
	#100 @(posedge clk_i)
	`OSEL<=4'b1000; `VLD<=1'b1; `RDY<=4'b1111; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000;
	#100 @(posedge clk_i)
	
	// Immediately drop packets with no destinations.
	`OSEL<=4'b0000; `VLD<=1'b1; `RDY<=4'b1111; `DRP<=1'b0; `SHDRP<=1'b1; `TODRP<=4'b0000;
	#100 @(posedge clk_i)
	
	// Multicast packets to unblocked outputs
	`OSEL<=4'b0001; `VLD<=1'b1; `RDY<=4'b1111; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000;
	#100 @(posedge clk_i)
	`OSEL<=4'b0011; `VLD<=1'b1; `RDY<=4'b1111; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000;
	#100 @(posedge clk_i)
	`OSEL<=4'b0111; `VLD<=1'b1; `RDY<=4'b1111; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000;
	#100 @(posedge clk_i)
	`OSEL<=4'b1111; `VLD<=1'b1; `RDY<=4'b1111; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000;
	#100 @(posedge clk_i)
	`OSEL<=4'b1110; `VLD<=1'b1; `RDY<=4'b1111; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000;
	#100 @(posedge clk_i)
	`OSEL<=4'b1100; `VLD<=1'b1; `RDY<=4'b1111; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000;
	#100 @(posedge clk_i)
	`OSEL<=4'b1000; `VLD<=1'b1; `RDY<=4'b1111; `DRP<=1'b0; `SHDRP<=1'b0; `TODRP<=4'b0000;
	#100 @(posedge clk_i)
	
	
	// Job done!
	$finish;
	end

////////////////////////////////////////////////////////////////////////////////
// Output checking
////////////////////////////////////////////////////////////////////////////////

// TODO Validate that no packets go missing

// Check the output port values
generate for (i = 0; i < 4; i = i + 1)
	begin : output_checker
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
					if (out_data_i[i][i+32+4] == 1'b1)
						$display( "Time=%08d: ERROR: Packet %018x at output %x should have been dropped."
						        , $time
						        , out_data_i[i]
						        , i
						        );
					
					// TODO: Check that every packet arrives
				end
	end
endgenerate


// Check the dropping port values
always @ (posedge clk_i, posedge reset_i)
	if (!reset_i && drop_vld_i)
		begin
			// Check that the packet was supposed to be dropped
			if (drop_data_i[4+32+4] != 1'b1)
				$display( "Time=%08d: ERROR: Packet %018x should not have been dropped."
				        , $time
				        , drop_data_i
				        );
			
			// Check that the packet was dropped due to the right outputs being
			// blocked.
			if (drop_data_i[32+4+:4] != drop_outputs_i)
				$display( "Time=%08d: ERROR: Packet %018x dropped due to outputs %x, should have been %x."
				        , $time
				        , drop_data_i
				        , drop_outputs_i
				        , drop_data_i[32+4+:4]
				        );
			
			// TODO: Check sequence number
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
