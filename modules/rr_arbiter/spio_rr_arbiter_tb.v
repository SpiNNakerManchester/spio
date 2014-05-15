/**
 * Simple testbench for the round-robbin arbiter.
 */


module spio_rr_arbiter_tb;

localparam PKT_BITS = 72;

genvar i;

reg clk_i;
reg reset_i;

// Input streams
reg [PKT_BITS-1:0] in_data_i [1:0];
reg                in_vld_i [1:0];
wire               in_rdy_i [1:0];

// Output stream
wire [PKT_BITS-1:0] out_data_i;
wire                out_vld_i;
reg                 out_rdy_i;


////////////////////////////////////////////////////////////////////////////////
// Device under test
////////////////////////////////////////////////////////////////////////////////

spio_rr_arbiter #( .PKT_BITS(PKT_BITS) )
spio_rr_arbiter_i( .CLK_IN   (clk_i)
                 , .RESET_IN (reset_i)
                 , .DATA0_IN (in_data_i[0])
                 , .VLD0_IN  (in_vld_i[0])
                 , .RDY0_OUT (in_rdy_i[0])
                 , .DATA1_IN (in_data_i[1])
                 , .VLD1_IN  (in_vld_i[1])
                 , .RDY1_OUT (in_rdy_i[1])
                 , .DATA_OUT (out_data_i)
                 , .VLD_OUT  (out_vld_i)
                 , .RDY_IN   (out_rdy_i)
                 );


////////////////////////////////////////////////////////////////////////////////
// Input Stimulus
////////////////////////////////////////////////////////////////////////////////

initial
	begin
	in_vld_i[0] <= 1'b1;   in_vld_i[1] <= 1'b1;   out_rdy_i <= 1'b1;
	@(negedge reset_i)
	
	// First see if fair arbitration occurrs
	#100 @(posedge clk_i)
	in_vld_i[0] <= 1'b1;   in_vld_i[1] <= 1'b1;   out_rdy_i <= 1'b1;
	// Next if nothing lost when we stop
	#100 @(posedge clk_i)
	in_vld_i[0] <= 1'b1;   in_vld_i[1] <= 1'b1;   out_rdy_i <= 1'b0;
	// Next if nothing lost when we and restart
	#100 @(posedge clk_i)
	in_vld_i[0] <= 1'b1;   in_vld_i[1] <= 1'b1;   out_rdy_i <= 1'b1;
	
	// Next see if both turning off does the trick
	#100 @(posedge clk_i)
	in_vld_i[0] <= 1'b0;   in_vld_i[1] <= 1'b0;   out_rdy_i <= 1'b1;
	// And nothing is confused when we stop
	#100 @(posedge clk_i)
	in_vld_i[0] <= 1'b0;   in_vld_i[1] <= 1'b0;   out_rdy_i <= 1'b0;
	// And restart...
	#100 @(posedge clk_i)
	in_vld_i[0] <= 1'b0;   in_vld_i[1] <= 1'b0;   out_rdy_i <= 1'b1;
	
	// Just one input forwarded at full throughput
	#100 @(posedge clk_i)
	in_vld_i[0] <= 1'b1;   in_vld_i[1] <= 1'b0;   out_rdy_i <= 1'b1;
	// Which we block...
	#100 @(posedge clk_i)
	in_vld_i[0] <= 1'b1;   in_vld_i[1] <= 1'b0;   out_rdy_i <= 1'b0;
	// And restart
	#100 @(posedge clk_i)
	in_vld_i[0] <= 1'b1;   in_vld_i[1] <= 1'b0;   out_rdy_i <= 1'b1;
	
	// Restart the other stream
	#100 @(posedge clk_i)
	in_vld_i[0] <= 1'b1;   in_vld_i[1] <= 1'b1;   out_rdy_i <= 1'b1;
	// And block them both again
	#100 @(posedge clk_i)
	in_vld_i[0] <= 1'b1;   in_vld_i[1] <= 1'b1;   out_rdy_i <= 1'b0;
	// And restart them again
	#100 @(posedge clk_i)
	in_vld_i[0] <= 1'b1;   in_vld_i[1] <= 1'b1;   out_rdy_i <= 1'b1;
	
	// Let the other run at full throughput
	#100 @(posedge clk_i)
	in_vld_i[0] <= 1'b0;   in_vld_i[1] <= 1'b1;   out_rdy_i <= 1'b1;
	// And block it...
	#100 @(posedge clk_i)
	in_vld_i[0] <= 1'b0;   in_vld_i[1] <= 1'b1;   out_rdy_i <= 1'b0;
	// And restart it...
	#100 @(posedge clk_i)
	in_vld_i[0] <= 1'b0;   in_vld_i[1] <= 1'b1;   out_rdy_i <= 1'b1;
	
	// Job done!
	#100 @(posedge clk_i)
	$finish;
	end

generate for (i = 0; i < 2; i = i + 1)
	begin: input_stimulus
		reg [PKT_BITS-4-1:0] packets_sent_i;
		
		always @ (posedge clk_i, posedge reset_i)
			if (reset_i)
				begin
					in_data_i[i]   <= {0, i==0 ? 4'hA : 4'hB};
					packets_sent_i <= 1;
				end
			else
				begin
					if (in_vld_i[i] && in_rdy_i[i])
						begin
							in_data_i[i]   <= {packets_sent_i, i==0 ? 4'hA : 4'hB};
							packets_sent_i <= packets_sent_i + 1;
						end
				end
	end
endgenerate

reg [PKT_BITS-4-1:0] next_arrival_i [1:0];

initial
	begin
		next_arrival_i[0] = 0;
		next_arrival_i[0] = 0;
	end

// Print arriving output data
always @ (posedge clk_i)
	if (out_vld_i && out_rdy_i)
		begin
			if (out_data_i[3:0] == 4'hA)
				begin
					$display("Tick=%8d; %018x", $time/10, out_data_i);
					
					if (out_data_i[PKT_BITS-1:4] != next_arrival_i[0])
						$display("Tick=%8d; %018x Missing!", $time/10, next_arrival_i[0]);
					next_arrival_i[0] = next_arrival_i[0]+1;
				end
			else
				begin
					$display("Tick=%8d;                                           %018x", $time/10, out_data_i);
					
					if (out_data_i[PKT_BITS-1:4] != next_arrival_i[1])
						$display("Tick=%8d;                                           %018x Missing", $time/10, next_arrival_i[1]);
					next_arrival_i[1] = next_arrival_i[1]+1;
				end
		end
	else
		$display("Tick=%8d; Output Idle.", $time/10);


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
