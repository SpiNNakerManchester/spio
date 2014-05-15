/**
 * Testbench for the spio_link_speed_doubler.
 */


module spio_link_speed_doubler_tb;

localparam PKT_BITS = 16;

genvar i;

reg sclk_i;
reg fclk_i;
reg reset_i;

// Input stream
reg [PKT_BITS-1:0] in_data_i;
reg                in_vld_i;
wire               in_rdy_i;

// Output stream
wire [PKT_BITS-1:0] out_data_i;
wire                out_vld_i;
reg                 out_rdy_i;


////////////////////////////////////////////////////////////////////////////////
// Device under test
////////////////////////////////////////////////////////////////////////////////

spio_link_speed_doubler #( .PKT_BITS(PKT_BITS) )
spio_link_speed_doubler_i( .RESET_IN (reset_i)
                         , .SCLK_IN  (sclk_i)
                         , .FCLK_IN  (fclk_i)
                         , .DATA_IN  (in_data_i)
                         , .VLD_IN   (in_vld_i)
                         , .RDY_OUT  (in_rdy_i)
                         , .DATA_OUT (out_data_i)
                         , .VLD_OUT  (out_vld_i)
                         , .RDY_IN   (out_rdy_i)
                         );


////////////////////////////////////////////////////////////////////////////////
// Input Stimulus
////////////////////////////////////////////////////////////////////////////////

initial
	begin
	in_vld_i <= 1'b1;   out_rdy_i <= 1'b1;
	@(negedge reset_i)
	@(posedge sclk_i)
	
	$display("Ensure full-throughput");
	in_vld_i <= 1'b1;   out_rdy_i <= 1'b1;
	#200 @(posedge sclk_i)
	$display("Ensure things stop cleanly");
	in_vld_i <= 1'b0;   out_rdy_i <= 1'b1;
	#200 @(posedge sclk_i)
	$display("Ensure things start-up cleanly again");
	in_vld_i <= 1'b1;   out_rdy_i <= 1'b1;
	#200 @(posedge sclk_i)
	
	
	$display("Ensure we can block the link");
	in_vld_i <= 1'b1;   out_rdy_i <= 1'b0;
	#200 @(posedge sclk_i)
	$display("...and resume");
	in_vld_i <= 1'b1;   out_rdy_i <= 1'b1;
	#200 @(posedge sclk_i)
	
	
	$display("Ensure we can block the link at the same time as the stream.");
	@(posedge fclk_i)
	in_vld_i <= 1'b0;   out_rdy_i <= 1'b0;
	#200 @(posedge sclk_i)
	$display("...and resume to a blocked link");
	in_vld_i <= 1'b1;   out_rdy_i <= 1'b0;
	#200 @(posedge sclk_i)
	$display("...and then recover when unblocked mid-sclk");
	@(posedge fclk_i)
	in_vld_i <= 1'b1;   out_rdy_i <= 1'b1;
	#200 @(posedge sclk_i)
	
	
	// Job done!
	$finish;
	end

// Send packets with ascending values
reg [PKT_BITS-1:0] packets_sent_i;
always @ (posedge sclk_i, posedge reset_i)
	if (reset_i)
		begin
			in_data_i      <= 0;
			packets_sent_i <= 1;
		end
	else
		begin
			if (in_vld_i && in_rdy_i)
				begin
					in_data_i      <= packets_sent_i;
					packets_sent_i <= packets_sent_i + 1;
				end
		end

reg [PKT_BITS-1:0] next_arrival_i;

initial
	begin
		next_arrival_i = 0;
	end

// Print arriving output data
always @ (posedge fclk_i)
	if (out_vld_i && out_rdy_i)
		begin
			$display("Time=%8d; %018x", $time, out_data_i);
			
			if (out_data_i != next_arrival_i)
				$display("Time=%8d; %018x Missing!", $time, next_arrival_i);
			next_arrival_i = out_data_i + 1;
		end
	else
		$display("Time=%8d; Output Idle.", $time);


////////////////////////////////////////////////////////////////////////////////
// Testbench signals
////////////////////////////////////////////////////////////////////////////////

// Clock generation
initial
	begin
	sclk_i = 1'b1;
	forever
		#10 sclk_i = ~sclk_i;
	end

initial
	begin
	fclk_i = 1'b1;
	forever
		#5 fclk_i = ~fclk_i;
	end


// Reset generation
initial
	begin
		reset_i <= 1'b1;
		#100
		reset_i <= 1'b0;
	end


endmodule
