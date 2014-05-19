/**
 * Module which allows a device to send data to a rdy/vld device running at
 * half the clock speed.
 *
 * Note that this module will misbehave if rdy or vld are deasserted when no
 * transfer has taken place.
 */


module spio_link_speed_halver #( parameter PKT_BITS = 72 )
                               ( // Reset the block
                                 input wire RESET_IN
                                 // The slower of the two clocks
                               , input wire SCLK_IN
                                 // The fast clock running at double the input
                                 // speed.
                               , input wire FCLK_IN
                                 // Incoming signals (on FCLK_IN)
                               , input  wire [PKT_BITS-1:0] DATA_IN
                               , input  wire                VLD_IN
                               , output reg                 RDY_OUT
                                 // Outgoing signals (on SCLK_IN)
                               , output reg  [PKT_BITS-1:0] DATA_OUT
                               , output reg                 VLD_OUT
                               , input  wire                RDY_IN
                               );

////////////////////////////////////////////////////////////////////////////////
// Slow-clock positive edge detection
////////////////////////////////////////////////////////////////////////////////

// Unfortunately it is not possible to just observe the slow clock value due to
// Xilinx synthesis constraints so instead we toggle a value on the slow clock
// and test to see if it changed to detect the positive edge.
reg sclk_toggler_i;
reg sclk_started_i;
always @ (posedge SCLK_IN, posedge RESET_IN)
	if (RESET_IN)
		begin
			sclk_toggler_i <= 1'b0;
			sclk_started_i <= 1'b0;
		end
	else
		begin
			sclk_toggler_i <= ~sclk_toggler_i;
			sclk_started_i <= 1'b1;
		end

reg last_sclk_toggler_i;
always @ (posedge FCLK_IN, posedge RESET_IN)
	if (RESET_IN)
		last_sclk_toggler_i <= 1'b0;
	else
		last_sclk_toggler_i <= sclk_toggler_i;

// We're on the positive edge if the last value and current value are the same
// (since it is changed on the positive edge).
reg sclk_i;
always @ (posedge FCLK_IN, posedge RESET_IN)
	if (RESET_IN)
		sclk_i <= 1'b0;
	else
		sclk_i <= sclk_started_i && (last_sclk_toggler_i == sclk_toggler_i);


////////////////////////////////////////////////////////////////////////////////
// Slow-clock sampled signals
////////////////////////////////////////////////////////////////////////////////

// The RDY_IN but only sampled on the posedge of the slow clock.
wire rdy_in_i;
reg last_rdy_in_i;
always @ (posedge FCLK_IN, posedge RESET_IN)
	if (RESET_IN) last_rdy_in_i <= 1'b0;
	else          last_rdy_in_i <= RDY_IN;
assign rdy_in_i = sclk_i ? last_rdy_in_i : RDY_IN;


////////////////////////////////////////////////////////////////////////////////
// Input parking
////////////////////////////////////////////////////////////////////////////////

// The incoming value will be forced to be parked if the output is not ready
// and something is already being sent.
wire wait_i = VLD_OUT && (!rdy_in_i);

// The incoming value must be parked
wire park_i = VLD_IN && RDY_OUT && wait_i;


// Park data which arrives when the output is blocked
reg [PKT_BITS-1:0] parked_data_i;
always @ (posedge FCLK_IN, posedge RESET_IN)
	if (RESET_IN)
		parked_data_i <= {PKT_BITS{1'bX}};
	else if (park_i)
		parked_data_i <= DATA_IN;


// State machine which indicates if a value is parked.
reg parked_i;
always @ (posedge FCLK_IN, posedge RESET_IN)
	if (RESET_IN)
		parked_i <= 1'b0;
	else
		if (park_i)
			parked_i <= 1'b1;
		else if (!sclk_i && !wait_i)
			parked_i <= 1'b0;


// The data to send next (the parked value or the current input)
wire [PKT_BITS-1:0] data_i = (parked_i ? parked_data_i : DATA_IN);
wire                vld_i  = (parked_i ?          1'b1 : VLD_IN);

// Become non-ready after every incoming transfer and only resume readiness
// during the second half of an output transfer cycle when there are no parked
// values.
always @ (posedge FCLK_IN, posedge RESET_IN)
	if (RESET_IN)
		RDY_OUT <= 1'b0;
	else if (VLD_IN && RDY_OUT)
		RDY_OUT <= 1'b0;
	else if (parked_i)
		RDY_OUT <= 1'b0;
	else if (sclk_i && rdy_in_i)
		RDY_OUT <= 1'b1;


////////////////////////////////////////////////////////////////////////////////
// Output control
////////////////////////////////////////////////////////////////////////////////

// Register data to the output every time a value is accepted on the input
always @ (posedge FCLK_IN, posedge RESET_IN)
	if (RESET_IN)
		DATA_OUT <= {PKT_BITS{1'bX}};
	else
		if ((vld_i && RDY_OUT && !wait_i) || (parked_i && rdy_in_i))
			DATA_OUT <= data_i;


////////////////////////////////////////////////////////////////////////////////
// Validity control
////////////////////////////////////////////////////////////////////////////////

// If an incoming transfer occurs during the first half of an slow clock cycle,
// we must remember this in order to mark the output as valid on the next
// slow-clock.
reg delayed_transfer_i;
always @ (posedge FCLK_IN, posedge RESET_IN)
	if (RESET_IN)
		delayed_transfer_i <= 1'b0;
	else
		delayed_transfer_i <= (vld_i && RDY_OUT && sclk_i);


// Become valid on the positive slow clock edge when either an incoming transfer
// has just occurred or one occurred during the first half of the last
// slow-clock cycle. The valid signal is lowered otherwise.
always @ (posedge FCLK_IN, posedge RESET_IN)
	if (RESET_IN)
		VLD_OUT <= 1'b0;
	else 
		if (!sclk_i)
			VLD_OUT <= parked_i || wait_i || (vld_i && RDY_OUT) || delayed_transfer_i;


endmodule

