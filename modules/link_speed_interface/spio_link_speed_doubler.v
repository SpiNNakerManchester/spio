/**
 * Module which allows a device to send data to a rdy/vld device running at
 * double the clock speed.
 *
 * Note that this module will misbehave if rdy or vld are deasserted when no
 * transfer has taken place.
 *
 * This module may assert a ready signal half-way-through the slow-clock cycle.
 * It is assumed that this does not violate the set-up time for the slow-clocked
 * circuit.
 */


module spio_link_speed_doubler #( parameter PKT_BITS = 72 )
                                ( // Reset the block
                                  input wire RESET_IN
                                  // The slower of the two clocks
                                , input wire SCLK_IN
                                  // The fast clock running at double the input
                                  // speed.
                                , input wire FCLK_IN
                                  // Incoming signals (on SCLK_IN)
                                , input  wire [PKT_BITS-1:0] DATA_IN
                                , input  wire                VLD_IN
                                , output reg                 RDY_OUT
                                  // Outgoing signals (on FCLK_IN)
                                , output wire [PKT_BITS-1:0] DATA_OUT
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
reg sclk_posedge_i;
always @ (posedge FCLK_IN, posedge RESET_IN)
	if (RESET_IN)
		sclk_posedge_i <= 1'b0;
	else
		sclk_posedge_i <= sclk_started_i && (last_sclk_toggler_i == sclk_toggler_i);



////////////////////////////////////////////////////////////////////////////////
// Translation state-machine.
////////////////////////////////////////////////////////////////////////////////

assign DATA_OUT = DATA_IN;

// Waiting for a positive slow-clock-edge with valid data and a ready output
localparam IDLE = 1'b0;

// Deasserting the valid signal for the fast-clock-world but holding the ready
// for the slow-clock world
localparam SEND = 1'b1;

reg state_i;

always @ (posedge FCLK_IN, posedge RESET_IN)
	if (RESET_IN)
		begin
			state_i <= IDLE;
			RDY_OUT <= 1'b0;
			VLD_OUT <= 1'b0;
		end
	else
		case (state_i)
			IDLE:
				if (sclk_posedge_i)
					begin
						RDY_OUT <= RDY_IN;
						if (VLD_IN && RDY_IN)
							begin
								state_i <= SEND;
								VLD_OUT <= 1'b1;
							end
					end
			
			SEND:
				begin
					VLD_OUT <= 1'b0;
					state_i <= IDLE;
				end
			
			default:
				begin
					state_i <= IDLE;
					RDY_OUT <= 1'b0;
					VLD_OUT <= 1'b0;
				end
		endcase


endmodule
