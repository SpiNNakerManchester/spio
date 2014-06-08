/**
 * A UART receiver module including a byte buffer for incoming packets.
 */

`include "spio_uart_common.h"

module spio_uart_rx ( // Common clock for synchronous signals
                      input wire CLK_IN
                      // Asynchronous active-high reset
                    , input wire RESET_IN
                      // "Asynchronous" signals (incoming signals must be
                      // externally synchronised)
                        // Incoming serial stream
                    ,   input  wire RX_IN
                      // Synchronous signals
                        // Incoming 'bytes', no rdy signal: vld will appear for
                        // one cycle whenever a byte is received.
                    ,   output reg  [7:0] DATA_OUT
                    ,   output wire       VLD_OUT
                        // Single-cycle pulse from the baud generator 8 times
                        // per bit
                    ,   input wire SUBSAMPLE_PULSE_IN
                    );

////////////////////////////////////////////////////////////////////////////////
// Counter
////////////////////////////////////////////////////////////////////////////////

// A continuous 0-7 counter to count bit times.
reg  [2:0] counter_i;

always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		counter_i <= 3'h0;
	else
		if (SUBSAMPLE_PULSE_IN)
			counter_i <= counter_i + 1;


////////////////////////////////////////////////////////////////////////////////
// Sub-sampling state machine
////////////////////////////////////////////////////////////////////////////////

// Waiting for the initial transition due to a start bit to arrive.
localparam STATE_IDLE = 9;

// Waiting half-a-bit into the start bit to ensure it is stable
localparam STATE_START = 10;

// Receiving the byte (states must be in ascending order)
localparam STATE_BIT0 = 0;
localparam STATE_BIT1 = 1;
localparam STATE_BIT2 = 2;
localparam STATE_BIT3 = 3;
localparam STATE_BIT4 = 4;
localparam STATE_BIT5 = 5;
localparam STATE_BIT6 = 6;
localparam STATE_BIT7 = 7;

// Receiving a stop bit (does not check its value). (State must follow STATE_BIT7.)
localparam STATE_STOP = 8;


// The counter value at the time of bit arrival
reg [2:0] bit_time_i;

// A signal asserted for a single cycle coinciding with SUBSAMPLE_PULSE_IN when
// the input should be sampled.
wire sample_now_i = SUBSAMPLE_PULSE_IN && counter_i == bit_time_i;

// State register
reg [3:0] state_i;

always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		state_i <= STATE_IDLE;
	else
		if (SUBSAMPLE_PULSE_IN)
			case (state_i)
				STATE_IDLE:
					if (RX_IN == `START_BIT)
						begin
							// Expected bit centre is half a bit time from the initial
							// transition.
							state_i <= STATE_START;
							bit_time_i <= counter_i + 3'd4;
						end
				
				STATE_START:
					if (RX_IN != `START_BIT)
						// If a non-start-bit is seen then we just encountered a glitch
						state_i <= STATE_IDLE;
					else if (sample_now_i)
						state_i <= STATE_BIT0;
				
				STATE_BIT0, STATE_BIT1, STATE_BIT2, STATE_BIT3,
				STATE_BIT4, STATE_BIT5, STATE_BIT6, STATE_BIT7:
					if (sample_now_i)
						state_i <= state_i + 1;
				
				STATE_STOP:
					if (sample_now_i)
						state_i <= STATE_IDLE;
				
				default:
					state_i <= STATE_IDLE;
			endcase


////////////////////////////////////////////////////////////////////////////////
// Shift register
////////////////////////////////////////////////////////////////////////////////

always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		DATA_OUT <= 8'hXX;
	else
		if (SUBSAMPLE_PULSE_IN && bit_time_i == counter_i)
			case (state_i)
				STATE_BIT0, STATE_BIT1, STATE_BIT2, STATE_BIT3,
				STATE_BIT4, STATE_BIT5, STATE_BIT6, STATE_BIT7:
					DATA_OUT <= {RX_IN, DATA_OUT[7:1]};
				
				default:
					DATA_OUT <= DATA_OUT;
			endcase


////////////////////////////////////////////////////////////////////////////////
// Output
////////////////////////////////////////////////////////////////////////////////

// The data is fully shifted into the register while the stop bit is being
// received.
assign VLD_OUT          = state_i == STATE_STOP && sample_now_i;

endmodule

