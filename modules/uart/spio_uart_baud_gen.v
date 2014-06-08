/**
 * A UART baud clock generator which produces a single-cycle pulse at a regular
 * interval (i.e. a timer). Also produces a second signal which pulses at a
 * approximately eight times this rate for the purpose of subsampling an
 * incoming signal at the given baudrate.
 */

`include "spio_uart_common.h"

module spio_uart_baud_gen#( // The number of ticks before the timer expires.
                            parameter PERIOD = 100
                            // The number of bits to use for the internal timer
                            // (must be enough to represent PERIOD-1)
                          , parameter NUM_BITS = 7
                          )
                          ( // Input clock source
                            input wire CLK_IN
                            // Asynchronous active-high reset
                          , input wire RESET_IN
                            // A single-cycle pulse every PERIOD clock ticks
                          , output wire BAUD_PULSE_OUT
                            // A single-cycle pulse (approximately) every
                            // PERIOD/8 clock ticks, for sub-sampling an
                            // incoming signal.
                          , output wire SUBSAMPLE_PULSE_OUT
                          );

// The baud pulse counter
reg [NUM_BITS-1:0] counter_i;
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		counter_i <= PERIOD-1;
	else
		if (counter_i == 0)
			counter_i <= PERIOD-1;
		else
			counter_i <= counter_i - 1;

assign BAUD_PULSE_OUT = counter_i == {NUM_BITS{1'b0}};

// The subsampling pulse counter
reg [NUM_BITS-3-1:0] counter8_i;
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		counter8_i <= (PERIOD>>3)-1;
	else
		if (counter8_i == 0)
			counter8_i <= (PERIOD>>3)-1;
		else
			counter8_i <= counter8_i - 1;

assign SUBSAMPLE_PULSE_OUT = counter8_i == {(NUM_BITS-3){1'b0}};

endmodule
