/**
 * A simple N-flop synchroniser.
 */

module spio_uart_sync#( // The number of bits to synchronise
                        parameter NUM_BITS = 1
                        // The number of synchroniser flops to go through (must
                        // be at least 1)
                      , parameter NUM_STAGES = 2
                        // The value to initialise internal flops to (and thus
                        // the value during reset)
                      , parameter INITIAL_VALUE = 0
                      )
                      ( // Clock to sync to
                        input wire CLK_IN
                        // Asynchronous active-high reset
                      , input wire RESET_IN
                        // Signal to synchronise
                      , input wire [NUM_BITS-1:0] DATA_IN
                        // Synchronised signal
                      , output wire [NUM_BITS-1:0] DATA_OUT
                      );

genvar i;

// Data trickles from flop 0 up to flop NUM_STAGES-1.
reg [NUM_BITS-1:0] sync_flops_i [NUM_STAGES-1:0];

always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		sync_flops_i[0] <= INITIAL_VALUE;
	else
		sync_flops_i[0] <= DATA_IN;

generate for (i = 1; i < NUM_STAGES; i = i + 1)
	begin : sync_stages
		always @ (posedge CLK_IN, posedge RESET_IN)
			if (RESET_IN)
				sync_flops_i[i] <= INITIAL_VALUE;
			else
				sync_flops_i[i] <= sync_flops_i[i-1];
	end
endgenerate

assign DATA_OUT = sync_flops_i[0];

endmodule
