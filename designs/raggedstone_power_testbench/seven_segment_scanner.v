/**
 * Scan a multiplexed seven-segment display.
 */

module seven_segment_scanner #( parameter NUM_DIGITS = 4
                              , parameter CLK_DIV_BITS = 10
                              )
                              ( input wire                       CLK_IN
                              , input wire                       RESET_IN
                              , input wire  [(4*NUM_DIGITS)-1:0] VALUE_IN
                              , output wire [6:0]                DISPLAY_OUT
                              , output wire [NUM_DIGITS-1:0]     DIGIT_ENABLE_OUT
                              );

// Divide down the clock
reg [CLK_DIV_BITS-1:0] clk_div_i;
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		clk_div_i <= 0;
	else
		clk_div_i <= clk_div_i + 1;


// The index of the current digit
reg [$clog2(NUM_DIGITS)-1:0] cur_digit_i;


// Cycle through each digit in turn
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		cur_digit_i <= 0;
	else
		if (clk_div_i == 0)
			begin
				if (cur_digit_i == NUM_DIGITS-1)
					cur_digit_i <= 0;
				else
					cur_digit_i <= (cur_digit_i + 1);
			end

// Select just the current digit
assign DIGIT_ENABLE_OUT = 1'b1 << cur_digit_i;

// Select just the correct digit to decode
seven_segment_decoder
seven_segment_decoder_i( .VALUE_IN(VALUE_IN[4*cur_digit_i+:4])
                       , .DISPLAY_OUT(DISPLAY_OUT)
                       );

endmodule
