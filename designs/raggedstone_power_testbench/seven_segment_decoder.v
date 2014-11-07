/**
 * Simple Seven Segment Display Decoder.
 */

module seven_segment_decoder ( // Value to be represented in hex
                               input wire [3:0] VALUE_IN
                               // Segment states: seg a = bit 0, seg b = bit 1, ...
                             , output reg [6:0] DISPLAY_OUT
                             );

always @ (*)
	case (VALUE_IN)          // GFEDCBA
		4'h0:    DISPLAY_OUT = 7'b0111111;
		4'h1:    DISPLAY_OUT = 7'b0000110;
		4'h2:    DISPLAY_OUT = 7'b1011011;
		4'h3:    DISPLAY_OUT = 7'b1001111;
		4'h4:    DISPLAY_OUT = 7'b1100110;
		4'h5:    DISPLAY_OUT = 7'b1101101;
		4'h6:    DISPLAY_OUT = 7'b1111101;
		4'h7:    DISPLAY_OUT = 7'b0000111;
		4'h8:    DISPLAY_OUT = 7'b1111111;
		4'h9:    DISPLAY_OUT = 7'b1101111;
		4'hA:    DISPLAY_OUT = 7'b1110111;
		4'hB:    DISPLAY_OUT = 7'b1111100;
		4'hC:    DISPLAY_OUT = 7'b0111001;
		4'hD:    DISPLAY_OUT = 7'b1011110;
		4'hE:    DISPLAY_OUT = 7'b1111001;
		4'hF:    DISPLAY_OUT = 7'b1110001;
		default: DISPLAY_OUT = 7'bXXXXXXX;
	endcase

endmodule
