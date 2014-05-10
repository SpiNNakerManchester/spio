/**
 * Together with the spio_hss_multiplexer_rx_control module, handles the
 * lower-level details of operating the link:
 *  * Ensure 32-bit word-alignment
 *  * Perform a handshake with the remote system to ensure protocol
 *    compatibility and link stability.
 *  * Insert and filter clock correction sequences transparently into the
 *    stream.
 *  * Re-acquire 32-bit word-alignment upon loss of sync.
 */

`include "spio_hss_multiplexer_common.h"

module spio_hss_multiplexer_tx_control #( // The interval at which clock correction sequences should
                                          // be inserted (in cycles).
                                          parameter CLOCK_CORRECTION_INTERVAL = 1000
                                          // The number of bits for the above counter.
                                        , parameter CLOCK_CORRECTION_INTERVAL_BITS = 10
                                        )
                                        ( input wire  CLK_IN
                                        , input wire  RESET_IN
                                        
                                          // Has the handshake been completed
                                        , input wire HANDSHAKE_COMPLETE_IN
                                          // The phase of the handshake procedure
                                        , input wire HANDSHAKE_PHASE_IN
                                        
                                          // High-Speed-Serial Interface
                                            // Transmitter connections
                                        ,   output reg [31:0] TXDATA_OUT
                                        ,   output reg  [3:0] TXCHARISK_OUT
                                        
                                          // Internal 32-bit word interface
                                            // Data to send
                                        ,   input wire [31:0] TXDATA_IN
                                        ,   input wire  [3:0] TXCHARISK_IN
                                            // If high, the value on TXDATA_IN and TXCHARISK_IN will be
                                            // sent in the next cycle
                                        ,   output reg TXRDY_OUT
                                        );


// Trigger clock correction at the required interval
reg [CLOCK_CORRECTION_INTERVAL_BITS-1:0] clock_correction_counter_i;
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		clock_correction_counter_i <= 0;
	else
		if (clock_correction_counter_i < CLOCK_CORRECTION_INTERVAL)
			clock_correction_counter_i <= clock_correction_counter_i + 1;
		else
			clock_correction_counter_i <= 0;

wire send_clock_correction_sequence_i = clock_correction_counter_i == 0;


// Select the appropriate value to send
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		begin
			TXDATA_OUT    <= 32'h0; // Simulation Xes out if these arrive
			TXCHARISK_OUT <=  4'h0; // Don't wish to generate illegal K-chars
		end
	else
		if (send_clock_correction_sequence_i)
			begin
				TXDATA_OUT    <= {4{`KCH_CLKC}};
				TXCHARISK_OUT <=  4'b1111;
			end
		else if (!HANDSHAKE_COMPLETE_IN)
			begin
				TXDATA_OUT    <= { `KCH_COMMA
				                 , `KCH_HANDSHAKE
				                 , 7'b000000, HANDSHAKE_PHASE_IN
				                 , `VERSION
				                 };
				TXCHARISK_OUT <=  4'b1100;
			end
		else
			begin
				TXDATA_OUT    <= TXDATA_IN;
				TXCHARISK_OUT <= TXCHARISK_IN;
			end


// Allow packets to be transmitted by the rest of the system only if the
// handshake has been completed and we're not sending a clock-correction
// sequence.
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		TXRDY_OUT <= 1'b0;
	else
		TXRDY_OUT <= !send_clock_correction_sequence_i && HANDSHAKE_COMPLETE_IN;

endmodule

