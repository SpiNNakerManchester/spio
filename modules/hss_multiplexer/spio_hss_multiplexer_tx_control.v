/**
 * Together with the spio_hss_multiplexer_rx_control module, handles the
 * lower-level details of operating the link:
 *  * Ensure 32-bit word-alignment
 *  * Perform a handshake with the remote system to ensure protocol
 *    compatibility and link stability.
 *  * Insert and filter clock correction sequences transparently into the
 *    stream.
 *  * Re-acquire 32-bit word-alignment upon loss of sync.
 *
 * Note that since handshakes are usually only restarted after a number of link
 * errors (e.g. due to disconnection) it is assumed that data is lost on the
 * link. As a result, the module discards the word it received in the cycle that
 * a handshake was restarted.
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
                                        
                                          // top-level control inputs
                                        , input wire  SCRMBL_IDL_DAT

                                          // register bank interface
                                        , input wire [`IDLE_BITS - 1:0] REG_IDSO_IN

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
                                        ,   input wire TXVLD_IN
                                        ,   output reg TXRDY_OUT
                                        );

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//--------------------- PRNG for idle frames --------------------
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
wire [15:0] lfsr_out;
wire        send_lfsr;

spio_hss_multiplexer_lfsr lfsr_0
(
  .clk      (CLK_IN),
  .rst      (RESET_IN),
  .next     (send_lfsr),
  .data_out (lfsr_out)
);
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


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

wire clock_correction_sequence_due_i = clock_correction_counter_i == 0;


// These signals trigger the transmission of the clock correction sequence and
// handshake a cycle after TXRDY_OUT was deasserted.
reg send_clock_correction_sequence_i;
reg send_handshake_i;
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN) send_handshake_i <= 1'b1;
	else          send_handshake_i <= !HANDSHAKE_COMPLETE_IN;
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN) send_clock_correction_sequence_i <= 1'b0;
	else          send_clock_correction_sequence_i <= clock_correction_sequence_due_i;


// decide what data to send on idle frames
assign send_lfsr = HANDSHAKE_COMPLETE_IN
                 && !clock_correction_sequence_due_i
                 && !TXVLD_IN
                 && SCRMBL_IDL_DAT
                 ;
wire [15:0] idle_data = send_lfsr ? lfsr_out : REG_IDSO_IN;

// Select the appropriate value to send.
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
		else if (send_handshake_i)
			begin
				TXDATA_OUT    <= { `KCH_COMMA
				                 , `KCH_HANDSHAKE
				                 , 7'b000000, HANDSHAKE_PHASE_IN
				                 , `PROTOCOL_VERSION
				                 };
				TXCHARISK_OUT <=  4'b1100;
			end
		else if (TXVLD_IN)
			begin
				TXDATA_OUT    <= TXDATA_IN;
				TXCHARISK_OUT <= TXCHARISK_IN;
			end
		else  // send idle frame
			begin
				TXDATA_OUT    <= {`KCH_IDLE, idle_data};
				TXCHARISK_OUT <= `IDLE_KBITS;
			end


//TODO: safe and efficient but may break the vld/rdy protocol!
// Allow packets to be transmitted by the rest of the system only if the
// handshake has been completed and we're not sending a clock-correction
// sequence.
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		TXRDY_OUT <= 1'b0;
	else
		TXRDY_OUT <= !clock_correction_sequence_due_i && HANDSHAKE_COMPLETE_IN;

endmodule

