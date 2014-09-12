/**
 * Together with the spio_hss_multiplexer_tx_control module, handles the
 * lower-level details of operating the link:
 *  * Ensure 32-bit word-alignment
 *  * Perform a handshake with the remote system to ensure protocol
 *    compatibility and link stability.
 *  * Filter clock correction sequences from stream. Insertion of these
 *    sequences is handled by the frame tx due to a limitation of the frame tx
 *    and disassembler and related components where they ignore ready/valid
 *    signals during a data frame. Clock correction sequences are still inserted
 *    during handshaking.

 *  * Re-acquire 32-bit word-alignment upon loss of sync.
 */

`include "spio_hss_multiplexer_common.h"

module spio_hss_multiplexer_rx_control #( // Number of consecutive handshakes which must arrive
                                          // before advancing the handshake phase.
                                          parameter NUM_HANDSHAKES = 100
                                          // Number of bits required for the above counter.
                                        , parameter NUM_HANDSHAKES_BITS = 7
                                        )
                                        ( input wire  CLK_IN
                                        , input wire  RESET_IN
                                        
                                          // Status Signals
                                            // Has the handshake been completed successfully?
                                        ,   output reg HANDSHAKE_COMPLETE_OUT
                                            // The phase number of the next handshake which should be
                                            // sent.
                                        ,   output wire HANDSHAKE_PHASE_OUT
                                            // High if the remote device is reporting the wrong
                                            // protocol version, low otherwise.
                                        ,   output reg VERSION_MISMATCH_OUT
                                        
                                          // High-Speed-Serial Interface
                                            // Receiver connections
                                        ,   input wire [31:0] RXDATA_IN
                                        ,   input wire  [3:0] RXCHARISCOMMA_IN
                                        ,   input wire  [3:0] RXCHARISK_IN
                                        ,   input wire  [1:0] RXLOSSOFSYNC_IN
                                        
                                          // Internal 32-bit word interface
                                            // Data to send
                                        ,   output reg [31:0] RXDATA_OUT
                                        ,   output reg  [3:0] RXCHARISK_OUT
                                            // If high, the value on RXDATA_OUT and TXCHARISK_OUT is
                                            // the currently read 32-bit word, if low, the value is
                                            // undefined.
                                        ,   output reg RXVLD_OUT
                                        );


wire is_byte_aligned_i = RXLOSSOFSYNC_IN == 2'b00;

////////////////////////////////////////////////////////////////////////////////
// Word alignment
////////////////////////////////////////////////////////////////////////////////

// Since commas may arrive at any point in the 4-bytes received each cycle, to
// produce an aligned 4-byte output, some of the previous cycle's bytes must be
// retained.

wire [31:0] this_rxdata_i    = RXDATA_IN;
wire  [3:0] this_rxcharisk_i = RXCHARISK_IN;
reg  [31:8] last_rxdata_i;
reg   [3:1] last_rxcharisk_i;

always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		begin
			last_rxdata_i    <= 32'b0;
			last_rxcharisk_i <=  4'b0;
		end
	else
		begin
			last_rxdata_i    <= RXDATA_IN[31:8];
			last_rxcharisk_i <= RXCHARISK_IN[3:1];
		end

wire [63:8] unaligned_rxdata_i    = {this_rxdata_i,    last_rxdata_i};
wire  [7:1] unaligned_rxcharisk_i = {this_rxcharisk_i, last_rxcharisk_i};

reg [31:0] aligned_rxdata_i;
reg  [3:0] aligned_rxcharisk_i;

// One-hot code indicating the byte/bit offset in
// unaligned_rxdata_i/unaligned_rxcharisk_i:
reg [3:0] alignment_i;

always @ (*)
	case (alignment_i)
		4'b0001: aligned_rxdata_i = unaligned_rxdata_i[39:8];
		4'b0010: aligned_rxdata_i = unaligned_rxdata_i[47:16];
		4'b0100: aligned_rxdata_i = unaligned_rxdata_i[55:24];
		4'b1000: aligned_rxdata_i = unaligned_rxdata_i[63:32];
		default: aligned_rxdata_i = 32'bX;
	endcase

always @ (*)
	case (alignment_i)
		4'b0001: aligned_rxcharisk_i = unaligned_rxcharisk_i[4:1];
		4'b0010: aligned_rxcharisk_i = unaligned_rxcharisk_i[5:2];
		4'b0100: aligned_rxcharisk_i = unaligned_rxcharisk_i[6:3];
		4'b1000: aligned_rxcharisk_i = unaligned_rxcharisk_i[7:4];
		default: aligned_rxcharisk_i = 4'bXXXX;
	endcase

// Detect the alignment when we're successfully byte-aligned since commas only
// exist as the first byte in a 32-bit sequence.
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		alignment_i <= 4'b0000;
	else
		if (!is_byte_aligned_i)
			alignment_i <= 4'b0000;
		else if (RXCHARISCOMMA_IN != 4'b0000)
			alignment_i <= RXCHARISCOMMA_IN;

wire is_word_aligned_i = is_byte_aligned_i && alignment_i != 4'b0000;


////////////////////////////////////////////////////////////////////////////////
// Check for clock correction packets
////////////////////////////////////////////////////////////////////////////////

// Detect clock correction sequence
wire is_clock_correction_sequence = is_word_aligned_i
                                  && aligned_rxdata_i    == {4{`KCH_CLKC}}
                                  && aligned_rxcharisk_i == 4'b1111
                                  ;

////////////////////////////////////////////////////////////////////////////////
// Handshake
////////////////////////////////////////////////////////////////////////////////

// The handshake mechanism starts by both devices sending a stream of words
// to the other containing four bytes:
//   * A comma for word alignment
//   * A K-char allowing the word to be identified uniquely as a handshake
//   * A 'phase' field (initially 0)
//   * The protocol version
//
// When the recipient has received a sufficient number of these (with the
// correct protocol version but any phase) consecutively to determine a base
// level of link stability, the phase field is incremented to 1 and more words
// are sent.
//
// As soon as a recipient gets a phase-1 handshake, they stop sending further
// handshakes and normal protocol usage proceeds.
//
// If a recipient receives a phase-0 handshake, they must restart the handshake
// process. This allows any device to restart the handshake process, for example
// if it never receives the phase-1 handshake from the remote end (and thus
// continues sending phase 0 syncs) or if it loses sync.

// Is the current packet a handshake (doesn't check the version or handshake
// cycle)
wire is_handshake_i = is_word_aligned_i
                    && aligned_rxdata_i[31:24] == `KCH_COMMA
                    && aligned_rxdata_i[23:16] == `KCH_HANDSHAKE
                    && aligned_rxcharisk_i == 4'b1100
                    ;

wire       handshake_phase_in_i   = is_handshake_i ? aligned_rxdata_i[8]   : 1'bX;
wire [7:0] handshake_version_in_i = is_handshake_i ? aligned_rxdata_i[7:0] : 8'bX;

// Version mismatch reporting
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		VERSION_MISMATCH_OUT <= 0;
	else
		if (is_handshake_i)
			VERSION_MISMATCH_OUT <= handshake_version_in_i != `PROTOCOL_VERSION;
		else
			VERSION_MISMATCH_OUT <= 1'b0;


// Keep track of the last handshake phase received. If we've gone backwards then
// this triggers a new handshake sequence.
reg last_handshake_phase_in_i;
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		last_handshake_phase_in_i <= 0;
	else
		if (is_handshake_i)
			last_handshake_phase_in_i <= handshake_phase_in_i;

// Restart the handshake if:
//   * We go out of word alignment.
//   * The remote device restarts the handshake.
//   * If the version number does not match (this simply prevents handshaking
//     completing with incompatible devices)
wire restart_handshake_i = !is_word_aligned_i
                         || (is_handshake_i && (handshake_phase_in_i   <  last_handshake_phase_in_i))
                         || (is_handshake_i && (handshake_version_in_i != `PROTOCOL_VERSION))
                         ;


// Count how many consecutive handshakes have been received, resetting the
// counter if the handshake is restarted or some other non-handshake (or
// non-clock-correction-sequence) data arrives.
reg [NUM_HANDSHAKES_BITS-1:0] handshake_counter_i;
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		handshake_counter_i <= 0;
	else
		if (is_handshake_i && !restart_handshake_i)
			begin
				if (handshake_counter_i < NUM_HANDSHAKES)
					handshake_counter_i <= handshake_counter_i + 1;
			end
		else if (!is_clock_correction_sequence)
			handshake_counter_i <= 0;

// Decide on the phase of the handshake. Starts in phase 0 and advances to 1
// once a sufficient number of consecutive handshakes have arrived.
assign HANDSHAKE_PHASE_OUT = handshake_counter_i == NUM_HANDSHAKES;

// Handshake completion is indicated by having advanced to phase 1 and receiving
// a phase 1 handshake.
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		HANDSHAKE_COMPLETE_OUT <= 1'b0;
	else
		if (restart_handshake_i)
			HANDSHAKE_COMPLETE_OUT <= 1'b0;
		else if (HANDSHAKE_PHASE_OUT == 1'b1 && handshake_phase_in_i == 1'b1)
			HANDSHAKE_COMPLETE_OUT <= 1'b1;



////////////////////////////////////////////////////////////////////////////////
// Input filtering
////////////////////////////////////////////////////////////////////////////////

// Filter unaligned data, clock-correction sequences and handshakes from the
// stream being received.

always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		begin
			RXDATA_OUT    <= 32'bX;
			RXCHARISK_OUT <= 4'bX;
			RXVLD_OUT     <= 1'b0;
		end
	else
		if ( is_word_aligned_i && HANDSHAKE_COMPLETE_OUT && !is_clock_correction_sequence && !is_handshake_i)
			begin
				RXDATA_OUT    <= aligned_rxdata_i;
				RXCHARISK_OUT <= aligned_rxcharisk_i;
				RXVLD_OUT     <= 1'b1;
			end
		else
			begin
				RXDATA_OUT    <= 32'bX;
				RXCHARISK_OUT <= 4'bX;
				RXVLD_OUT     <= 1'b0;
			end


endmodule
