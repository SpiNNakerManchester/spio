/**
 * Transmission control. Receives SpiNNaker packets and sends them out one byte
 * at a time. Also handles the insertion of synchronisation sequences.
 */

`include "spio_uart_common.h"

module spio_uart_tx_control ( // Common clock for synchronous signals
                              input wire CLK_IN
                              // Asynchronous active-high reset
                            , input wire RESET_IN
                              // A one-cycle pulse on this line will cause a
                              // synchronisation sequence to be sent after any
                              // in-transmission packets are sent.
                            , input wire SYNC_TRIGGER_IN
                              // This signal is high whenever synchronisation is taking
                              // place.
                            , output wire SYNCHRONISING_OUT
                              // Incoming packet stream
                            , input  wire [`PKT_LEN-1:0] PKT_DATA_IN
                            , input  wire                PKT_VLD_IN
                            , output wire                PKT_RDY_OUT
                              // A one-cycle pulse is produced on this line if
                              // the packet received in the previous cycle had
                              // invalid parity and was dropped.
                            , output wire PKT_DROPPED_OUT
                              // Outgoing byte stream
                            , output wire [7:0] BYTE_DATA_OUT
                            , output wire       BYTE_VLD_OUT
                            , input  wire       BYTE_RDY_IN
                            );

// The number of NULLs to insert when sending the synchronisation sequence.
localparam NULLS_TO_SEND = 4'd13;

// The transmitter is not producing any bytes
localparam STATE_IDLE = 0;

// Transmitting various parts of a SpiNNaker packet
localparam STATE_HEAD  = 1;
localparam STATE_BODY0 = 2;
localparam STATE_BODY1 = 3;
localparam STATE_BODY2 = 4;
localparam STATE_BODY3 = 5;
localparam STATE_BODY4 = 6; // Optional
localparam STATE_BODY5 = 7; // Optional
localparam STATE_BODY6 = 8; // Optional
localparam STATE_BODY7 = 9; // Optional

// Transmitting null bytes for synchronisation
localparam STATE_SYNC0 = 10;
// Transmitting an all-ones byte for synchronisation
localparam STATE_SYNC1 = 11;

// The packet to be transmitted (a byte-wise shift register)
reg [`PKT_BITS-1:0] pkt_i;

// The main state machine state
reg [3:0] state_i;

// A counter used to count the number of null bytes transmitted
reg [3:0] remaining_null_counter_i;

// Is the incoming packet's parity correct
wire pkt_data_parity_correct_i = (PKT_DATA_IN[1] ? (^PKT_DATA_IN[ `PKT_LEN-1:0])
                                                 : (^PKT_DATA_IN[`SPKT_LEN-1:0])
                                 ) == 1'b1;

// A register that remembers when a SYNC_TRIGGER_IN pulse arrives
reg sync_trigger_i;

// A signal which indicates when there is a SpiNNaker packet either arriving or
// in pkt_i with correct parity.
wire pkt_available_i = !PKT_RDY_OUT || (PKT_VLD_IN && pkt_data_parity_correct_i);

// Stores the packet length (as head is shifted out first)
reg is_long_packet_i;

// Synchronisation status
assign SYNCHRONISING_OUT = (state_i == STATE_SYNC0) || (state_i == STATE_SYNC1);

// Byte Output
assign BYTE_DATA_OUT = pkt_i[7:0];
assign BYTE_VLD_OUT  =  (state_i == STATE_HEAD)
                     || (state_i == STATE_BODY0) || (state_i == STATE_BODY1)
                     || (state_i == STATE_BODY2) || (state_i == STATE_BODY3)
                     || (state_i == STATE_BODY4) || (state_i == STATE_BODY5)
                     || (state_i == STATE_BODY6) || (state_i == STATE_BODY7)
                     ;



////////////////////////////////////////////////////////////////////////////////
// State machine
////////////////////////////////////////////////////////////////////////////////

always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		state_i <= STATE_IDLE;
	else
		case (state_i)
			STATE_IDLE:
				if (sync_trigger_i)
					// Priority entry into the sync state when requested.
					state_i <= STATE_SYNC0;
				else if (pkt_available_i)
					state_i <= STATE_HEAD;
			
			STATE_HEAD:  if (BYTE_RDY_IN) state_i <= STATE_BODY0;
			STATE_BODY0: if (BYTE_RDY_IN) state_i <= STATE_BODY1;
			STATE_BODY1: if (BYTE_RDY_IN) state_i <= STATE_BODY2;
			STATE_BODY2: if (BYTE_RDY_IN) state_i <= STATE_BODY3;
			STATE_BODY3:
				if (BYTE_RDY_IN)
					begin
						if (is_long_packet_i)
							state_i <= STATE_BODY4;
						else
							state_i <= STATE_IDLE;
					end
			
			STATE_BODY4: if (BYTE_RDY_IN) state_i <= STATE_BODY5;
			STATE_BODY5: if (BYTE_RDY_IN) state_i <= STATE_BODY6;
			STATE_BODY6: if (BYTE_RDY_IN) state_i <= STATE_BODY7;
			STATE_BODY7: if (BYTE_RDY_IN) state_i <= STATE_IDLE;
			
			STATE_SYNC0:
				if (remaining_null_counter_i == 4'd0)
					state_i <= STATE_SYNC1;
			
			STATE_SYNC1:
				if (BYTE_RDY_IN)
					state_i <= STATE_IDLE;
			
			default:
				state_i <= STATE_IDLE;
		endcase


////////////////////////////////////////////////////////////////////////////////
// Input capturing/readiness
////////////////////////////////////////////////////////////////////////////////

// Latch incoming data, shift out bytes
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		pkt_i <= {`PKT_BITS{1'bX}};
	else
		if (PKT_VLD_IN && PKT_RDY_OUT && pkt_data_parity_correct_i)
			pkt_i <= PKT_DATA_IN;
		else if (BYTE_RDY_IN && (  (state_i == STATE_HEAD)
		                        || (state_i == STATE_BODY0)
		                        || (state_i == STATE_BODY1)
		                        || (state_i == STATE_BODY2)
		                        || (state_i == STATE_BODY3)
		                        || (state_i == STATE_BODY4)
		                        || (state_i == STATE_BODY5)
		                        || (state_i == STATE_BODY6)
		                        || (state_i == STATE_BODY7)
		                        ))
			pkt_i <= {8'hXX, pkt_i[`PKT_LEN-1:8]};


// Record packet length
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		is_long_packet_i <= 1'bX;
	else
		if (PKT_VLD_IN && PKT_RDY_OUT)
			is_long_packet_i <= PKT_DATA_IN[1] == 1'b1;


// Report parity rejections
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		PKT_DROPPED_OUT <= 1'b0;
	else
		PKT_DROPPED_OUT <= PKT_VLD_IN && PKT_RDY_OUT && !pkt_data_parity_correct_i;


// Control input readiness
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		PKT_RDY_OUT <= 1'b0;
	else
		if (PKT_VLD_IN && PKT_RDY_OUT)
			PKT_RDY_OUT <= pkt_data_parity_correct_i;
		else if (state_i == STATE_IDLE)
			PKT_RDY_OUT <= 1'b1;


////////////////////////////////////////////////////////////////////////////////
// Null byte insertion counter
////////////////////////////////////////////////////////////////////////////////


always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		remaining_null_counter_i <= NULLS_TO_SEND - 4'd1;
	else
		if (state_i != STATE_SYNC0)
			remaining_null_counter_i <= NULLS_TO_SEND - 4'd1;
		else
			remaining_null_counter_i <= remaining_null_counter_i - 4'd1;


////////////////////////////////////////////////////////////////////////////////
// Synchronisation trigger latching
////////////////////////////////////////////////////////////////////////////////

always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		sync_trigger_i < 1'b0;
	else
		if (SYNC_TRIGGER_IN)
			sync_trigger_i <= 1'b1;
		else if (state_i == STATE_SYNC1)
			sync_trigger_i <= 1'b0;

endmodule

