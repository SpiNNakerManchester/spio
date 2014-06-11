/**
 * Receive control. Receives bytes from the UART assembling them into SpiNNaker
 * packets. Also handles the consumption of synchronisation sequences.
 */

`include "spio_uart_common.h"

module spio_uart_rx_control ( // Common clock for synchronous signals
                              input wire CLK_IN
                              // Asynchronous active-high reset
                            , input wire RESET_IN
                              // This signal is high whenever synchronisation is
                              // taking place.
                            , output wire SYNCHRONISING_OUT
                              // Incoming byte stream. No vld signal, bytes are
                              // accepted in a single cycle. Note: this module
                              // relies on no two bytes arriving in consecutive
                              // cycles.
                            , input  wire [7:0] BYTE_DATA_IN
                            , input  wire       BYTE_VLD_IN
                              // Outgoing packet stream. No rdy signal, data and
                              // vld are produced for a single cycle.
                            , output reg  [`PKT_LEN-1:0] PKT_DATA_OUT
                            , output reg                 PKT_VLD_OUT
                            );

// Receiving various parts of a SpiNNaker packet
localparam STATE_HEAD  = 0;
localparam STATE_BODY0 = 1;
localparam STATE_BODY1 = 2;
localparam STATE_BODY2 = 3;
localparam STATE_BODY3 = 4;
localparam STATE_BODY4 = 5; // Optional
localparam STATE_BODY5 = 6; // Optional
localparam STATE_BODY6 = 7; // Optional
localparam STATE_BODY7 = 8; // Optional

// Single cycle varify checksum after each packet. Note: during this stage the
// incoming byte stream is ignored so any byte arriving during this cycle may be
// missed.
localparam STATE_VARIFY = 9;

// Receiving null bytes during synchronisation
localparam STATE_SYNC = 10;

// State machine state
reg [3:0] state_i;

// The register into which the incoming packet will be accumulated
reg [`PKT_LEN-1:0] pkt_i;

// The length of the incoming packet
wire is_long_packet_i = pkt_i[1] == 1'b1;

// Is the incoming packet's parity correct
wire parity_correct_i = (is_long_packet_i ? (^pkt_i[ `PKT_LEN-1:0])
                                          : (^pkt_i[`SPKT_LEN-1:0])
                        ) == 1'b1;

// Indicate when synchronisation is taking place
assign SYNCHRONISING_OUT = state_i == STATE_SYNC;

////////////////////////////////////////////////////////////////////////////////
// State machine
////////////////////////////////////////////////////////////////////////////////

always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		state_i <= STATE_SYNC;
	else
		case (state_i)
			STATE_HEAD:  if (BYTE_VLD_IN) state_i <= STATE_BODY0;
			STATE_BODY0: if (BYTE_VLD_IN) state_i <= STATE_BODY1;
			STATE_BODY1: if (BYTE_VLD_IN) state_i <= STATE_BODY2;
			STATE_BODY2: if (BYTE_VLD_IN) state_i <= STATE_BODY3;
			STATE_BODY3:
				if (BYTE_VLD_IN)
					begin
						if (is_long_packet_i)
							state_i <= STATE_BODY4;
						else
							state_i <= STATE_VARIFY;
					end
			
			STATE_BODY4: if (BYTE_VLD_IN) state_i <= STATE_BODY5;
			STATE_BODY5: if (BYTE_VLD_IN) state_i <= STATE_BODY6;
			STATE_BODY6: if (BYTE_VLD_IN) state_i <= STATE_BODY7;
			STATE_BODY7: if (BYTE_VLD_IN) state_i <= STATE_VARIFY;
			
			STATE_VARIFY:
				if (parity_correct_i)
					state_i <= STATE_HEAD;
				else
					state_i <= STATE_SYNC;
			
			STATE_SYNC:
				if (BYTE_VLD_IN && BYTE_DATA_IN == 8'hFF)
					state_i <= STATE_HEAD;
			
			default:
				state_i <= STATE_SYNC;
		endcase


////////////////////////////////////////////////////////////////////////////////
// Packet accumulation & output
////////////////////////////////////////////////////////////////////////////////

// Accumulate packet data
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		pkt_i <= {`PKT_LEN{1'bX}};
	else
		if (BYTE_VLD_IN)
			case (state_i)
				STATE_HEAD:  pkt_i[0*8+:8] <= BYTE_DATA_IN;
				STATE_BODY0: pkt_i[1*8+:8] <= BYTE_DATA_IN;
				STATE_BODY1: pkt_i[2*8+:8] <= BYTE_DATA_IN;
				STATE_BODY2: pkt_i[3*8+:8] <= BYTE_DATA_IN;
				STATE_BODY3: pkt_i[4*8+:8] <= BYTE_DATA_IN;
				STATE_BODY4: pkt_i[5*8+:8] <= BYTE_DATA_IN;
				STATE_BODY5: pkt_i[6*8+:8] <= BYTE_DATA_IN;
				STATE_BODY6: pkt_i[7*8+:8] <= BYTE_DATA_IN;
				STATE_BODY7: pkt_i[8*8+:8] <= BYTE_DATA_IN;
				
				default:
					pkt_i <= {`PKT_LEN{1'bX}};
			endcase


// On arrival of a complete packet, check parity and produce output for one
// cycle.
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		begin
			PKT_DATA_OUT <= {`PKT_LEN-1{1'bX}};
			PKT_VLD_OUT  <= 1'b0;
		end
	else
		if (state_i == STATE_VARIFY && parity_correct_i)
			begin
				PKT_DATA_OUT <= pkt_i;
				PKT_VLD_OUT  <= 1'b1;
			end
		else
			begin
				PKT_DATA_OUT <= {`PKT_LEN-1{1'bX}};
				PKT_VLD_OUT  <= 1'b0;
			end

endmodule

