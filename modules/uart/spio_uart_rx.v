/**
 * A UART receiver module including a byte buffer for incoming packets.
 */

`include "spio_uart_common.h"

module spio_uart_rx#( // The number of bits required to address the specified
                      // buffer size (i.e. the buffer will have size
                      // 1<<BUFFER_ADDR_BITS). If set to zero, no buffer will be
                      // used.
                      parameter BUFFER_ADDR_BITS = 4
                      // The high water mark for buffer occupancy beyond which
                      // the clear-to-send signal is deasserted.
                    , parameter HIGH_WATER_MARK = 8
                    )
                    ( // Common clock for synchronous signals
                      input wire CLK_IN
                      // Asynchronous active-high reset
                    , input wire RESET_IN
                      // "Asynchronous" signals (incoming signals must be
                      // externally synchronised)
                        // Incoming serial stream
                    ,   input  wire RX_IN
                        // Clear to send signal (asserted when the internal FIFO
                        // passes the high water mark).
                    ,   output wire CTS_OUT
                      // Synchronous signals
                        // Incoming 'bytes' using a rdy/vld protocol. If the
                        // BUFFER_ADDR_BITS has been set to 0, the rdy signal is
                        // ignored and data and vld will appear for one cycle
                        // whenever a byte is received.
                    ,   output wire [7:0] DATA_OUT
                    ,   output wire       VLD_OUT
                    ,   input  wire       RDY_IN
                        // This signal pulses for one clock cycle whenever a
                        // byte is dropped due to the FIFO being full. This
                        // signal is always 0 when BUFFER_ADDR_BITS is 0.
                    ,   output wire BYTE_DROPPED_OUT
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

// Shift register into which data will be received
reg [7:0] data_i;

always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		data_i <= 8'hXX;
	else
		if (SUBSAMPLE_PULSE_IN && bit_time_i == counter_i)
			case (state_i)
				STATE_BIT0, STATE_BIT1, STATE_BIT2, STATE_BIT3,
				STATE_BIT4, STATE_BIT5, STATE_BIT6, STATE_BIT7:
					data_i <= {RX_IN, data_i[7:1]};
				
				default:
					data_i <= data_i;
			endcase


////////////////////////////////////////////////////////////////////////////////
// Output
////////////////////////////////////////////////////////////////////////////////

generate if (BUFFER_ADDR_BITS == 0)
	begin : unbuffered_output
		// The data is fully shifted into the register while the stop bit is being
		// received.
		assign DATA_OUT         = data_i;
		assign VLD_OUT          = state_i == STATE_STOP && sample_now_i;
		assign CTS_OUT          = !RESET_IN;
		assign BYTE_DROPPED_OUT = 1'b0;
	end
else
	begin : buffered_output
		// Bytes are placed in the FIFO as soon as it arrives. If the FIFO is not
		// ready (i.e. it is full) the byte is dropped.
		wire data_vld_i = state_i == STATE_STOP && sample_now_i;
		wire data_rdy_i;
		
		// Bytes are dropped whenever a byte is available but the FIFO is full.
		assign BYTE_DROPPED_OUT = data_vld_i && !data_rdy_i;
		
		// Drop CTS if the occupancy rises above the specified high-water mark.
		wire [BUFFER_ADDR_BITS-1:0] fifo_occupancy_i;
		assign CTS_OUT = !RESET_IN && fifo_occupancy_i < HIGH_WATER_MARK;
		
		// The FIFO to store the incoming bytes
		spio_uart_fifo #( .BUFFER_ADDR_BITS(BUFFER_ADDR_BITS)
		                , .WORD_SIZE(8)
		                )
		spio_uart_fifo_i( .CLK_IN       (CLK_IN)
		                , .RESET_IN     (RESET_IN)
		                , .OCCUPANCY_OUT(fifo_occupancy_i)
		                , .IN_DATA_IN   (data_i)
		                , .IN_VLD_IN    (data_vld_i)
		                , .IN_RDY_OUT   (data_rdy_i)
		                , .OUT_DATA_OUT (DATA_OUT)
		                , .OUT_VLD_OUT  (VLD_OUT)
		                , .OUT_RDY_IN   (RDY_IN)
		                );
		
	end
endgenerate

endmodule

