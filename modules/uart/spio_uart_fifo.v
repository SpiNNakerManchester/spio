/**
 * A simple FIFO based on a cyclic buffer for storing incoming bytes from a
 * serial port.
 *
 * This FIFO is guaranteed to sensibly handle vld going low on its input before
 * a word has been accepted (which is normally not allowed by the rdy/vld
 * protocol used in spI/O).
 */

`include "spio_uart_common.h"

module spio_uart_fifo#( // The number of bits required to address the specified
                        // buffer size (i.e. the buffer will have size
                        // (1<<BUFFER_ADDR_BITS)-1).
                        parameter BUFFER_ADDR_BITS = 4
                        // The number of bits in a word in the buffer.
                      , parameter WORD_SIZE = 8
                      )
                      ( // Common clock for synchronous signals
                        input wire CLK_IN
                        // Asynchronous active-high reset
                      , input wire RESET_IN
                        // The number of words in the FIFO.
                      , output wire [BUFFER_ADDR_BITS-1:0] OCCUPANCY_OUT
                        // Input (rdy/vld protocol)
                      ,   input  wire [WORD_SIZE-1:0] IN_DATA_IN
                      ,   input  wire                 IN_VLD_IN
                      ,   output wire                 IN_RDY_OUT
                        // Output (rdy/vld protocol)
                      ,   output wire [WORD_SIZE-1:0] OUT_DATA_OUT
                      ,   output wire                 OUT_VLD_OUT
                      ,   input  wire                 OUT_RDY_IN
                      );

// A circular buffer where head_i points to the next empty slot and tail_i
// points to the last value in the buffer.
reg [WORD_SIZE-1:0]        buffer_i [(1<<BUFFER_ADDR_BITS)-1:0];
reg [BUFFER_ADDR_BITS-1:0] head_i;
reg [BUFFER_ADDR_BITS-1:0] tail_i;

// If the FIFO is empty, head_i and tail_i are equal.
assign OUT_VLD_OUT = head_i != tail_i;

// If the FIFO is full, head_i == tail_i-1.
assign IN_RDY_OUT = head_i != (tail_i - 1);

// The output data is just the current tail
assign OUT_DATA_OUT = (head_i != tail_i) ? buffer_i[tail_i] : {WORD_SIZE{1'bX}};

// The difference between the pointers positive-modulo the length of the buffer
// yields the number of words in the FIFO.
assign OCCUPANCY_OUT = head_i - tail_i;

// Advance (and fill) the head of the FIFO whenever valid data arrives
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		head_i <= {BUFFER_ADDR_BITS{1'b0}};
	else
		if (IN_RDY_OUT && IN_VLD_IN)
			begin
				head_i           <= head_i + 1;
				buffer_i[head_i] <= IN_DATA_IN;
			end


// Advance the tail whenever a value is output
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		tail_i <= {BUFFER_ADDR_BITS{1'b0}};
	else
		if (OUT_RDY_IN && OUT_VLD_OUT)
			tail_i <= tail_i + 1;

endmodule
