/**
 * A UART transmitter module.
 */

`include "spio_uart_common.h"

module spio_uart_tx ( // Common clock for synchronous signals
                      input wire CLK_IN
                      // Asynchronous active-high reset
                    , input wire RESET_IN
                      // Synchronous signals
                        // Incoming 'bytes' (using a rdy/vld protocol)
                    ,   input  wire [7:0] DATA_IN
                    ,   input  wire       VLD_IN
                    ,   output reg        RDY_OUT
                        // Single-cycle pulse from the baud generator for every bit
                    ,   input wire BAUD_PULSE_IN
                      // "Asynchronous" serial stream
                    , output reg TX_OUT
                    );

// Initial state
localparam STATE_RESET = 8;

// Waiting for a byte to transmit
localparam STATE_IDLE = 9;

// Sending a start bit
localparam STATE_START = 10;

// Transmitting the stop bit
localparam STATE_STOP = 11;

// Transmitting the byte
localparam STATE_BIT0 = 0;
localparam STATE_BIT1 = 1;
localparam STATE_BIT2 = 2;
localparam STATE_BIT3 = 3;
localparam STATE_BIT4 = 4;
localparam STATE_BIT5 = 5;
localparam STATE_BIT6 = 6;
localparam STATE_BIT7 = 7;

// State register
reg [3:0] state_i;

// Shift register out of which values will be transmitted
reg [7:0] data_i;

////////////////////////////////////////////////////////////////////////////////
// State machine
////////////////////////////////////////////////////////////////////////////////

always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		state_i <= STATE_RESET;
	else
		if (state_i == STATE_RESET)
			state_i <= STATE_IDLE;
		else if (BAUD_PULSE_IN)
			case (state_i)
				STATE_IDLE: if (!RDY_OUT||VLD_IN) state_i <= STATE_START;
				STATE_START:                      state_i <= STATE_BIT0;
				STATE_BIT0:                       state_i <= STATE_BIT1;
				STATE_BIT1:                       state_i <= STATE_BIT2;
				STATE_BIT2:                       state_i <= STATE_BIT3;
				STATE_BIT3:                       state_i <= STATE_BIT4;
				STATE_BIT4:                       state_i <= STATE_BIT5;
				STATE_BIT5:                       state_i <= STATE_BIT6;
				STATE_BIT6:                       state_i <= STATE_BIT7;
				STATE_BIT7:                       state_i <= STATE_STOP;
				STATE_STOP: if (!RDY_OUT||VLD_IN) state_i <= STATE_START;
				            else                  state_i <= STATE_IDLE;
				
				default:
					state_i <= STATE_RESET;
			endcase


////////////////////////////////////////////////////////////////////////////////
// Input readiness
////////////////////////////////////////////////////////////////////////////////

// Can accept a new data value at any time after the old one has been
// transmitted
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		RDY_OUT <= 1'b0;
	else
		if (RDY_OUT && VLD_IN)
			RDY_OUT <= 1'b0;
		else if (state_i == STATE_RESET)
			RDY_OUT <= 1'b1;
		else if (state_i == STATE_BIT7 && BAUD_PULSE_IN)
			RDY_OUT <= 1'b1;


////////////////////////////////////////////////////////////////////////////////
// Shift register
////////////////////////////////////////////////////////////////////////////////

always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		data_i <= 8'hXX;
	else
		if (RDY_OUT && VLD_IN)
			data_i <= DATA_IN;
		else if (BAUD_PULSE_IN
		        && (  state_i == STATE_BIT0 || state_i == STATE_BIT1
		           || state_i == STATE_BIT2 || state_i == STATE_BIT3
		           || state_i == STATE_BIT4 || state_i == STATE_BIT5
		           || state_i == STATE_BIT6 || state_i == STATE_BIT7
		           )
		        )
			data_i <= {1'bX, data_i[7:1]};


////////////////////////////////////////////////////////////////////////////////
// Output (combinatorial)
////////////////////////////////////////////////////////////////////////////////

always @ (*)
	if (RESET_IN)
		TX_OUT = `LINE_IDLE;
	else
		case (state_i)
			default:
				TX_OUT = `LINE_IDLE;
			
			STATE_RESET,
			STATE_IDLE:
				TX_OUT = `LINE_IDLE;
			
			STATE_START:
				TX_OUT = `START_BIT;
			
			STATE_BIT0, STATE_BIT1, STATE_BIT2, STATE_BIT3,
			STATE_BIT4, STATE_BIT5, STATE_BIT6, STATE_BIT7:
				TX_OUT = data_i[0];
			
			STATE_STOP:
				TX_OUT = `STOP_BIT;
		endcase


endmodule
