/**
 * A simple SPI slave device which exposes a peek/poke style interface.
 *
 * The master should send an address of which bit 0 indiciates direction (0 =
 * read, 1 = write) and bit 1 is reserved and should be set to zero. Following
 * this the master can either send the value to poke or an ardbitary data when
 * peeking.
 *
 * This SPI device uses CPOL=0 and CPHA=0, i.e. sample bits on the positive-edge
 * of the clock and set-up the next bit on the falling edge. This implementation
 * expects that the SPI clock runs considerably slower than the system clock to
 * allow for actions to be taken between positive/negative clock edges of the
 * SPI clock.
 */

`define MIN(a,b) (((a)<(b)) ? (a) : (b))
`define MAX(a,b) (((a)<(b)) ? (b) : (a))

module spinnaker_fpgas_spi #( // Number of bits in an address
                              parameter ADDR_BITS = 32
                              // Number of bits in a value
                            , parameter VAL_BITS = 32
                            )
                            ( // System reset and clock
                              input wire RESET_IN
                            , input wire CLK_IN
                              
                              // SPI Signals (must be buffered and synchronised
                              // appropriately)
                                // SPI Clock (should much slower than the system
                                // clock).
                            ,   input  wire SCLK_IN
                                // Master output, slave input.
                            ,   input  wire MOSI_IN
                                // Master input, slave output. This should be
                                // externally tri-stated when NSS_IN is high.
                            ,   output reg  MISO_OUT
                                // Slave-select (active low).
                            ,   input  wire NSS_IN
                              
                              // Peek/Poke interface.
                                // Address of the value to be peeked/poked. If
                                // neither READ_OUT nor WRITE_OUT are high, this
                                // value should be ignored.
                            ,   output reg [ADDR_BITS-1:0] ADDRESS_OUT
                                // One-cycle strobe indicating a value is
                                // expected on READ_VALUE_IN next cycle.
                            ,   output reg READ_OUT
                                // One-cycle strobe indicating that
                                // WRITE_VALUE_OUT contains a value to write to
                                // the specified address.
                            ,   output reg WRITE_OUT
                                // Value to write when WRITE_OUT is high.
                            ,   output reg [VAL_BITS-1:0] WRITE_VALUE_OUT
                                // Value read, to be set on next cycle after
                                // READ_OUT.
                            ,   input wire [VAL_BITS-1:0] READ_VALUE_IN
                            );

// State machine states
localparam STATE_ADDR  = 2'd0;
localparam STATE_WRITE = 2'd1;
localparam STATE_READ  = 2'd2;


// Internal registers
reg [1:0] sclk_i;
reg [1:0] state_i;
reg [VAL_BITS-1:0] value_i;
reg [$clog2(`MAX(ADDR_BITS, VAL_BITS))-1:0] counter_i;


// Edge detector for SPI clock
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		sclk_i <= 2'b00;
	else
		sclk_i <= {sclk_i[0], SCLK_IN};

// SCLK edge signals
wire sclk_posedge_i = sclk_i == 2'b01;
wire sclk_negedge_i = sclk_i == 2'b10;


// Signals indicating when the counter has reached the final bit of a given
// field.
wire addr_last_bit_i = counter_i == ADDR_BITS - 1;
wire val_last_bit_i  = counter_i == VAL_BITS - 1;

// Counter for shifting in/out bits
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		counter_i <= 0;
	else
		// Clear the counter if not selected
		if (NSS_IN)
			counter_i <= 0;
		else if (sclk_posedge_i)
			begin
				// Clear the counter when it reaches its limit
				if (state_i == STATE_ADDR && addr_last_bit_i)
					counter_i <= 0;
				else if ((state_i == STATE_READ || state_i == STATE_WRITE)
				          && val_last_bit_i)
					counter_i <= 0;
				// Increment the counter otherwise
				else
					counter_i <= counter_i + 1;
			end


// Address shift register
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		ADDRESS_OUT <= 'bX;
	else
		if (sclk_posedge_i && state_i == STATE_ADDR)
			ADDRESS_OUT <= {ADDRESS_OUT[ADDR_BITS-2:0], MOSI_IN};


// Write value shift register
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		WRITE_VALUE_OUT <= 'bX;
	else
		if (sclk_posedge_i && state_i == STATE_WRITE)
			WRITE_VALUE_OUT <= {WRITE_VALUE_OUT[VAL_BITS-2:0], MOSI_IN};


// Read value shift register. Since the value is loaded on the shortly after the
// rising edge of the SPI clock (i.e. when the read strobe pulses after the last
// address bit), the MISO_OUT pin must be latched to ensure that it only changes
// on the SPI clock negative edge (to prevent timing violations). This is why
// the shift register is shifted on the positive edge of the SPI clock.
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		value_i <= 'bX;
	else
		if (READ_OUT)
			value_i <= READ_VALUE_IN;
		else if (sclk_posedge_i && state_i == STATE_READ)
			value_i <= {value_i[VAL_BITS-2:0], 1'bX};

// Expose the bits of the value being sent on the negative edge of the SPI clock
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		MISO_OUT <= 1'bX;
	else
		if (sclk_negedge_i)
			MISO_OUT <= value_i[VAL_BITS-1];


// Read strobe
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		READ_OUT <= 1'b0;
	else
		// Carry out the read immediately upon the recipt of the last address bit
		// when a read is to take place.
		READ_OUT <= sclk_posedge_i
		            && state_i == STATE_ADDR
		            && addr_last_bit_i
		            && MOSI_IN == 1'b0;

// Write strobe
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		WRITE_OUT <= 1'b0;
	else
		WRITE_OUT <= sclk_posedge_i && state_i == STATE_WRITE && val_last_bit_i;


// State-machine logic
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		state_i <= STATE_ADDR;
	else
		// Reset the state machine when not selected
		if (NSS_IN)
			state_i <= STATE_ADDR;
		else if (sclk_posedge_i)
			case (state_i)
				STATE_ADDR:
					if (sclk_posedge_i && addr_last_bit_i)
						state_i <= MOSI_IN ? STATE_WRITE : STATE_READ;
				
				STATE_READ,
				STATE_WRITE:
					if (sclk_posedge_i && val_last_bit_i)
						state_i <= STATE_ADDR;
				
				default:
					state_i <= STATE_ADDR;
			endcase

endmodule
