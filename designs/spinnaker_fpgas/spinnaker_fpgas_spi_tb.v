/**
 * A testbench for the SPI slave.
 *
 * In this test, values are written/read from the FPGA at ascending addresses
 * (ascending 4 addresses each time), the value written will always be the
 * negation of the address, the value read must always be the negation of the
 * address. Writes will occur when bit 2 is high and reads otherwise.
 */

`timescale 1ns / 1ps

module spinnaker_fpgas_spi_tb;

localparam CLK_PERIOD  =  10;
localparam SCLK_PERIOD = 100;

localparam DIR_BITS  =  8;
localparam ADDR_BITS = 32;
localparam VAL_BITS  = ADDR_BITS;

////////////////////////////////////////////////////////////////////////////////
// Registers/signals
////////////////////////////////////////////////////////////////////////////////

// System clock/reset
reg clk_i;
reg reset_i;

// SPI signals
reg  sclk_i;
reg  mosi_i;
wire miso_i;
reg  nss_i;

// Peek/Poke signals
wire [ADDR_BITS-1:0] address_i;
wire                 read_i;
wire                 write_i;
wire [VAL_BITS-1:0]  write_value_i;
wire [VAL_BITS-1:0]  read_value_i;


////////////////////////////////////////////////////////////////////////////////
// Test driver variables
////////////////////////////////////////////////////////////////////////////////

// Address of next read/write to be driven. Note: for correct operation, the
// peek/poke checker requires that this field is updated after nss_i has gone
// low when a value is actually transmitted.
reg [ADDR_BITS-3:0] next_address_i;

// Address of next read/write expected by the peek/poke checker
reg [ADDR_BITS-3:0] expected_address_i;

// A bit returned by SPI reads which is always ignored
reg ignored_bit;

////////////////////////////////////////////////////////////////////////////////
// Device under test
////////////////////////////////////////////////////////////////////////////////

spinnaker_fpgas_spi #( .DIR_BITS  (DIR_BITS)
                     , .ADDR_BITS (ADDR_BITS)
                     , .VAL_BITS  (VAL_BITS)
                     )
spinnaker_fpgas_spi_i( .CLK_IN   (clk_i)
                     , .RESET_IN (reset_i)
                        // SPI
                     , .SCLK_IN  (sclk_i)
                     , .MOSI_IN  (mosi_i)
                     , .MISO_OUT (miso_i)
                     , .NSS_IN   (nss_i)
                       // Peek/Poke
                     , .ADDRESS_OUT     (address_i)
                     , .READ_OUT        (read_i)
                     , .WRITE_OUT       (write_i)
                     , .WRITE_VALUE_OUT (write_value_i)
                     , .READ_VALUE_IN   (read_value_i)
                     );


////////////////////////////////////////////////////////////////////////////////
// Peek/Poke Interface Checker
////////////////////////////////////////////////////////////////////////////////

// Check nothing is being sent/received while the module is not selected.
// (Note: absuses knowledge that the module strobes the peek/poke during the
// middle/final bit of the transfer before the slave select signal has chance to
// change).
always @ (posedge clk_i, posedge reset_i)
	if (!reset_i && (write_i || read_i) && (nss_i))
		$display("%0d: ERROR: Peek/poke attempted while not selected!", $time);


// Check that no addresses are missed. This check also serves to check that
// strobes are only present for a single cycle (though obviously only in a case
// where there isn't a fault which incremenets the address...).
always @ (posedge clk_i, posedge reset_i)
	if (reset_i)
		expected_address_i <= 0;
	else
		if (nss_i)
			expected_address_i <= next_address_i;
		else if ((write_i || read_i))
			begin
				if (address_i != {expected_address_i, 1'b0, expected_address_i[0]})
					$display( "%0d: ERROR: Address 0x%0X accessed, expected 0x%0X."
					        , $time
					        , address_i
					        , {expected_address_i, 1'b0, expected_address_i[0]}
					        );
				expected_address_i <= expected_address_i + 1;
			end


// Check whether the correct operation has occurred given the address
always @ (posedge clk_i, posedge reset_i)
	if (!reset_i && ((write_i && !address_i[0]) || (read_i && address_i[0])))
		$display( "%0d: ERROR: At address 0x%0X a %s is expected, got a %s."
		        , $time
		        , address_i
		        , address_i[0] ? "write" : "read"
		        , address_i[0] ? "read" : "write"
		        );


// Check written values are inversions of the address
always @ (posedge clk_i, posedge reset_i)
	if (!reset_i && write_i && (write_value_i != ~address_i))
		$display( "%0d: ERROR: At address 0x%0X, wrote 0x%0X, expected 0x%0X."
		        , $time
		        , address_i
		        , write_value_i
		        , ~address_i
		        );


// Combinatorial. Respond with a valid "read" value only during the read strobe
// (i.e. readly to be picked up on the clock-edge after).
assign read_value_i = read_i ? ~address_i : {VAL_BITS{1'bX}};


////////////////////////////////////////////////////////////////////////////////
// SPI interface test driver
////////////////////////////////////////////////////////////////////////////////


/**
 * A task which places a bit on the SPI 
 */
task spi_bit;
	input ibit;
	output obit;
	begin
		// Set up the bit to send
		mosi_i <= ibit;
		
		// On the rising clock, sample the bit receieved
		#(SCLK_PERIOD/2)
		sclk_i <= 1'b1;
		obit <= miso_i;
		
		// Finish on the falling edge of the clock
		#(SCLK_PERIOD/2)
		sclk_i <= 1'b0;
	end
endtask


/**
 * Send an address over SPI.
 */
task spi_addr;
	input reg [ADDR_BITS-1:0] addr;
	integer i;
	begin
		for (i = ADDR_BITS-1; i >= 0; i = i - 1)
			spi_bit(addr[i], ignored_bit);
	end
endtask


/**
 * Send/receive a value over SPI.
 */
task spi_val;
	input  reg [VAL_BITS-1:0] ival;
	output reg [VAL_BITS-1:0] oval;
	integer i;
	begin
		for (i = VAL_BITS-1; i >= 0; i = i - 1)
			spi_bit(ival[i], oval[i]);
	end
endtask


/**
 * Perform whatever the next read/write operation is and validate the response.
 */
task spi_command;
	reg [VAL_BITS-1:0] val;
	reg [ADDR_BITS-3:0] addr;
	begin
		addr = next_address_i;
		next_address_i = next_address_i + 1;
		
		spi_addr({addr, 1'b0, addr[0]});
		spi_val(~{addr, 1'b0, addr[0]}, val);
		
		// Validate when the device is selected and a read occurs.
		if (~nss_i && ~addr[0] && val != ~{addr, 1'b0, addr[0]})
			$display( "%0d: ERROR: Read from 0x%0X returned 0x%0X expected 0x%0X."
			        , $time
			        , {addr, 1'b0, addr[0]}
			        , val
			        , ~{addr, 1'b0, addr[0]}
			        );
	end
endtask

// Main test routine
initial
	begin
	// Start with the SPI device not-selected and the clock in its base state
	nss_i  <= 1'b1;
	sclk_i <= 1'b0;
	next_address_i <= 0;
	
	// Hold the system in reset
	reset_i <= 1'b1;
	#(CLK_PERIOD * 10)
	reset_i <= 1'b0;
	#(CLK_PERIOD * 10)
	
	// Test that the device ignores everything when not selected
	spi_command();
	spi_command();
	spi_command();
	spi_command();
	
	// Test that values do arrive when selected
	nss_i <= 1'b0;
	spi_command();
	spi_command();
	spi_command();
	spi_command();
	
	// Test that after sending a few random bits, re-selecting the device
	// re-aligns everything
	spi_bit(1'b1, ignored_bit);
	spi_bit(1'b1, ignored_bit);
	spi_bit(1'b1, ignored_bit);
	nss_i <= 1'b1;
	#(SCLK_PERIOD*10)
	nss_i <= 1'b0;
	spi_command();
	spi_command();
	spi_command();
	spi_command();
	
	// Test that the device ignores everything again
	nss_i <= 1'b1;
	spi_command();
	spi_command();
	spi_command();
	spi_command();
	
	#(SCLK_PERIOD*2)
	$display("%0d: End of stimulus, stopping simulation.", $time);
	$finish();
	end


////////////////////////////////////////////////////////////////////////////////
// System clock generation
////////////////////////////////////////////////////////////////////////////////

initial
	begin
		clk_i = 1'b0;
		forever #(CLK_PERIOD/2) clk_i = ~clk_i;
	end



endmodule
