/**
 * Address decoder for SPI peek/pokes for each HSS block. Top two bits are used
 * to determine which device is selected:
 *
 * 2'b00: Board-to-Board 0
 * 2'b01: Board-to-Board 1
 * 2'b10: Peripheral Connection
 * 2'b11: Ring Network
 *
 * Combinatorial.
 */

module spinnaker_fpgas_spi_address_decode #( // Number of address bits from SPI. Top
                                             // two bits are used to select the device.
                                             parameter SPI_ADDR_BITS = 32
                                             // Number of bits in a word
                                           , parameter VAL_BITS = 32
                                           )
                                           ( // Raw address from SPI
                                             input wire [SPI_ADDR_BITS-1:0] SPI_ADDR_IN
                                             // Read strobe from SPI
                                           , input wire SPI_READ_IN
                                             // Write strobe from SPI
                                           , input wire SPI_WRITE_IN
                                             // Read value to return via SPI
                                           , output reg [VAL_BITS-1:0] SPI_READ_VALUE_OUT
                                             // Address for intended device (missing
                                             // the bottom two bits)
                                           , output wire [15:2] ADDR_OUT
                                             // Per-device signals
                                               // Read strobes
                                           ,   output wire [1:0]    B2B_READ_OUT
                                           ,   output wire       PERIPH_READ_OUT
                                           ,   output wire         RING_READ_OUT
                                           ,   output wire          TOP_READ_OUT
                                               // Write strobes
                                           ,   output wire [1:0]    B2B_WRITE_OUT
                                           ,   output wire       PERIPH_WRITE_OUT
                                           ,   output wire         RING_WRITE_OUT
                                           ,   output wire          TOP_WRITE_OUT
                                               // Data being read back
                                           ,   input  wire [(2*VAL_BITS)-1:0]    B2B_READ_VALUE_IN
                                           ,   input  wire [VAL_BITS-1:0]     PERIPH_READ_VALUE_IN
                                           ,   input  wire [VAL_BITS-1:0]       RING_READ_VALUE_IN
                                           ,   input  wire [VAL_BITS-1:0]        TOP_READ_VALUE_IN
                                           );
// Device addresses
localparam   B2B0_ADDR = 16'h0000;
localparam   B2B1_ADDR = 16'h0001;
localparam PERIPH_ADDR = 16'h0002;
localparam   RING_ADDR = 16'h0003;
localparam    TOP_ADDR = 16'h0004;

assign ADDR_OUT = SPI_ADDR_IN[15:2];

// Address decode
wire [1:0]    b2b_sel = { SPI_ADDR_IN[SPI_ADDR_BITS-1:16] == B2B1_ADDR
                        , SPI_ADDR_IN[SPI_ADDR_BITS-1:16] == B2B0_ADDR
                        };
wire periph_sel = SPI_ADDR_IN[SPI_ADDR_BITS-1:16] == PERIPH_ADDR;
wire ring_sel   = SPI_ADDR_IN[SPI_ADDR_BITS-1:16] == RING_ADDR;
wire top_sel    = SPI_ADDR_IN[SPI_ADDR_BITS-1:16] == TOP_ADDR;

// Read strobes
assign B2B_READ_OUT = { SPI_READ_IN && b2b_sel[1]
                      , SPI_READ_IN && b2b_sel[0]
                      };
assign PERIPH_READ_OUT = SPI_READ_IN && periph_sel;
assign   RING_READ_OUT = SPI_READ_IN && ring_sel;
assign    TOP_READ_OUT = SPI_READ_IN && top_sel;

// Write strobes
assign B2B_WRITE_OUT = { SPI_WRITE_IN && b2b_sel[1]
                       , SPI_WRITE_IN && b2b_sel[0]
                       };
assign PERIPH_WRITE_OUT = SPI_WRITE_IN && periph_sel;
assign   RING_WRITE_OUT = SPI_WRITE_IN && ring_sel;
assign    TOP_WRITE_OUT = SPI_WRITE_IN && top_sel;

// Value multiplexer
always @ (*)
	case (SPI_ADDR_IN[SPI_ADDR_BITS-1:16])
		  B2B0_ADDR: SPI_READ_VALUE_OUT =    B2B_READ_VALUE_IN[0*VAL_BITS+:VAL_BITS];
		  B2B1_ADDR: SPI_READ_VALUE_OUT =    B2B_READ_VALUE_IN[1*VAL_BITS+:VAL_BITS];
		PERIPH_ADDR: SPI_READ_VALUE_OUT = PERIPH_READ_VALUE_IN;
		  RING_ADDR: SPI_READ_VALUE_OUT =   RING_READ_VALUE_IN;
		   TOP_ADDR: SPI_READ_VALUE_OUT =    TOP_READ_VALUE_IN;
		    default: SPI_READ_VALUE_OUT = {VAL_BITS{1'bX}};
	endcase

endmodule
