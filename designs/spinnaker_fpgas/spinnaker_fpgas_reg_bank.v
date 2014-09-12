/**
 * Top-level control/diagnostic registers.
 */


module spinnaker_fpgas_reg_bank #( // Address bits
                                   parameter REGA_BITS = 14
                                   // Data bits
                                 , parameter REGD_BITS = 32
                                 )
                                 ( input  wire CLK_IN
                                 , input  wire RESET_IN
                                   // Register bank interface
                                 ,   input  wire                 WRITE_IN
                                 ,   input  wire [REGA_BITS-1:0] ADDR_IN
                                 ,   input  wire [REGD_BITS-1:0] WRITE_DATA_IN
                                 ,   output reg  [REGD_BITS-1:0] READ_DATA_OUT
                                   // Version
                                 , input  wire [REGD_BITS-1:0] VERSION_IN
                                   // Compilation flags { FPGAID
                                   //                   , NORTH_SOUTH_ON_FRONT
                                   //                   , DEBUG_CHIPSCOPE_VIO
                                   //                   }
                                 , input  wire [3:0]           FLAGS_IN
                                   // Peripheral routing key/mask
                                 , output reg  [31:0]          PERIPH_MC_KEY
                                 , output reg  [31:0]          PERIPH_MC_MASK
                                 );

localparam VERS_REG = 0; // Top level design version
localparam FLAG_REG = 1; // Compile flags { 3-2: FPGA ID
                         //               ,   1: north/south on front
                         //               ,   0: chip scope
                         //               }
localparam PKEY_REG = 2; // Peripheral MC route key
localparam PMSK_REG = 3; // Peripheral MC route mask


// Write address decode
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		begin
			PERIPH_MC_KEY  <= 32'hFFFFFFFF;
			PERIPH_MC_MASK <= 32'h00000000;
		end
	else
		if (WRITE_IN)
			case (ADDR_IN)
				PKEY_REG: PERIPH_MC_KEY  <= WRITE_DATA_IN;
				PMSK_REG: PERIPH_MC_MASK <= WRITE_DATA_IN;
			endcase


// Read address decode
always @ (*)
	case (ADDR_IN)
		VERS_REG: READ_DATA_OUT = VERSION_IN;
		FLAG_REG: READ_DATA_OUT = FLAGS_IN;
		PKEY_REG: READ_DATA_OUT = PERIPH_MC_KEY;
		PMSK_REG: READ_DATA_OUT = PERIPH_MC_MASK;
		default:  READ_DATA_OUT = {REGD_BITS{1'b1}};
	endcase


endmodule
