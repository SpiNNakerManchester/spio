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
                                 , input  wire   [REGD_BITS-1:0] VERSION_IN
                                   // Compilation flags { DEBUG_CHIPSCOPE_VIO
                                   //                   , INCLUDE_PERIPH_SUPPORT
                                   //                   , INCLUDE_RING_SUPPORT
                                   //                   , NORTH_SOUTH_ON_FRONT
                                   //                   , FPGAID
                                   //                   }
                                 , input  wire             [5:0] FLAGS_IN
                                   // 2-of-7 Link enable signals
                                 , output reg             [31:0] SPINNAKER_LINK_ENABLE
                                   // Peripheral routing key/mask
                                 , output reg             [31:0] PERIPH_MC_KEY
                                 , output reg             [31:0] PERIPH_MC_MASK
                                 , output reg             [31:0] SCRMBL_IDL_DAT
                                 );

localparam VERS_REG = 0; // Top level design version
localparam FLAG_REG = 1; // Compile flags {   5: chip scope
                         //               ,   4: peripheral support
                         //               ,   3: ring support
                         //               ,   2: north/south on front
                         //               , 1-0: FPGA ID
                         //               }
localparam PKEY_REG = 2; // Peripheral MC route key
localparam PMSK_REG = 3; // Peripheral MC route mask
localparam SCRM_REG = 4; // idle data scrambling control
localparam SLEN_REG = 5; // SpiNNaker (2-of-7) link enable


// Write address decode
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		begin
			PERIPH_MC_KEY  <= 32'hFFFFFFFF;
			PERIPH_MC_MASK <= 32'h00000000;
			SCRMBL_IDL_DAT <= 32'hFFFFFFFF;
			SPINNAKER_LINK_ENABLE <= 32'hFFFFFFFF;
		end
	else
		if (WRITE_IN)
			case (ADDR_IN)
				PKEY_REG: PERIPH_MC_KEY  <= WRITE_DATA_IN;
				PMSK_REG: PERIPH_MC_MASK <= WRITE_DATA_IN;
				SCRM_REG: SCRMBL_IDL_DAT <= WRITE_DATA_IN;
				SLEN_REG: SPINNAKER_LINK_ENABLE <= WRITE_DATA_IN;
			endcase


// Read address decode
always @ (*)
	case (ADDR_IN)
		VERS_REG: READ_DATA_OUT = VERSION_IN;
		FLAG_REG: READ_DATA_OUT = FLAGS_IN;
		PKEY_REG: READ_DATA_OUT = PERIPH_MC_KEY;
		PMSK_REG: READ_DATA_OUT = PERIPH_MC_MASK;
		SCRM_REG: READ_DATA_OUT = SCRMBL_IDL_DAT;
		SLEN_REG: READ_DATA_OUT = SPINNAKER_LINK_ENABLE;
		default:  READ_DATA_OUT = {REGD_BITS{1'b1}};
	endcase


endmodule
