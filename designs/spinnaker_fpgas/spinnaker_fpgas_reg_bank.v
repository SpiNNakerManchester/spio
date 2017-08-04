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
                                 , output reg              [3:0] SCRMBL_IDL_DAT
                                   // Status LED overrides (for indicating
                                   // configuration status of the FPGA)
                                   //     { DIM_RING
                                   //     , DIM_PERIPH
                                   //     , DIM_B2B1
                                   //     , DIM_B2B0
                                   //     , FORCE_ERROR_RING
                                   //     , FORCE_ERROR_PERIPH
                                   //     , FORCE_ERROR_B2B1
                                   //     , FORCE_ERROR_B2B0
                                   //     }
                                 , output reg              [7:0] LED_OVERRIDE
                                   // rx equalization
                                   //   {RING_RXEQMIX
                                   //   , PERIPH_RXEQMIX
                                   //   , B2B_RXEQMIX
                                   //   , B2B_RXEQMIX
                                   //   };
                                 , output reg              [7:0] RXEQMIX
                                   // tx driver swing
                                   //   {RING_TXDIFFCTRL
                                   //   , PERIPH_TXDIFFCTRL
                                   //   , B2B_TXDIFFCTRL
                                   //   , B2B_TXDIFFCTRL
                                   //   };
                                 , output reg             [15:0] TXDIFFCTRL
                                   // tx pre-emphasis
                                   //   {RING_TXPREEMPHASIS
                                   //   , PERIPH_TXPREEMPHASIS
                                   //   , B2B_TXPREEMPHASIS
                                   //   , B2B_TXPREEMPHASIS
                                   //   };
                                 , output reg             [11:0] TXPREEMPHASIS
                                 );

// GTP Analog signal generation settings (either found via IBERT or left as zeros)
localparam    B2B_RXEQMIX = 2'b10;   // 5.4 dB
localparam PERIPH_RXEQMIX = 2'b00;   // Default
localparam   RING_RXEQMIX = 2'b00;   // Default

//!! localparam    B2B_TXDIFFCTRL = 4'b0010; // 495 mV
localparam    B2B_TXDIFFCTRL = 4'b0110; // 762 mV
//!! localparam    B2B_TXDIFFCTRL = 4'b0111; // 849 mV
//!! localparam    B2B_TXDIFFCTRL = 4'b1010; // 1054 mV
localparam PERIPH_TXDIFFCTRL = 4'b0000; // Default
localparam   RING_TXDIFFCTRL = 4'b0000; // Default

localparam    B2B_TXPREEMPHASIS = 3'b010;  // 1.7 dB
localparam PERIPH_TXPREEMPHASIS = 3'b000;  // Default
localparam   RING_TXPREEMPHASIS = 3'b000;  // Default

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
localparam LEDO_REG = 6; // LED link override { 7: DIM_RING
                         //                   , 6: DIM_PERIPH
                         //                   , 5: DIM_B2B1
                         //                   , 4: DIM_B2B0
                         //                   , 3: FORCE_ERROR_RING
                         //                   , 2: FORCE_ERROR_PERIPH
                         //                   , 1: FORCE_ERROR_B2B1
                         //                   , 0: FORCE_ERROR_B2B0
                         //                   }
localparam RXEQ_REG = 7; // rx equalization   { 7-6: RING_RXEQMIX
                         //                   , 5-4: PERIPH_RXEQMIX
                         //                   , 3-2: B2B1_RXEQMIX
                         //                   , 1-0: B2B0_RXEQMIX
                         //                   }
localparam TXDS_REG = 8; // tx driver swing   { 15-12: RING_TXDIFFCTRL
                         //                   ,  11-8: PERIPH_TXDIFFCTRL
                         //                   ,   7-4: B2B1_TXDIFFCTRL
                         //                   ,   3-0: B2B0_TXDIFFCTRL
                         //                   }
localparam TXPE_REG = 9; // tx pre-emphasis   { 11-9: RING_TXPREEMPHASIS
                         //                   ,  8-6: PERIPH_TXPREEMPHASIS
                         //                   ,  5-3: B2B1_TXPREEMPHASIS
                         //                   ,  2-0: B2B0_TXPREEMPHASIS
                         //                   }


// Write address decode
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		begin
			PERIPH_MC_KEY  <= 32'hFFFFFFFF;
			PERIPH_MC_MASK <= 32'h00000000;
			SCRMBL_IDL_DAT <=  4'hF;
			SPINNAKER_LINK_ENABLE <= 32'h00000000;
			LED_OVERRIDE <= 8'h0F;
			RXEQMIX <= {RING_RXEQMIX
			           , PERIPH_RXEQMIX
			           , B2B_RXEQMIX
			           , B2B_RXEQMIX
			           };
			TXDIFFCTRL <= {RING_TXDIFFCTRL
			              , PERIPH_TXDIFFCTRL
			              , B2B_TXDIFFCTRL
			              , B2B_TXDIFFCTRL
			              };
			TXPREEMPHASIS <= {RING_TXPREEMPHASIS
			                 , PERIPH_TXPREEMPHASIS
			                 , B2B_TXPREEMPHASIS
			                 , B2B_TXPREEMPHASIS
			                 };
		end
	else
		if (WRITE_IN)
			case (ADDR_IN)
				PKEY_REG: PERIPH_MC_KEY  <= WRITE_DATA_IN;
				PMSK_REG: PERIPH_MC_MASK <= WRITE_DATA_IN;
				SCRM_REG: SCRMBL_IDL_DAT <= WRITE_DATA_IN;
				SLEN_REG: SPINNAKER_LINK_ENABLE <= WRITE_DATA_IN;
				LEDO_REG: LED_OVERRIDE <= WRITE_DATA_IN;
				RXEQ_REG: RXEQMIX <= WRITE_DATA_IN;
				TXDS_REG: TXDIFFCTRL <= WRITE_DATA_IN;
				TXPE_REG: TXPREEMPHASIS <= WRITE_DATA_IN;
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
		LEDO_REG: READ_DATA_OUT = LED_OVERRIDE;
		RXEQ_REG: READ_DATA_OUT = RXEQMIX;
		TXDS_REG: READ_DATA_OUT = TXDIFFCTRL;
		TXPE_REG: READ_DATA_OUT = TXPREEMPHASIS;
		default:  READ_DATA_OUT = {REGD_BITS{1'b1}};
	endcase


endmodule
