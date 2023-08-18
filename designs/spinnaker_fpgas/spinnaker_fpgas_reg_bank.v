/**
 * Top-level control/diagnostic registers.
 */


module spinnaker_fpgas_reg_bank #( // Address bits
                                   parameter REGA_BITS = 14
                                   // Data bits
                                 , parameter REGD_BITS = 32
                                   // extra peripheral routing registers
                                 , parameter XR        = 6
                                 )
                                 ( input  wire                   CLK_IN
                                 , input  wire                   RESET_IN
                                   // Register bank SPI interface
                                 , input  wire                   WRITE_IN
                                 , input  wire   [REGA_BITS-1:0] ADDR_IN
                                 , input  wire   [REGD_BITS-1:0] WRITE_DATA_IN
                                 , output reg    [REGD_BITS-1:0] READ_DATA_OUT
                                   // Register bank local configuration  interface
                                 , input  wire                   LC_WRITE_IN
                                 , input  wire    [LCA_BITS-1:0] LC_ADDR_IN
                                 , input  wire   [REGD_BITS-1:0] LC_WR_DATA_IN
                                   // Version
                                 , input  wire   [REGD_BITS-1:0] VERSION_IN
                                   // Compilation flags { INCLUDE_DEBUG_CHIPSCOPE_VIO
                                   //                   , INCLUDE_PERIPH_SUPPORT
                                   //                   , INCLUDE_RING_SUPPORT
                                   //                   , NORTH_SOUTH_ON_FRONT
                                   //                   , FPGAID
                                   //                   }
                                 , input  wire             [5:0] FLAGS_IN
                                   // 2-of-7 Link enable signals
                                 , output reg             [31:0] SPINNAKER_LINK_ENABLE
                                   // Peripheral routing key/mask
                                 , output wire   [REGD_BITS-1:0] PERIPH_MC_KEY
                                 , output wire   [REGD_BITS-1:0] PERIPH_MC_MASK
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
                                   // peripheral input enable
                                 , output reg                    PERIPH_IN_EN
                                   // local configuration routing key/mask
                                 , output reg    [REGD_BITS-1:0] LC_KEY
                                 , output reg    [REGD_BITS-1:0] LC_MASK
                                   // remote configuration routing key/mask
                                 , output reg    [REGD_BITS-1:0] RC_KEY
                                 , output reg    [REGD_BITS-1:0] RC_MASK
                                   // extra peripheral routing keys/masks
                                 , output reg [XR*REGD_BITS-1:0] XPER_MC_KEY
                                 , output reg [XR*REGD_BITS-1:0] XPER_MC_MASK
                                 );

// local configuration address bits
parameter LCA_BITS = 8;

////////////////////////////////////////////////////////////////////////////////
// extra peripheral routing key/mask registers
//NOTE: XR must be a power of 2!
////////////////////////////////////////////////////////////////////////////////
parameter  XR_SHF = $clog2 (REGD_BITS);  //NOTE: Xilinx complains about $clog2
localparam XR_MSK = 32'h0000_000f;       //NOTE: room for 16 XREGS
localparam XR_ADM = ~XR_MSK;

// default register values
localparam PKEY_DEF   = 32'hffff_ffff;  // peripheral routing key
localparam PMSK_DEF   = 32'h0000_0000;  // peripheral routing mask
localparam SCRMBL_DEF =  4'hf;          // idle frame scrambler
localparam LINKEN_DEF = 32'h0000_0000;  // SpiNNaker link enables
localparam LEDOVR_DEF =  8'h0f;         // LED overrides

// peripheral configuration registers
localparam PINE_DEF   = 1'b0;           // peripheral input enable
localparam LKEY_DEF   = 32'hffff_fe00;  // local configuration routing key
localparam LMSK_DEF   = 32'hffff_ff00;  // local configuration routing mask
localparam RKEY_DEF   = 32'hffff_ff00;  // remote configuration routing key
localparam RMSK_DEF   = 32'hffff_ff00;  // remote configuration routing mask

localparam XKEY_DEF   = {XR {PKEY_DEF}};  // extra peripheral routing keys
localparam XMSK_DEF   = {XR {PMSK_DEF}};  // extra peripheral routing masks

// GTP Analog signal generation settings (either found via IBERT or left as zeros)
localparam    B2B_RXEQMIX = 2'b10;   // 5.4 dB
localparam PERIPH_RXEQMIX = 2'b10;   // 5.4 dB
localparam   RING_RXEQMIX = 2'b00;   // Default

//!! localparam    B2B_TXDIFFCTRL = 4'b0010; // 495 mV
localparam    B2B_TXDIFFCTRL = 4'b0110; // 762 mV
//!! localparam    B2B_TXDIFFCTRL = 4'b0111; // 849 mV
//!! localparam    B2B_TXDIFFCTRL = 4'b1010; // 1054 mV
localparam PERIPH_TXDIFFCTRL = 4'b0110; // 762 mV
localparam   RING_TXDIFFCTRL = 4'b0000; // Default

localparam    B2B_TXPREEMPHASIS = 3'b010;  // 1.7 dB
localparam PERIPH_TXPREEMPHASIS = 3'b010;  // 1.7 dB
localparam   RING_TXPREEMPHASIS = 3'b000;  // Default


// register addresses
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

// peripheral configuration registers
localparam LKEY_REG = 12;  // local configuration route key
localparam LMSK_REG = 13;  // local configuration route mask
localparam RKEY_REG = 14;  // local configuration route key
localparam RMSK_REG = 15;  // local configuration route mask
localparam PIND_REG = 16;  // peripheral input disable
localparam PINE_REG = 17;  // peripheral input enable

localparam XKEY_REG = 32;  // extra peripheral MC route keys
localparam XMSK_REG = 48;  // extra peripheral MC route masks

// LC_KEY and LC_MASK must be updated atomically to avoid
// configuration errors. Changes to LC_MASK will not take
// effect until a subsequent change to LC_KEY takes effect
//NOTE: use temp register to store the new LC_MASK value
reg [31:0] LC_MASK_TMP;
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		LC_MASK <= LMSK_DEF;
	else
		if ((LC_WRITE_IN && (LC_ADDR_IN == LKEY_REG))
			|| (WRITE_IN && (ADDR_IN == LKEY_REG)))
			LC_MASK <= LC_MASK_TMP;


// Write address decode for non-routing registers
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		begin
			SCRMBL_IDL_DAT <= SCRMBL_DEF;
			SPINNAKER_LINK_ENABLE <= LINKEN_DEF;
			LED_OVERRIDE <= LEDOVR_DEF;
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
				SCRM_REG: SCRMBL_IDL_DAT <= WRITE_DATA_IN;
				SLEN_REG: SPINNAKER_LINK_ENABLE <= WRITE_DATA_IN;
				LEDO_REG: LED_OVERRIDE <= WRITE_DATA_IN;
				RXEQ_REG: RXEQMIX <= WRITE_DATA_IN;
				TXDS_REG: TXDIFFCTRL <= WRITE_DATA_IN;
				TXPE_REG: TXPREEMPHASIS <= WRITE_DATA_IN;
			endcase

// PERIPH_MC_KEY and PERIPH_MC_MASK are now aliases of XR[0]
assign PERIPH_MC_KEY  = XPER_MC_KEY[0 +: REGD_BITS];
assign PERIPH_MC_MASK = XPER_MC_MASK[0 +: REGD_BITS];

// Write address decode for routing registers
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		begin
			XPER_MC_KEY    <= XKEY_DEF;
			XPER_MC_MASK   <= XMSK_DEF;
			LC_KEY         <= LKEY_DEF;
			LC_MASK_TMP    <= LMSK_DEF;
			RC_KEY         <= RKEY_DEF;
			RC_MASK        <= RMSK_DEF;
			PERIPH_IN_EN   <= PINE_DEF;
		end
	else
		begin
			// local configuration writes
			//NOTE: LC_KEY and LC_MASK must be updated atomically!
			if (LC_WRITE_IN)
				begin
					if ((LC_ADDR_IN & XR_ADM) == XKEY_REG)
						XPER_MC_KEY[((LC_ADDR_IN & XR_MSK) << XR_SHF) +: REGD_BITS]  <= LC_WR_DATA_IN;

					if ((LC_ADDR_IN & XR_ADM) == XMSK_REG)
						XPER_MC_MASK[((LC_ADDR_IN & XR_MSK) << XR_SHF) +: REGD_BITS] <= LC_WR_DATA_IN;

					case (LC_ADDR_IN)
						PKEY_REG: XPER_MC_KEY[0 +: REGD_BITS]  <= LC_WR_DATA_IN;
						PMSK_REG: XPER_MC_MASK[0 +: REGD_BITS] <= LC_WR_DATA_IN;
						LKEY_REG: LC_KEY                       <= LC_WR_DATA_IN;
						LMSK_REG: LC_MASK_TMP                  <= LC_WR_DATA_IN;
						RKEY_REG: RC_KEY                       <= LC_WR_DATA_IN;
						RMSK_REG: RC_MASK                      <= LC_WR_DATA_IN;
						PIND_REG: PERIPH_IN_EN                 <= 1'b0;
						PINE_REG: PERIPH_IN_EN                 <= 1'b1;
					endcase
				end

			// SPI writes
			//NOTE: prioritise local configuration writes
			if (WRITE_IN && (!LC_WRITE_IN || (ADDR_IN != LC_ADDR_IN)))
				begin
					if ((ADDR_IN & XR_ADM) == XKEY_REG)
						XPER_MC_KEY[((ADDR_IN & XR_MSK) << XR_SHF) +: REGD_BITS]  <= WRITE_DATA_IN;

					if ((ADDR_IN & XR_ADM) == XMSK_REG)
						XPER_MC_MASK[((ADDR_IN & XR_MSK) << XR_SHF) +: REGD_BITS] <= WRITE_DATA_IN;

					case (ADDR_IN)
						PKEY_REG: XPER_MC_KEY[0 +: REGD_BITS]  <= WRITE_DATA_IN;
						PMSK_REG: XPER_MC_MASK[0 +: REGD_BITS] <= WRITE_DATA_IN;
						LKEY_REG: LC_KEY                       <= WRITE_DATA_IN;
						LMSK_REG: LC_MASK_TMP                  <= WRITE_DATA_IN;
						RKEY_REG: RC_KEY                       <= WRITE_DATA_IN;
						RMSK_REG: RC_MASK                      <= WRITE_DATA_IN;
						PIND_REG: PERIPH_IN_EN                 <= 1'b0;
						PINE_REG: PERIPH_IN_EN                 <= 1'b1;
					endcase
				end
		end

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
		LKEY_REG: READ_DATA_OUT = LC_KEY;
		LMSK_REG: READ_DATA_OUT = LC_MASK;
		RKEY_REG: READ_DATA_OUT = RC_KEY;
		RMSK_REG: READ_DATA_OUT = RC_MASK;
		PIND_REG: READ_DATA_OUT = PERIPH_IN_EN;
		PINE_REG: READ_DATA_OUT = PERIPH_IN_EN;
		default:  READ_DATA_OUT = {REGD_BITS{1'b1}};
	endcase


endmodule
