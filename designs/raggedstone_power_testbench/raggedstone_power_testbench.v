/**
 * Raggedstone FPGA design for power management experiments on a Spartan 6's GTP
 * tile.
 */

module raggedstone_power_testbench #( // Speed up simulated reset of GTP tile
                                      parameter SIM_GTPRESET_SPEEDUP = 0
                                    )
                                    ( // FPGA active-low buttons {SW2, SW1}
                                      input wire [1:0] NBUTTON_IN
                                      
                                      // Status LEDs
                                    , output wire [5:2]  LEDS_OUT
                                    , output wire [13:0] SEVEN_SEG_OUT
                                      
                                      // Differential 150 MHz clock source
                                    , input wire REFCLK_PAD_P_IN
                                    , input wire REFCLK_PAD_N_IN
                                    
                                      // Wires for S-ATA ports
                                    , input  wire SATA1_RXN_IN
                                    , input  wire SATA1_RXP_IN
                                    , output wire SATA1_TXN_OUT
                                    , output wire SATA1_TXP_OUT
                                    
                                    , input  wire SATA2_RXN_IN
                                    , input  wire SATA2_RXP_IN
                                    , output wire SATA2_TXN_OUT
                                    , output wire SATA2_TXP_OUT
                                    
                                    , input  wire SATA3_RXN_IN
                                    , input  wire SATA3_RXP_IN
                                    , output wire SATA3_TXN_OUT
                                    , output wire SATA3_TXP_OUT
                                    );

genvar i;

// GTP internal loopback connectivity (for debugging)
localparam GTP_LOOPBACK = 3'b000;

// GTP Analog signal generation settings (either found via IBERT or left as zeros)
localparam RXEQMIX       =   2'b00; // Default
localparam TXDIFFCTRL    = 4'b0001; // Slightly higher, noted as producing less
                                    // bit errors
localparam TXPREEMPHASIS =  3'b000; // Default

// Number of bits in blinker counter
localparam BLINK_BITS = 22;

////////////////////////////////////////////////////////////////////////////////
// Internal signals
////////////////////////////////////////////////////////////////////////////////

// LED blinker state
reg [BLINK_BITS-1:0] blink_count_i;

// LED signals
wire [5:2]  leds_i;
wire [13:0] seven_seg_i;
reg         seven_seg_en_i;

// 7-seg segments {DP G F E D C B A}, active high
wire [6:0] seven_seg_segments_i;

// 7-seg LED digit enables, active high
wire [3:0] seven_seg_digits_i;

// Button signals (including an active-high version)
wire [1:0] nbutton_i;
wire [1:0] button_i;
reg  [1:0] button_sync_i [1:0];

// Input clock to the GTP modules from the external clock
wire gtpclkin_i;

// The GTP's external clock signal, exposed for general use
wire [1:0] unbuffered_gtpclkout_i;
wire gtpclkout_i;

// GTP tile PLL stability signal
wire plllkdet_i;

// User clocks, all of these are positive-edge aligned.
wire usrclk_i;
wire usrclk2_i;

// Are the user clocks stable?
wire usrclks_stable_i;

// Has a bad value been received by the PRBS checker?
wire prbserr_i;

// Has the PRBS checker has encountered an error
reg sticky_prbserr_i;

// A blinker at some human-visible frequency
wire blink_i;

// Power down mode cycle counter
reg [2:0] pd_cycle_i;

// PLL powerdown signal
reg pll_pd_i;

// TX/RX block powerdown signal
reg rxtx_pd_i;

// Is the receiver data valid
wire rxvalid_i;

// A timer between power-down state changes and rxvalid_i
reg [15:0] valid_latency_i;


////////////////////////////////////////////////////////////////////////////////
// Buttons
////////////////////////////////////////////////////////////////////////////////

// Buffer the button signals
IBUF nbuttons_buf_i[1:0] (.I(NBUTTON_IN), .O(nbutton_i));

// Convert to active high
assign button_i = ~nbutton_i;

// Synchronise
always @ (posedge usrclk2_i)
	begin
		button_sync_i[0] <= {button_sync_i[0][0], button_i[0]};
		button_sync_i[1] <= {button_sync_i[1][0], button_i[1]};
	end


////////////////////////////////////////////////////////////////////////////////
// LEDs
////////////////////////////////////////////////////////////////////////////////

// Buffer the LEDs
OBUF leds_buf_i [5:2] (.I(leds_i[5:2]), .O(LEDS_OUT[5:2]));

// Just set the LEDs in some arbitrary state
assign leds_i[5:2] = {
	sticky_prbserr_i,
	rxtx_pd_i,
	pll_pd_i,
	blink_i
};


// Buffer the 7-seg display pins
OBUF seven_seg_buf_i [13:0] (.I(seven_seg_i & {14{seven_seg_en_i}}), .O(SEVEN_SEG_OUT));


// Assign the pins accordingly
assign seven_seg_i = {
	~seven_seg_segments_i[5], // Pin 11 F
	1'b0,                     // Pin 12 (Not-connected)
	~seven_seg_segments_i[2], // Pin 13 C
	~seven_seg_segments_i[0], // Pin 14 A
	~seven_seg_segments_i[6], // Pin 15 G
	~seven_seg_segments_i[1], // Pin 16 B
	seven_seg_digits_i[0],    // Pin  8 DIGIT 4
	1'b1,                     // Pin  7 DP (unused)
	seven_seg_digits_i[1],    // Pin  6 DIGIT 3
	~seven_seg_segments_i[4], // Pin  5 E
	1'b0,                     // Pin  4 Unused (Colon + Apostrophe)
	~seven_seg_segments_i[3], // Pin  3 D
	seven_seg_digits_i[2],    // Pin  2 DIGIT 2
	seven_seg_digits_i[3]     // Pin  1 DIGIT 1
};


////////////////////////////////////////////////////////////////////////////////
// Seven Segment Display Driver
////////////////////////////////////////////////////////////////////////////////

seven_segment_scanner #( .NUM_DIGITS(4)
                       , .CLK_DIV_BITS(14)
                       )
seven_segment_scanner_i( .CLK_IN(usrclk2_i)
                       , .RESET_IN(1'b0)
                       , .VALUE_IN(valid_latency_i)
                       , .DISPLAY_OUT(seven_seg_segments_i)
                       , .DIGIT_ENABLE_OUT(seven_seg_digits_i)
                       );



////////////////////////////////////////////////////////////////////////////////
// Blinker
////////////////////////////////////////////////////////////////////////////////

initial
	blink_count_i = 0;
always @(posedge usrclk2_i)
	blink_count_i <= blink_count_i + 1;
assign blink_i = blink_count_i[BLINK_BITS-1];



////////////////////////////////////////////////////////////////////////////////
// Sticky PRBS bit
////////////////////////////////////////////////////////////////////////////////

initial
	sticky_prbserr_i = 1'b0;
always @(posedge usrclk2_i)
	if (button_sync_i[0][1])
		sticky_prbserr_i <= 1'b0;
	else if (prbserr_i)
		sticky_prbserr_i <= 1'b1;



////////////////////////////////////////////////////////////////////////////////
// RX valid timer
////////////////////////////////////////////////////////////////////////////////

initial
	begin
		valid_latency_i = 16'b0;
		seven_seg_en_i = 1'b0;
	end

always @(posedge usrclk_i)
	begin
		// Increment counter if not valid
		if (!rxvalid_i && valid_latency_i != 16'hFFFF)
			valid_latency_i <= valid_latency_i + 1;
		
		// Reset counter on mode change
		if (button_sync_i[1][1] && blink_count_i == 0)
			begin
				valid_latency_i <= 1'b0;
				seven_seg_en_i <= 1'b1;
			end
		
		// Disable display on clear PRBS flag clear
		if (button_sync_i[0][1])
			seven_seg_en_i <= 1'b0;
	end


////////////////////////////////////////////////////////////////////////////////
// Power-down mode selection
////////////////////////////////////////////////////////////////////////////////

initial
	begin
		pd_cycle_i = 3'd0;
		pll_pd_i   = 1'b0;
		rxtx_pd_i  = 1'b0;
	end

// Advance every time the blink occurs with SW2 held down
always @(posedge usrclk2_i)
	if (button_sync_i[1][1] && blink_count_i == 0)
		begin
			case (pd_cycle_i)
				3'd0:
					begin
						pd_cycle_i = 3'd1;
						pll_pd_i  = 1'b0;
						rxtx_pd_i = 1'b1;
					end
				
				3'd1:
					begin
						pd_cycle_i = 3'd2;
						pll_pd_i  = 1'b0;
						rxtx_pd_i = 1'b0;
					end
				
				3'd2:
					begin
						pd_cycle_i = 3'd3;
						pll_pd_i  = 1'b1;
						rxtx_pd_i = 1'b0;
					end
				
				3'd3:
					begin
						pd_cycle_i = 3'd4;
						pll_pd_i  = 1'b0;
						rxtx_pd_i = 1'b0;
					end
				
				3'd4:
					begin
						pd_cycle_i = 3'd5;
						pll_pd_i  = 1'b1;
						rxtx_pd_i = 1'b1;
					end
				
				3'd5:
					begin
						pd_cycle_i = 3'd0;
						pll_pd_i  = 1'b0;
						rxtx_pd_i = 1'b0;
					end
				
				default:
					begin
						pd_cycle_i = 3'd0;
						pll_pd_i  = 1'b0;
						rxtx_pd_i = 1'b0;
					end
			endcase
		end

////////////////////////////////////////////////////////////////////////////////
// Clock generation/scaling
////////////////////////////////////////////////////////////////////////////////

// External differential clock signal for GTP tile (150 MHz)
IBUFDS refclk_ibufds_i ( .O (gtpclkin_i)
                       , .I (REFCLK_PAD_P_IN)
                       , .IB(REFCLK_PAD_N_IN)
                       );

// Buffer the copy of the transceiver's external clock
BUFIO2 # (.DIVIDE (1), .DIVIDE_BYPASS ("TRUE"))
gtpclkout_0_pll_bufio2_i ( .I            (unbuffered_gtpclkout_i[0])
                         , .DIVCLK       (gtpclkout_i)
                         , .IOCLK        () // Unused
                         , .SERDESSTROBE () // Unused
                         );

// Multiply/divide the transceiver's external clock to get the two user clocks
// for each link type
wire clk_150_i;
wire clk_75_i;
wire clk_37_5_i;
clock_scaler
clock_scaler_i ( .CLK_IN1  (gtpclkout_i) // 150  Mhz (Input)
               , .CLK_OUT1 (clk_150_i)   // 150  MHz (Output)
               , .CLK_OUT2 (clk_75_i)    // 75   MHz (Output)
               , .CLK_OUT3 (clk_37_5_i)  // 37.5 MHz (Output)
               , .RESET    (!plllkdet_i)
               , .LOCKED   (usrclks_stable_i)
               );

assign usrclk_i   = clk_150_i;
assign usrclk2_i  = clk_37_5_i;
;


////////////////////////////////////////////////////////////////////////////////
// GTP Block
////////////////////////////////////////////////////////////////////////////////

GTPA1_DUAL #(//_______________________ Simulation-Only Attributes __________________
             .SIM_TX_ELEC_IDLE_LEVEL         ("Z"),
             .SIM_RECEIVER_DETECT_PASS       ("TRUE"),
             .SIM_VERSION                    ("2.0"),
            
             .SIM_REFCLK0_SOURCE             (3'b000),
             .SIM_REFCLK1_SOURCE             (3'b000),
            
             .SIM_GTPRESET_SPEEDUP           (SIM_GTPRESET_SPEEDUP),
             .CLK25_DIVIDER_0                (6), // 150 MHz / 6
             .CLK25_DIVIDER_1                (6), // 150 MHz / 6
             .PLL_DIVSEL_FB_0                (2),
             .PLL_DIVSEL_FB_1                (2),
             .PLL_DIVSEL_REF_0               (1),
             .PLL_DIVSEL_REF_1               (1),
            
             // PLL Attributes
             .CLKINDC_B_0                            ("TRUE"),
             .CLKRCV_TRST_0                          ("TRUE"),
             .OOB_CLK_DIVIDER_0                      (6),
             .PLL_COM_CFG_0                          (24'h21680a),
             .PLL_CP_CFG_0                           (8'h00),
             .PLL_RXDIVSEL_OUT_0                     (2),
             .PLL_SATA_0                             ("FALSE"),
             .PLL_SOURCE_0                           ("PLL0"),
             .PLL_TXDIVSEL_OUT_0                     (2),
             .PLLLKDET_CFG_0                         (3'b111),
            
            //
             .CLKINDC_B_1                            ("TRUE"),
             .CLKRCV_TRST_1                          ("TRUE"),
             .OOB_CLK_DIVIDER_1                      (6),
             .PLL_COM_CFG_1                          (24'h21680a),
             .PLL_CP_CFG_1                           (8'h00),
             .PLL_RXDIVSEL_OUT_1                     (2),
             .PLL_SATA_1                             ("FALSE"),
             .PLL_SOURCE_1                           ("PLL0"),
             .PLL_TXDIVSEL_OUT_1                     (2),
             .PLLLKDET_CFG_1                         (3'b111),
             .PMA_COM_CFG_EAST                       (36'h000008000),
             .PMA_COM_CFG_WEST                       (36'h00000a000),
             .TST_ATTR_0                             (32'h00000000),
             .TST_ATTR_1                             (32'h00000000),
            
            //TX Interface Attributes
             .CLK_OUT_GTP_SEL_0                      ("REFCLKPLL0"),
             .TX_TDCC_CFG_0                          (2'b00),
             .CLK_OUT_GTP_SEL_1                      ("REFCLKPLL1"),
             .TX_TDCC_CFG_1                          (2'b00),
            
            //TX Buffer and Phase Alignment Attributes
             .PMA_TX_CFG_0                           (20'h00082),
             .TX_BUFFER_USE_0                        ("TRUE"),
             .TX_XCLK_SEL_0                          ("TXOUT"),
             .TXRX_INVERT_0                          (3'b011),
             .PMA_TX_CFG_1                           (20'h00082),
             .TX_BUFFER_USE_1                        ("TRUE"),
             .TX_XCLK_SEL_1                          ("TXOUT"),
             .TXRX_INVERT_1                          (3'b011),
            
            //TX Driver and OOB signalling Attributes
             .CM_TRIM_0                              (2'b00),
             .TX_IDLE_DELAY_0                        (3'b011),
             .CM_TRIM_1                              (2'b00),
             .TX_IDLE_DELAY_1                        (3'b011),
            
            //TX PIPE/SATA Attributes
             .COM_BURST_VAL_0                        (4'b1111),
             .COM_BURST_VAL_1                        (4'b1111),
            
            //RX Driver,OOB signalling,Coupling and Eq,CDR Attributes
             .AC_CAP_DIS_0                           ("TRUE"),
             .OOBDETECT_THRESHOLD_0                  (3'b110),
             .PMA_CDR_SCAN_0                         (27'h6404040),
             .PMA_RX_CFG_0                           (25'h05ce049),
             .PMA_RXSYNC_CFG_0                       (7'h00),
             .RCV_TERM_GND_0                         ("FALSE"),
             .RCV_TERM_VTTRX_0                       ("TRUE"),
             .RXEQ_CFG_0                             (8'b01111011),
             .TERMINATION_CTRL_0                     (5'b10100),
             .TERMINATION_OVRD_0                     ("FALSE"),
             .TX_DETECT_RX_CFG_0                     (14'h1832),
             .AC_CAP_DIS_1                           ("TRUE"),
             .OOBDETECT_THRESHOLD_1                  (3'b110),
             .PMA_CDR_SCAN_1                         (27'h6404040),
             .PMA_RX_CFG_1                           (25'h05ce049),
             .PMA_RXSYNC_CFG_1                       (7'h00),
             .RCV_TERM_GND_1                         ("FALSE"),
             .RCV_TERM_VTTRX_1                       ("TRUE"),
             .RXEQ_CFG_1                             (8'b01111011),
             .TERMINATION_CTRL_1                     (5'b10100),
             .TERMINATION_OVRD_1                     ("FALSE"),
             .TX_DETECT_RX_CFG_1                     (14'h1832),
            
            //PRBS Detection Attributes
             .RXPRBSERR_LOOPBACK_0                   (1'b0),
             .RXPRBSERR_LOOPBACK_1                   (1'b0),
            
            //Comma Detection and Alignment Attributes
             .ALIGN_COMMA_WORD_0                     (1),
             .COMMA_10B_ENABLE_0                     (10'b1111111111),
             .DEC_MCOMMA_DETECT_0                    ("TRUE"),
             .DEC_PCOMMA_DETECT_0                    ("TRUE"),
             .DEC_VALID_COMMA_ONLY_0                 ("TRUE"),
             .MCOMMA_10B_VALUE_0                     (10'b1010000011),
             .MCOMMA_DETECT_0                        ("TRUE"),
             .PCOMMA_10B_VALUE_0                     (10'b0101111100),
             .PCOMMA_DETECT_0                        ("TRUE"),
             .RX_SLIDE_MODE_0                        ("PCS"),
             .ALIGN_COMMA_WORD_1                     (1),
             .COMMA_10B_ENABLE_1                     (10'b1111111111),
             .DEC_MCOMMA_DETECT_1                    ("TRUE"),
             .DEC_PCOMMA_DETECT_1                    ("TRUE"),
             .DEC_VALID_COMMA_ONLY_1                 ("TRUE"),
             .MCOMMA_10B_VALUE_1                     (10'b1010000011),
             .MCOMMA_DETECT_1                        ("TRUE"),
             .PCOMMA_10B_VALUE_1                     (10'b0101111100),
             .PCOMMA_DETECT_1                        ("TRUE"),
             .RX_SLIDE_MODE_1                        ("PCS"),
            
            //RX Loss-of-sync State Machine Attributes
             .RX_LOS_INVALID_INCR_0                  (8),
             .RX_LOS_THRESHOLD_0                     (128),
             .RX_LOSS_OF_SYNC_FSM_0                  ("TRUE"),
             .RX_LOS_INVALID_INCR_1                  (8),
             .RX_LOS_THRESHOLD_1                     (128),
             .RX_LOSS_OF_SYNC_FSM_1                  ("TRUE"),
            
            //RX Elastic Buffer and Phase alignment Attributes
             .RX_BUFFER_USE_0                        ("TRUE"),
             .RX_EN_IDLE_RESET_BUF_0                 ("TRUE"),
             .RX_IDLE_HI_CNT_0                       (4'b1000),
             .RX_IDLE_LO_CNT_0                       (4'b0000),
             .RX_XCLK_SEL_0                          ("RXREC"),
             .RX_BUFFER_USE_1                        ("TRUE"),
             .RX_EN_IDLE_RESET_BUF_1                 ("TRUE"),
             .RX_IDLE_HI_CNT_1                       (4'b1000),
             .RX_IDLE_LO_CNT_1                       (4'b0000),
             .RX_XCLK_SEL_1                          ("RXREC"),
            
            //Clock Correction Attributes
             .CLK_COR_ADJ_LEN_0                      (4),
             .CLK_COR_DET_LEN_0                      (4),
             .CLK_COR_INSERT_IDLE_FLAG_0             ("FALSE"),
             .CLK_COR_KEEP_IDLE_0                    ("FALSE"),
             .CLK_COR_MAX_LAT_0                      (18),
             .CLK_COR_MIN_LAT_0                      (16),
             .CLK_COR_PRECEDENCE_0                   ("TRUE"),
             .CLK_COR_REPEAT_WAIT_0                  (5),
             .CLK_COR_SEQ_1_1_0                      (10'b0100011100),
             .CLK_COR_SEQ_1_2_0                      (10'b0100011100),
             .CLK_COR_SEQ_1_3_0                      (10'b0100011100),
             .CLK_COR_SEQ_1_4_0                      (10'b0100011100),
             .CLK_COR_SEQ_1_ENABLE_0                 (4'b1111),
             .CLK_COR_SEQ_2_1_0                      (10'b0100000000),
             .CLK_COR_SEQ_2_2_0                      (10'b0100000000),
             .CLK_COR_SEQ_2_3_0                      (10'b0100000000),
             .CLK_COR_SEQ_2_4_0                      (10'b0100000000),
             .CLK_COR_SEQ_2_ENABLE_0                 (4'b0000),
             .CLK_COR_SEQ_2_USE_0                    ("FALSE"),
             .CLK_CORRECT_USE_0                      ("TRUE"),
             .RX_DECODE_SEQ_MATCH_0                  ("TRUE"),
             .CLK_COR_ADJ_LEN_1                      (4),
             .CLK_COR_DET_LEN_1                      (4),
             .CLK_COR_INSERT_IDLE_FLAG_1             ("FALSE"),
             .CLK_COR_KEEP_IDLE_1                    ("FALSE"),
             .CLK_COR_MAX_LAT_1                      (18),
             .CLK_COR_MIN_LAT_1                      (16),
             .CLK_COR_PRECEDENCE_1                   ("TRUE"),
             .CLK_COR_REPEAT_WAIT_1                  (5),
             .CLK_COR_SEQ_1_1_1                      (10'b0100011100),
             .CLK_COR_SEQ_1_2_1                      (10'b0100011100),
             .CLK_COR_SEQ_1_3_1                      (10'b0100011100),
             .CLK_COR_SEQ_1_4_1                      (10'b0100011100),
             .CLK_COR_SEQ_1_ENABLE_1                 (4'b1111),
             .CLK_COR_SEQ_2_1_1                      (10'b0100000000),
             .CLK_COR_SEQ_2_2_1                      (10'b0100000000),
             .CLK_COR_SEQ_2_3_1                      (10'b0100000000),
             .CLK_COR_SEQ_2_4_1                      (10'b0100000000),
             .CLK_COR_SEQ_2_ENABLE_1                 (4'b0000),
             .CLK_COR_SEQ_2_USE_1                    ("FALSE"),
             .CLK_CORRECT_USE_1                      ("TRUE"),
             .RX_DECODE_SEQ_MATCH_1                  ("TRUE"),
            
            //Channel Bonding Attributes
             .CHAN_BOND_1_MAX_SKEW_0                 (1),
             .CHAN_BOND_2_MAX_SKEW_0                 (1),
             .CHAN_BOND_KEEP_ALIGN_0                 ("FALSE"),
             .CHAN_BOND_SEQ_1_1_0                    (10'b0000000000),
             .CHAN_BOND_SEQ_1_2_0                    (10'b0000000000),
             .CHAN_BOND_SEQ_1_3_0                    (10'b0000000000),
             .CHAN_BOND_SEQ_1_4_0                    (10'b0000000000),
             .CHAN_BOND_SEQ_1_ENABLE_0               (4'b0000),
             .CHAN_BOND_SEQ_2_1_0                    (10'b0000000000),
             .CHAN_BOND_SEQ_2_2_0                    (10'b0000000000),
             .CHAN_BOND_SEQ_2_3_0                    (10'b0000000000),
             .CHAN_BOND_SEQ_2_4_0                    (10'b0000000000),
             .CHAN_BOND_SEQ_2_ENABLE_0               (4'b0000),
             .CHAN_BOND_SEQ_2_USE_0                  ("FALSE"),
             .CHAN_BOND_SEQ_LEN_0                    (1),
             .RX_EN_MODE_RESET_BUF_0                 ("FALSE"),
             .CHAN_BOND_1_MAX_SKEW_1                 (1),
             .CHAN_BOND_2_MAX_SKEW_1                 (1),
             .CHAN_BOND_KEEP_ALIGN_1                 ("FALSE"),
             .CHAN_BOND_SEQ_1_1_1                    (10'b0000000000),
             .CHAN_BOND_SEQ_1_2_1                    (10'b0000000000),
             .CHAN_BOND_SEQ_1_3_1                    (10'b0000000000),
             .CHAN_BOND_SEQ_1_4_1                    (10'b0000000000),
             .CHAN_BOND_SEQ_1_ENABLE_1               (4'b0000),
             .CHAN_BOND_SEQ_2_1_1                    (10'b0000000000),
             .CHAN_BOND_SEQ_2_2_1                    (10'b0000000000),
             .CHAN_BOND_SEQ_2_3_1                    (10'b0000000000),
             .CHAN_BOND_SEQ_2_4_1                    (10'b0000000000),
             .CHAN_BOND_SEQ_2_ENABLE_1               (4'b0000),
             .CHAN_BOND_SEQ_2_USE_1                  ("FALSE"),
             .CHAN_BOND_SEQ_LEN_1                    (1),
             .RX_EN_MODE_RESET_BUF_1                 ("FALSE"),
            
            //RX PCI Express Attributes
             .CB2_INH_CC_PERIOD_0                    (8),
             .CDR_PH_ADJ_TIME_0                      (5'b01010),
             .PCI_EXPRESS_MODE_0                     ("FALSE"),
             .RX_EN_IDLE_HOLD_CDR_0                  ("TRUE"),
             .RX_EN_IDLE_RESET_FR_0                  ("TRUE"),
             .RX_EN_IDLE_RESET_PH_0                  ("TRUE"),
             .RX_STATUS_FMT_0                        ("PCIE"),
             .TRANS_TIME_FROM_P2_0                   (12'h03c),
             .TRANS_TIME_NON_P2_0                    (8'h19),
             .TRANS_TIME_TO_P2_0                     (10'h064),
             .CB2_INH_CC_PERIOD_1                    (8),
             .CDR_PH_ADJ_TIME_1                      (5'b01010),
             .PCI_EXPRESS_MODE_1                     ("FALSE"),
             .RX_EN_IDLE_HOLD_CDR_1                  ("TRUE"),
             .RX_EN_IDLE_RESET_FR_1                  ("TRUE"),
             .RX_EN_IDLE_RESET_PH_1                  ("TRUE"),
             .RX_STATUS_FMT_1                        ("PCIE"),
             .TRANS_TIME_FROM_P2_1                   (12'h03c),
             .TRANS_TIME_NON_P2_1                    (8'h19),
             .TRANS_TIME_TO_P2_1                     (10'h064),
            
            //RX SATA Attributes
             .SATA_BURST_VAL_0                       (3'b100),
             .SATA_IDLE_VAL_0                        (3'b100),
             .SATA_MAX_BURST_0                       (7),
             .SATA_MAX_INIT_0                        (22),
             .SATA_MAX_WAKE_0                        (7),
             .SATA_MIN_BURST_0                       (4),
             .SATA_MIN_INIT_0                        (12),
             .SATA_MIN_WAKE_0                        (4),
             .SATA_BURST_VAL_1                       (3'b100),
             .SATA_IDLE_VAL_1                        (3'b100),
             .SATA_MAX_BURST_1                       (7),
             .SATA_MAX_INIT_1                        (22),
             .SATA_MAX_WAKE_1                        (7),
             .SATA_MIN_BURST_1                       (4),
             .SATA_MIN_INIT_1                        (12),
             .SATA_MIN_WAKE_1                        (4)
            )
gtpa1_dual_i( //---------------------- Loopback and Powerdown Ports ----------------------
             .LOOPBACK0                      (3'b0),
             .LOOPBACK1                      (3'b0),
             .RXPOWERDOWN0                   (2'b0),
             .RXPOWERDOWN1                   ({rxtx_pd_i,rxtx_pd_i}),
             .TXPOWERDOWN0                   (2'b0),
             .TXPOWERDOWN1                   ({rxtx_pd_i,rxtx_pd_i}),
             //------------------------------- PLL Ports --------------------------------
             .CLK00                          (gtpclkin_i),
             .CLK01                          (1'b0),
             .CLK10                          (1'b0),
             .CLK11                          (1'b0),
             .CLKINEAST0                     (1'b0),
             .CLKINEAST1                     (1'b0),
             .CLKINWEST0                     (1'b0),
             .CLKINWEST1                     (1'b0),
             .GCLK00                         (1'b0),
             .GCLK01                         (1'b0),
             .GCLK10                         (1'b0),
             .GCLK11                         (1'b0),
             .GTPRESET0                      (1'b0),
             .GTPRESET1                      (1'b0),
             .GTPTEST0                       (8'b00010000),
             .GTPTEST1                       (8'b00010000),
             .INTDATAWIDTH0                  (1'b1),
             .INTDATAWIDTH1                  (1'b1),
             .PLLCLK00                       (1'b0),
             .PLLCLK01                       (1'b0),
             .PLLCLK10                       (1'b0),
             .PLLCLK11                       (1'b0),
             .PLLLKDET0                      (plllkdet_i),
             .PLLLKDET1                      (),
             .PLLLKDETEN0                    (1'b1),
             .PLLLKDETEN1                    (1'b1),
             .PLLPOWERDOWN0                  (1'b0),
             .PLLPOWERDOWN1                  (pll_pd_i),
             .REFCLKOUT0                     (),
             .REFCLKOUT1                     (),
             .REFCLKPLL0                     (),
             .REFCLKPLL1                     (),
             .REFCLKPWRDNB0                  (1'b1),
             .REFCLKPWRDNB1                  (1'b1),
             .REFSELDYPLL0                   (3'b0),
             .REFSELDYPLL1                   (3'b0),
             .RESETDONE0                     (),
             .RESETDONE1                     (),
             .TSTCLK0                        (1'b0),
             .TSTCLK1                        (1'b0),
             .TSTIN0                         (12'b0),
             .TSTIN1                         (12'b0),
             .TSTOUT0                        (),
             .TSTOUT1                        (),
             //--------------------- Receive Ports - 8b10b Decoder ----------------------
             .RXCHARISCOMMA0                 (),
             .RXCHARISCOMMA1                 (),
             .RXCHARISK0                     (),
             .RXCHARISK1                     (),
             .RXDEC8B10BUSE0                 (1'b1),
             .RXDEC8B10BUSE1                 (1'b1),
             .RXDISPERR0                     (),
             .RXDISPERR1                     (),
             .RXNOTINTABLE0                  (),
             .RXNOTINTABLE1                  (),
             .RXRUNDISP0                     (),
             .RXRUNDISP1                     (),
             .USRCODEERR0                    (1'b0),
             .USRCODEERR1                    (1'b0),
             //-------------------- Receive Ports - Channel Bonding ---------------------
             .RXCHANBONDSEQ0                 (),
             .RXCHANBONDSEQ1                 (),
             .RXCHANISALIGNED0               (),
             .RXCHANISALIGNED1               (),
             .RXCHANREALIGN0                 (),
             .RXCHANREALIGN1                 (),
             .RXCHBONDI                      (3'b0),
             .RXCHBONDMASTER0                (1'b0),
             .RXCHBONDMASTER1                (1'b0),
             .RXCHBONDO                      (),
             .RXCHBONDSLAVE0                 (1'b0),
             .RXCHBONDSLAVE1                 (1'b0),
             .RXENCHANSYNC0                  (1'b0),
             .RXENCHANSYNC1                  (1'b0),
             //-------------------- Receive Ports - Clock Correction --------------------
             .RXCLKCORCNT0                   (),
             .RXCLKCORCNT1                   (),
             //------------- Receive Ports - Comma Detection and Alignment --------------
             .RXBYTEISALIGNED0               (),
             .RXBYTEISALIGNED1               (),
             .RXBYTEREALIGN0                 (),
             .RXBYTEREALIGN1                 (),
             .RXCOMMADET0                    (),
             .RXCOMMADET1                    (),
             .RXCOMMADETUSE0                 (1'b1),
             .RXCOMMADETUSE1                 (1'b1),
             .RXENMCOMMAALIGN0               (1'b1), // Always realign
             .RXENMCOMMAALIGN1               (1'b1), // Always realign
             .RXENPCOMMAALIGN0               (1'b1), // Always realign
             .RXENPCOMMAALIGN1               (1'b1), // Always realign
             .RXSLIDE0                       (1'b0),
             .RXSLIDE1                       (1'b0),
             //--------------------- Receive Ports - PRBS Detection ---------------------
             .PRBSCNTRESET0                  (1'b0),
             .PRBSCNTRESET1                  (1'b0),
             .RXENPRBSTST0                   (3'b0),
             .RXENPRBSTST1                   (3'b100),
             .RXPRBSERR0                     (),
             .RXPRBSERR1                     (prbserr_i),
             //----------------- Receive Ports - RX Data Path interface -----------------
             .RXDATA0                        (),
             .RXDATA1                        (),
             .RXDATAWIDTH0                   (2'b10),
             .RXDATAWIDTH1                   (2'b10),
             .RXRECCLK0                      (),
             .RXRECCLK1                      (),
             .RXRESET0                       (1'b0),
             .RXRESET1                       (1'b0),
             .RXUSRCLK0                      (usrclk_i),
             .RXUSRCLK1                      (usrclk_i),
             .RXUSRCLK20                     (usrclk2_i),
             .RXUSRCLK21                     (usrclk2_i),
             //----- Receive Ports - RX Driver,OOB signalling,Coupling and Eq.,CDR ------
             .GATERXELECIDLE0                (1'b0),
             .GATERXELECIDLE1                (1'b0),
             .IGNORESIGDET0                  (1'b0),
             .IGNORESIGDET1                  (1'b0),
             .RCALINEAST                     (5'b0),
             .RCALINWEST                     (5'b0),
             .RCALOUTEAST                    (),
             .RCALOUTWEST                    (),
             .RXCDRRESET0                    (1'b0),
             .RXCDRRESET1                    (1'b0),
             .RXELECIDLE0                    (),
             .RXELECIDLE1                    (),
             .RXEQMIX0                       (RXEQMIX),
             .RXEQMIX1                       (RXEQMIX),
             .RXN0                           (1'bX),
             .RXN1                           (SATA1_RXN_IN),
             .RXP0                           (1'bX),
             .RXP1                           (SATA1_RXP_IN),
             //--------- Receive Ports - RX Elastic Buffer and Phase Alignment ----------
             .RXBUFRESET0                    (1'b0),
             .RXBUFRESET1                    (1'b0),
             .RXBUFSTATUS0                   (),
             .RXBUFSTATUS1                   (),
             .RXENPMAPHASEALIGN0             (1'b0),
             .RXENPMAPHASEALIGN1             (1'b0),
             .RXPMASETPHASE0                 (1'b0),
             .RXPMASETPHASE1                 (1'b0),
             .RXSTATUS0                      (),
             .RXSTATUS1                      (),
             //------------- Receive Ports - RX Loss-of-sync State Machine --------------
             .RXLOSSOFSYNC0                  (),
             .RXLOSSOFSYNC1                  (),
             //------------ Receive Ports - RX Pipe Control for PCI Express -------------
             .PHYSTATUS0                     (),
             .PHYSTATUS1                     (),
             .RXVALID0                       (),
             .RXVALID1                       (rxvalid_i),
             //------------------ Receive Ports - RX Polarity Control -------------------
             .RXPOLARITY0                    (1'b0),
             .RXPOLARITY1                    (1'b0),
             //----------- Shared Ports - Dynamic Reconfiguration Port (DRP) ------------
             .DADDR                          (8'b0),
             .DCLK                           (1'b0),
             .DEN                            (1'b0),
             .DI                             (16'b0),
             .DRDY                           (),
             .DRPDO                          (),
             .DWE                            (1'b0),
             //-------------------------- TX/RX Datapath Ports --------------------------
             .GTPCLKFBEAST                   (),
             .GTPCLKFBSEL0EAST               (2'b10),
             .GTPCLKFBSEL0WEST               (2'b00),
             .GTPCLKFBSEL1EAST               (2'b11),
             .GTPCLKFBSEL1WEST               (2'b01),
             .GTPCLKFBWEST                   (),
             .GTPCLKOUT0                     (unbuffered_gtpclkout_i),
             .GTPCLKOUT1                     (),
             //----------------- Transmit Ports - 8b10b Encoder Control -----------------
             .TXBYPASS8B10B0                 (4'b0),
             .TXBYPASS8B10B1                 (4'b0),
             .TXCHARDISPMODE0                (4'b0),
             .TXCHARDISPMODE1                (4'b0),
             .TXCHARDISPVAL0                 (4'b0),
             .TXCHARDISPVAL1                 (4'b0),
             .TXCHARISK0                     (4'b0),
             .TXCHARISK1                     (4'b0),
             .TXENC8B10BUSE0                 (1'b1),
             .TXENC8B10BUSE1                 (1'b1),
             .TXKERR0                        (),
             .TXKERR1                        (),
             .TXRUNDISP0                     (),
             .TXRUNDISP1                     (),
             //------------- Transmit Ports - TX Buffer and Phase Alignment -------------
             .TXBUFSTATUS0                   (),
             .TXBUFSTATUS1                   (),
             .TXENPMAPHASEALIGN0             (1'b0),
             .TXENPMAPHASEALIGN1             (1'b0),
             .TXPMASETPHASE0                 (1'b0),
             .TXPMASETPHASE1                 (1'b0),
             //---------------- Transmit Ports - TX Data Path interface -----------------
             .TXDATA0                        (32'hDEADBEEF),
             .TXDATA1                        (32'hDEADBEEF),
             .TXDATAWIDTH0                   (2'b10), // 32-bits
             .TXDATAWIDTH1                   (2'b10), // 32-bits
             .TXOUTCLK0                      (),
             .TXOUTCLK1                      (),
             .TXRESET0                       (1'b0),
             .TXRESET1                       (1'b0),
             .TXUSRCLK0                      (usrclk_i),
             .TXUSRCLK1                      (usrclk_i),
             .TXUSRCLK20                     (usrclk2_i),
             .TXUSRCLK21                     (usrclk2_i),
             //------------- Transmit Ports - TX Driver and OOB signalling --------------
             .TXBUFDIFFCTRL0                 (3'b101),
             .TXBUFDIFFCTRL1                 (3'b101),
             .TXDIFFCTRL0                    (TXDIFFCTRL),
             .TXDIFFCTRL1                    (TXDIFFCTRL),
             .TXINHIBIT0                     (1'b0),
             .TXINHIBIT1                     (1'b0),
             .TXN0                           (), // PCI-E not connected
             .TXN1                           (SATA1_TXN_OUT),
             .TXP0                           (), // PCI-E not connected
             .TXP1                           (SATA1_TXP_OUT),
             .TXPREEMPHASIS0                 (TXPREEMPHASIS),
             .TXPREEMPHASIS1                 (TXPREEMPHASIS),
             //------------------- Transmit Ports - TX PRBS Generator -------------------
             .TXENPRBSTST0                   (3'b0),
             .TXENPRBSTST1                   (3'b100),
             .TXPRBSFORCEERR0                (1'b0),
             .TXPRBSFORCEERR1                (1'b0),
             //------------------ Transmit Ports - TX Polarity Control ------------------
             .TXPOLARITY0                    (1'b0),
             .TXPOLARITY1                    (1'b0),
             //--------------- Transmit Ports - TX Ports for PCI Express ----------------
             .TXDETECTRX0                    (1'b0),
             .TXDETECTRX1                    (1'b0),
             .TXELECIDLE0                    (1'b0),
             .TXELECIDLE1                    (1'b0),
             .TXPDOWNASYNCH0                 (1'b0),
             .TXPDOWNASYNCH1                 (1'b0),
             //------------------- Transmit Ports - TX Ports for SATA -------------------
             .TXCOMSTART0                    (1'b0),
             .TXCOMSTART1                    (1'b0),
             .TXCOMTYPE0                     (1'b0),
             .TXCOMTYPE1                     (1'b0)
            );

endmodule
