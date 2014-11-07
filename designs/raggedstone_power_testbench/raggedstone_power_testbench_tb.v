`timescale 1ns / 1ps

module raggedstone_power_testbench_tb ();

reg nreset_i;
reg refclk_i;

wire [5:2] leds_i;

reg [1:0] buttons_i;

// Loopback all ports
wire sata1_txn;
wire sata1_txp;
wire sata1_rxn = sata1_txn;
wire sata1_rxp = sata1_txp;

wire sata2_txn;
wire sata2_txp;
wire sata2_rxn = sata2_txn;
wire sata2_rxp = sata2_txp;

wire sata3_txn;
wire sata3_txp;
wire sata3_rxn = sata3_txn;
wire sata3_rxp = sata3_txp;


// Device Under Test
raggedstone_power_testbench #( // Enable simulation mode for GTP tile
                               .SIM_GTPRESET_SPEEDUP(0)
                             )
raggedstone_power_testbench_i( .NBUTTON_IN(buttons_i)
                             , .LEDS_OUT(leds_i)
                               
                               // Differential 150 MHz clock source
                             , .REFCLK_PAD_P_IN(refclk_i)
                             , .REFCLK_PAD_N_IN(~refclk_i)
                             
                               // Wires for S-ATA ports
                             , .SATA1_RXN_IN (sata1_rxn)
                             , .SATA1_RXP_IN (sata1_rxp)
                             , .SATA1_TXN_OUT(sata1_txn)
                             , .SATA1_TXP_OUT(sata1_txp)
                             
                             , .SATA2_RXN_IN (sata2_rxn)
                             , .SATA2_RXP_IN (sata2_rxp)
                             , .SATA2_TXN_OUT(sata2_txn)
                             , .SATA2_TXP_OUT(sata2_txp)
                             
                             , .SATA3_RXN_IN (sata3_rxn)
                             , .SATA3_RXP_IN (sata3_rxp)
                             , .SATA3_TXN_OUT(sata3_txn)
                             , .SATA3_TXP_OUT(sata3_txp)
                             );

// Ref clock generation
initial refclk_i = 0;
always #3.3333 refclk_i = ~refclk_i;


initial
	buttons_i = 2'b11;


endmodule
