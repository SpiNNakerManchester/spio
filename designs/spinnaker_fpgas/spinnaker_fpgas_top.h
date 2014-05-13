/**
 * FPGA identifier constants, defined as localparams so must be included within
 * a module. Only broken out into a header file for the benefit of the
 * testbench.
 */

`ifndef SPINNAKER_FPGAS_TOP_H
`define SPINNAKER_FPGAS_TOP_H

// FPGA unique identifiers (as in FPGA_ID_IN)
localparam FPGA0 = 2'b00;
localparam FPGA1 = 2'b01;
localparam FPGA2 = 2'b10;

// In the below bit fields, 1=input, 0=output with the LSB corresponding to the
// SpiNNaker -> FPGA data pins and FPGA -> SpiNNaker ack pin and the MSB the
// FPGA -> SpiNNaker data pins and SpiNNaker -> FPGA ack pin.
localparam LOW_SL    = 2'b10;
localparam HIGH_SL   = 2'b01;
localparam UNUSED_SL = 2'b11;

localparam [31:0] FPGA0_SL_TYPES = { HIGH_SL, HIGH_SL, HIGH_SL, HIGH_SL
                                   , HIGH_SL, HIGH_SL, HIGH_SL, HIGH_SL
                                   , HIGH_SL, LOW_SL,  HIGH_SL, LOW_SL
                                   , HIGH_SL, LOW_SL,  HIGH_SL, LOW_SL
                                   };

localparam [31:0] FPGA1_SL_TYPES = { HIGH_SL, LOW_SL,  HIGH_SL, LOW_SL
                                   , HIGH_SL, LOW_SL,  HIGH_SL, LOW_SL
                                   , HIGH_SL, HIGH_SL, HIGH_SL, HIGH_SL
                                   , HIGH_SL, HIGH_SL, HIGH_SL, HIGH_SL
                                   };

localparam [31:0] FPGA2_SL_TYPES = { LOW_SL,  LOW_SL,  LOW_SL,  LOW_SL
                                   , LOW_SL,  LOW_SL,  LOW_SL,  LOW_SL
                                   , LOW_SL,  LOW_SL,  LOW_SL,  LOW_SL
                                   , LOW_SL,  LOW_SL,  LOW_SL,  LOW_SL
                                   };

`endif
