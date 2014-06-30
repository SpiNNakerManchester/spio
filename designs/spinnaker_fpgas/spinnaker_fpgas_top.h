/**
 * FPGA identifier constants, defined as localparams so must be included within
 * a module. Only broken out into a header file for the benefit of the
 * testbench.
 */

`ifndef SPINNAKER_FPGAS_TOP_H
`define SPINNAKER_FPGAS_TOP_H

// In the below bit fields, 1=input, 0=output with the LSB corresponding to the
// SpiNNaker -> FPGA data pins and FPGA -> SpiNNaker ack pin and the MSB the
// FPGA -> SpiNNaker data pins and SpiNNaker -> FPGA ack pin.
localparam LOW_SL    = 2'b10;
localparam HIGH_SL   = 2'b01;
localparam UNUSED_SL = 2'b11;

// The first 32 bits are for FPGA0, the next for FPGA1 and the last for FPGA2.
// This is to work around the inability of some simulators to support literal
// assignment to arrays.
localparam [31:0] FPGA_SL_TYPES = { // FPGA2
                                    LOW_SL,  LOW_SL,  LOW_SL,  LOW_SL
                                  , LOW_SL,  LOW_SL,  LOW_SL,  LOW_SL
                                  , LOW_SL,  LOW_SL,  LOW_SL,  LOW_SL
                                  , LOW_SL,  LOW_SL,  LOW_SL,  LOW_SL
                                    // FPGA1
                                  , HIGH_SL, LOW_SL,  HIGH_SL, LOW_SL
                                  , HIGH_SL, LOW_SL,  HIGH_SL, LOW_SL
                                  , HIGH_SL, HIGH_SL, HIGH_SL, HIGH_SL
                                  , HIGH_SL, HIGH_SL, HIGH_SL, HIGH_SL
                                    // FPGA0
                                  , HIGH_SL, HIGH_SL, HIGH_SL, HIGH_SL
                                  , HIGH_SL, HIGH_SL, HIGH_SL, HIGH_SL
                                  , HIGH_SL, LOW_SL,  HIGH_SL, LOW_SL
                                  , HIGH_SL, LOW_SL,  HIGH_SL, LOW_SL
                                  };

`endif
