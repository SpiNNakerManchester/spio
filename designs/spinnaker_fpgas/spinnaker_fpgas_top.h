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

// The 32 least-significant bits are for FPGA0, the next for FPGA1 and the
// last for FPGA2. This is to work around the inability of some simulators
// to support literal assignment to arrays.
// 3 FPGAs, 16 links per FPGA, 2 bits per link
localparam [(3*16*2)-1:0] FPGA_SL_TYPES = { // FPGA2
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

// the STAC configuration of the SpiNNaker link senders is different for
// every SpiNNaker link type (0-5). This array stores the SpiNNkaker link
// type connected to each FPGA link.
// 3 FPGAs, 16 links per FPGA, 3 bits per link
localparam [(3*16*3)-1:0] SPIN_LINK_TYPE = { // FPGA2
                                             3'd1, 3'd0, 3'd1, 3'd0
                                           , 3'd1, 3'd0, 3'd1, 3'd0
                                           , 3'd1, 3'd2, 3'd1, 3'd2
                                           , 3'd1, 3'd2, 3'd1, 3'd2
                                             // FPGA1
                                           , 3'd3, 3'd2, 3'd3, 3'd2
                                           , 3'd3, 3'd2, 3'd3, 3'd2
                                           , 3'd3, 3'd4, 3'd3, 3'd4
                                           , 3'd3, 3'd4, 3'd3, 3'd4
                                             // FPGA0
                                           , 3'd5, 3'd4, 3'd5, 3'd4
                                           , 3'd5, 3'd4, 3'd5, 3'd4
                                           , 3'd5, 3'd0, 3'd5, 3'd0
                                           , 3'd5, 3'd0, 3'd5, 3'd0
                                           };

`endif
