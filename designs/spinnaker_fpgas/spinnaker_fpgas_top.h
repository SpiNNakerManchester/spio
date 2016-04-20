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
localparam [(32*3)-1:0] FPGA_SL_TYPES = { // FPGA2
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

// the back-pressure point (bpp) is different for each link
// 3FPGAs, 16 links per FPGA, 4 bits per link
localparam [(3*16*4)-1:0] BPP = { // FPGA2
                                  4'd6, 4'd7, 4'd6, 4'd7
                                , 4'd6, 4'd7, 4'd6, 4'd7
                                , 4'd6, 4'd9, 4'd6, 4'd9
                                , 4'd6, 4'd9, 4'd6, 4'd9
                                  // FPGA1
                                , 4'd6, 4'd9, 4'd6, 4'd9
                                , 4'd6, 4'd9, 4'd6, 4'd9
                                , 4'd6, 4'd6, 4'd6, 4'd6
                                , 4'd6, 4'd6, 4'd6, 4'd6
                                  // FPGA0
                                , 4'd5, 4'd6, 4'd5, 4'd6
                                , 4'd5, 4'd6, 4'd5, 4'd6
                                , 4'd5, 4'd7, 4'd5, 4'd7
                                , 4'd5, 4'd7, 4'd5, 4'd7
                                };

// the number of "synchronous" flits before the back-pressure point
// is different for each link
// 3FPGAs, 16 links per FPGA, 5 bits per link
localparam [(3*16*5)-1:0] BSF_LONG = { // FPGA2
                                       5'd17, 5'd17, 5'd17, 5'd17
                                     , 5'd17, 5'd17, 5'd17, 5'd17
                                     , 5'd17, 5'd17, 5'd17, 5'd17
                                     , 5'd17, 5'd17, 5'd17, 5'd17
                                       // FPGA1
                                     , 5'd17, 5'd17, 5'd17, 5'd17
                                     , 5'd17, 5'd17, 5'd17, 5'd17
                                     , 5'd17, 5'd17, 5'd17, 5'd17
                                     , 5'd17, 5'd17, 5'd17, 5'd17
                                       // FPGA0
                                     , 5'd15, 5'd17, 5'd15, 5'd17
                                     , 5'd15, 5'd17, 5'd15, 5'd17
                                     , 5'd15, 5'd17, 5'd15, 5'd17
                                     , 5'd15, 5'd17, 5'd15, 5'd17
                                     };

// the number of "asynchronous" flits before the back-pressure point
// is different for each link
// 3FPGAs, 16 links per FPGA, 3 bits per link
localparam [(3*16*3)-1:0] BAF_LONG = { // FPGA2
                                       3'd2, 3'd2, 3'd2, 3'd2
                                     , 3'd2, 3'd2, 3'd2, 3'd2
                                     , 3'd2, 3'd2, 3'd2, 3'd2
                                     , 3'd2, 3'd2, 3'd2, 3'd2
                                       // FPGA1
                                     , 3'd2, 3'd2, 3'd2, 3'd2
                                     , 3'd2, 3'd2, 3'd2, 3'd2
                                     , 3'd2, 3'd2, 3'd2, 3'd2
                                     , 3'd2, 3'd2, 3'd2, 3'd2
                                       // FPGA0
                                     , 3'd4, 3'd2, 3'd4, 3'd2
                                     , 3'd4, 3'd2, 3'd4, 3'd2
                                     , 3'd4, 3'd2, 3'd4, 3'd2
                                     , 3'd4, 3'd2, 3'd4, 3'd2
                                     };

`endif
