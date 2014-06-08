/**
 * Common (internal) values shared by the UART modules.
 */

`ifndef SPIO_UART_COMMON_H
`define SPIO_UART_COMMON_H

// Line levels for various types of bit
`define LINE_IDLE 1'b1
`define START_BIT 1'b0
`define STOP_BIT  1'b1

// (Maximum) SpiNNaker packet length
`define PKT_LEN 72

// Short (payload-less) SpiNNaker packet length
`define SPKT_LEN 40

// Determine if a packet is long or short given the header.
// TODO

`endif
