/**
 * Common values used by spio_hss_multiplexer.
 */

// Protocol Version Identifier (checked at time of handshake)
`define VERSION 8'h01

////////////////////////////////////////////////////////////////////////////////
// K-Characters
////////////////////////////////////////////////////////////////////////////////

// 8b/10b K28.0: Used for clock correction. A clock correction sequence consists
// of four of these.
`define KCHR_CLKC 8'h1C

// 8b/10b K28.2: Used within the rx/tx_control handshake phase to uniqely
// identify the handshake words.
`define KCHR_HANDSHAKE 8'h5C

// 8b/10b K28.5: Commma symbol used to ensure byte-allignment
`define KCHR_COMMA 8'hBC
