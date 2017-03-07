/**
 * Common constants, defined as localparams, that can
 * be included within a module.
 */

`ifndef RAGGEDSTONE_SPINN_AER_IF_TOP_H
`define RAGGEDSTONE_SPINN_AER_IF_TOP_H

// ---------------------
// virtual coord
// ---------------------
localparam VC_BITS = 1;

// virtual coord options
localparam VC_DEF = 0;
localparam VC_ALT = VC_DEF + 1;
localparam LAST_VC = VC_ALT;

// alternative virtual coordinates
localparam VIRTUAL_COORD_DEF = 16'h0200;
localparam VIRTUAL_COORD_ALT = 16'hfefe;
// ---------------------

// ---------------------
// mode
// ---------------------
localparam MODE_BITS = 4;

// mode options
localparam RET_128 = 0;
localparam RET_64  = RET_128 + 1;
localparam RET_32  = RET_64  + 1;
localparam RET_16  = RET_32  + 1;
localparam COCHLEA = RET_16  + 1;
localparam DIRECT  = COCHLEA + 1;
localparam LAST_MODE = DIRECT;

// ---------------------
// control and routing
// ---------------------
localparam CTRL_KEY = 32'h0000fffe
localparam CTRL_MSK = 32'h0000fffe

`endif
