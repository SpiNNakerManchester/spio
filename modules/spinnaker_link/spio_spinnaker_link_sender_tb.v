// -------------------------------------------------------------------------
//  SpiNNaker link transmitter testbench module
//
// -------------------------------------------------------------------------
// AUTHOR
//  lap - luis.plana@manchester.ac.uk
//
// -------------------------------------------------------------------------
// COPYRIGHT
//  Copyright (c) The University of Manchester, 2012. All rights reserved.
//  SpiNNaker Project
//  Advanced Processor Technologies Group
//  School of Computer Science
// -------------------------------------------------------------------------
// TODO
// -------------------------------------------------------------------------


`define PKT_HDR_RNG       0 +: 8
`define PKT_KEY_RNG       8 +: 32
`define PKT_PLD_RNG      40 +: 32


`timescale 1ns / 1ps
module spio_spinnaker_link_sender_tb ();
//---------------------------------------------------------------
// constants
//---------------------------------------------------------------
localparam UUT_CLK_HPER = (6.666 / 2);  // currently testing @ 150 MHz
localparam TB_CLK_HPER  = (6.666 / 2);  // currently testing @ 150 MHz

//!! localparam SPL_HSDLY = 16;  // external link delay estimate
localparam SPL_HSDLY = 23;  // external link delay estimate (includes SpiNNaker)

localparam INIT_DELAY = (10 * TB_CLK_HPER);
localparam RST_DELAY  = (51 * TB_CLK_HPER);  // align with clock posedge

localparam COMB_DELAY = 2;


//---------------------------------------------------------------
// internal signals
//---------------------------------------------------------------
reg         tb_rst;
reg         tb_clk;

reg         uut_rst;
reg         uut_clk;

wire [71:0] ipkt_data;
reg         ipkt_vld;
wire        ipkt_rdy;

wire  [6:0] ospl_data;
reg         ospl_ack;
wire        ospl_sync_ack;


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//--------------------------- functions -------------------------
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//---------------------------------------------------------------
// NRZ 2-of-7 decoder
//---------------------------------------------------------------
function [3:0] decode_nrz_2of7 ;
  input [6:0] data;
  input [6:0] old_data;

  case (data ^ old_data)
    7'b0010001: decode_nrz_2of7 = 0;    // 0
    7'b0010010: decode_nrz_2of7 = 1;    // 1
    7'b0010100: decode_nrz_2of7 = 2;    // 2
    7'b0011000: decode_nrz_2of7 = 3;    // 3
    7'b0100001: decode_nrz_2of7 = 4;    // 4
    7'b0100010: decode_nrz_2of7 = 5;    // 5
    7'b0100100: decode_nrz_2of7 = 6;    // 6
    7'b0101000: decode_nrz_2of7 = 7;    // 7
    7'b1000001: decode_nrz_2of7 = 8;    // 8
    7'b1000010: decode_nrz_2of7 = 9;    // 9
    7'b1000100: decode_nrz_2of7 = 10;   // 10
    7'b1001000: decode_nrz_2of7 = 11;   // 11
    7'b0000011: decode_nrz_2of7 = 12;   // 12
    7'b0000110: decode_nrz_2of7 = 13;   // 13
    7'b0001100: decode_nrz_2of7 = 14;   // 14
    7'b0001001: decode_nrz_2of7 = 15;   // 15
    default:    decode_nrz_2of7 = 4'hx; // eop, incomplete, oob
  endcase
endfunction
//---------------------------------------------------------------

//---------------------------------------------------------------
// NRZ 2-of-7 eop detection
//---------------------------------------------------------------
function [3:0] eop_nrz_2of7 ;
  input [6:0] data;
  input [6:0] old_data;

  case (data ^ old_data)
    7'b1100000:   // eop
                eop_nrz_2of7 = 1;
    default:    eop_nrz_2of7 = 0;
  endcase
endfunction
//---------------------------------------------------------------

//---------------------------------------------------------------
// NRZ 2-of-7 completion detection
//---------------------------------------------------------------
function [3:0] complete_nrz_2of7 ;
  input [6:0] data;
  input [6:0] old_data;

  case (data ^ old_data)
    7'b0010001,   // 0
    7'b0010010,   // 1
    7'b0010100,   // 2
    7'b0011000,   // 3
    7'b0100001,   // 4
    7'b0100010,   // 5
    7'b0100100,   // 6
    7'b0101000,   // 7
    7'b1000001,   // 8
    7'b1000010,   // 9
    7'b1000100,   // 10
    7'b1001000,   // 11
    7'b0000011,   // 12
    7'b0000110,   // 13
    7'b0001100,   // 14
    7'b0001001,   // 15
    7'b1100000:   // eop
                complete_nrz_2of7 = 1;
    default:    complete_nrz_2of7 = 0;
  endcase
endfunction
//---------------------------------------------------------------
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//--------------------------- datapath --------------------------
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
spio_spinnaker_link_sender uut
(
  .CLK_IN           (uut_clk),
  .RESET_IN         (uut_rst),

  // synchronous packet interface
  .PKT_DATA_IN      (ipkt_data),
  .PKT_VLD_IN       (ipkt_vld),
  .PKT_RDY_OUT      (ipkt_rdy),

  // SpiNNaker link asynchronous interface
  .SL_DATA_2OF7_OUT (ospl_data),
  .SL_ACK_IN        (ospl_sync_ack)
);

// synchronize acknowledge into uut
spio_spinnaker_link_sync 
#(.SIZE(1)
)
sync
(
  .CLK_IN (uut_clk),
  .IN     (ospl_ack),
  .OUT    (ospl_sync_ack)
);
//--------------------------------------------------
// drive the input packet interface
//--------------------------------------------------
reg  [31:0] tb_cnt;

wire  [7:0] tb_hdr; 
reg  [31:0] tb_key;
reg  [31:0] tb_pld;

reg         tb_send_pld;

assign tb_hdr = {6'b000000, tb_send_pld, ^(tb_key ^ tb_pld)};  // compute parity!

always @ (posedge tb_clk or posedge tb_rst)
  if (tb_rst)
  begin
    tb_key      <= 32'h0000_0001;
    tb_pld      <= 32'ha5a5_a5a5;
    tb_send_pld <= 1'b0;
  end
  else
    if (ipkt_vld && ipkt_rdy)
      tb_key <= # COMB_DELAY tb_key + 1;

assign ipkt_data = {tb_pld, tb_key, tb_hdr};

always @ (posedge tb_clk or posedge tb_rst)
  if (tb_rst)
    ipkt_vld <= # COMB_DELAY 1'b0;
  else
    if (tb_cnt == 0)
      ipkt_vld <= # COMB_DELAY 1'b1;
    else
      ipkt_vld <= # COMB_DELAY 1'b0;

always @ (posedge tb_clk or posedge tb_rst)
  if (tb_rst)
    tb_cnt <= 50;
  else
    if (!ipkt_rdy)
      tb_cnt = 10;
    else if (tb_cnt != 0)
      tb_cnt <= tb_cnt - 1;
//--------------------------------------------------

//--------------------------------------------------
// drive the output SpiNNaker link interface
//--------------------------------------------------
reg   [6:0] old_data;
wire  [3:0] tb_rec_data;
wire        tb_rec_eop;

reg   [4:0] tb_fltc;

reg  [71:0] tb_opkt;
wire  [7:0] tb_opkt_hdr; 
wire [31:0] tb_opkt_key;
wire [31:0] tb_opkt_pld;


assign tb_rec_data = decode_nrz_2of7 (ospl_data, old_data);
assign tb_rec_eop  = eop_nrz_2of7 (ospl_data, old_data);

assign tb_opkt_hdr = tb_opkt[`PKT_HDR_RNG];
assign tb_opkt_key = tb_opkt[`PKT_KEY_RNG];
assign tb_opkt_pld = tb_opkt[`PKT_PLD_RNG];

initial
begin
  old_data = 0;
  ospl_ack = 0;

  tb_opkt = 72'hxxxxxxxx_xxxxxxxx_xx;
  tb_fltc = 0;

  wait (tb_rst);   // wait for reset to be applied
  wait (!tb_rst);  // wait for reset to be released

  # COMB_DELAY
    ospl_ack = 1;  // initial ack (mimic SpiNNaker behaviour)
  old_data = ospl_data;  // remember data (for nrz completion detection)

  forever
  begin
    wait (complete_nrz_2of7 (ospl_data, old_data));

    if (tb_rec_eop)
    begin
      tb_opkt = 72'hxxxxxxxx_xxxxxxxx_xx;
      tb_fltc = 0;
    end
    else
    begin
      tb_opkt[tb_fltc * 4 +: 4] = tb_rec_data;
      tb_fltc = tb_fltc + 1;
    end

    # SPL_HSDLY
      ospl_ack = ~ospl_ack;
    old_data = ospl_data;
  end
end
//--------------------------------------------------
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//---------------------------- control --------------------------
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//--------------------------------------------------
// reset signals
//--------------------------------------------------
initial
begin
  tb_rst  = 1'b0;
  uut_rst = 1'b0;

  // wait a few clock cycles before triggering the reset signals
  # INIT_DELAY
    tb_rst  = 1'b1;
    uut_rst = 1'b1;

  # RST_DELAY
    uut_rst = 1'b0;  // release uut

//  # RST_DELAY 
    tb_rst  = 1'b0;  // release tb
end
//--------------------------------------------------

//--------------------------------------------------
// clock signals
//--------------------------------------------------
initial
begin
   tb_clk = 1'b0;

  forever
  begin
    # TB_CLK_HPER
      tb_clk = ~tb_clk;
   end
end

initial
begin
   uut_clk = 1'b0;

  forever
  begin
    # UUT_CLK_HPER
      uut_clk = ~uut_clk;
   end
end
//--------------------------------------------------
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
endmodule
