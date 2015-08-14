// -------------------------------------------------------------------------
//  SpiNNaker link receiver testbench module
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
// - test for variable asynchronous delay on input interface
// - test error flits
// -------------------------------------------------------------------------


`define PKT_HDR_RNG       0 +: 8
`define PKT_KEY_RNG       8 +: 32
`define PKT_PLD_RNG      40 +: 32

`define EOP              5'b1_0000


`timescale 1ns / 1ps
module spio_spinnaker_link_receiver_tb ();
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

reg   [6:0] uut_ispl_data;
wire        uut_ispl_ack;

wire [71:0] uut_opkt_data;
wire        uut_opkt_vld;
reg         uut_opkt_rdy;

wire [71:0] tb_ipkt;
wire  [7:0] tb_ipkt_hdr; 
reg   [1:0] tb_ipkt_type; 
wire        tb_ipkt_prty; 
reg  [31:0] tb_ipkt_key;
reg  [31:0] tb_ipkt_pld;
reg         tb_ipkt_send_pld;
wire        tb_ipkt_send_eop;

reg   [6:0] tb_old_data;
reg         tb_old_ack;

reg   [4:0] tb_flt_cnt;

wire  [7:0] tb_opkt_hdr; 
wire [31:0] tb_opkt_key;
wire [31:0] tb_opkt_pld;

reg         tb_bad_pkt;

reg  [71:0] tb_fifo_pkt [0:7];
reg   [2:0] tb_fifo_rdp;
reg   [2:0] tb_fifo_wrp;

reg [31:0] tb_pkt_cnt;
reg        tb_cnt_on;
reg [31:0] tb_cycle_cnt;
reg [31:0] tb_start_cnt;


//---------------------------------------------------------------
// functions
//---------------------------------------------------------------

//-------------------------------------------------------------
// NRZ 2-of-7 encoder
//-------------------------------------------------------------
function [6:0] encode_nrz_2of7 ;
  input [4:0] data;
  input [6:0] old_data;

  casex (data)
    5'b00000 : encode_nrz_2of7 = old_data ^ 7'b0010001; // 0
    5'b00001 : encode_nrz_2of7 = old_data ^ 7'b0010010; // 1
    5'b00010 : encode_nrz_2of7 = old_data ^ 7'b0010100; // 2
    5'b00011 : encode_nrz_2of7 = old_data ^ 7'b0011000; // 3
    5'b00100 : encode_nrz_2of7 = old_data ^ 7'b0100001; // 4
    5'b00101 : encode_nrz_2of7 = old_data ^ 7'b0100010; // 5
    5'b00110 : encode_nrz_2of7 = old_data ^ 7'b0100100; // 6
    5'b00111 : encode_nrz_2of7 = old_data ^ 7'b0101000; // 7
    5'b01000 : encode_nrz_2of7 = old_data ^ 7'b1000001; // 8
    5'b01001 : encode_nrz_2of7 = old_data ^ 7'b1000010; // 9
    5'b01010 : encode_nrz_2of7 = old_data ^ 7'b1000100; // 10
    5'b01011 : encode_nrz_2of7 = old_data ^ 7'b1001000; // 11
    5'b01100 : encode_nrz_2of7 = old_data ^ 7'b0000011; // 12
    5'b01101 : encode_nrz_2of7 = old_data ^ 7'b0000110; // 13
    5'b01110 : encode_nrz_2of7 = old_data ^ 7'b0001100; // 14
    5'b01111 : encode_nrz_2of7 = old_data ^ 7'b0001001; // 15
    5'b1xxxx : encode_nrz_2of7 = old_data ^ 7'b1100000; // EOP
    default  : encode_nrz_2of7 = 7'bxxxxxxx;
  endcase
endfunction


//---------------------------------------------------------------
// unit under test
//---------------------------------------------------------------
spio_spinnaker_link_receiver uut
(
  .CLK_IN           (uut_clk),
  .RESET_IN         (uut_rst),

  // incoming SpiNNaker link interface
  .SL_DATA_2OF7_IN  (uut_ispl_data),
  .SL_ACK_OUT       (uut_ispl_ack),

  // outgoing packet interface
  .PKT_DATA_OUT     (uut_opkt_data),
  .PKT_VLD_OUT      (uut_opkt_vld),
  .PKT_RDY_IN       (uut_opkt_rdy)
);


//--------------------------------------------------
// SpiNNaker link interface: generate 2of7 data flits
//--------------------------------------------------
//TODO: test different (maybe time-varying) asynchronous delays!
initial
begin
  wait (tb_rst);   // wait for reset to be applied

  # COMB_DELAY;

  // initialize interface signals
  uut_ispl_data = 0;
  tb_old_data = 0;

  // initialize first packet
  tb_ipkt_type = 0;
  tb_ipkt_key = 32'h0000_0001;
  tb_ipkt_pld = 32'ha5a5_a5a5;
  tb_ipkt_send_pld = 1'b0;

  // initialize fifo write pointer
  tb_fifo_wrp = 0;

  tb_flt_cnt = 0;

  wait (!tb_rst);  // wait for reset to be released

  # COMB_DELAY;
   
  // store first packet in fifo for later checking
  tb_fifo_pkt[tb_fifo_wrp] = tb_ipkt;

  # COMB_DELAY;

  // update fifo write pointer
  tb_fifo_wrp = tb_fifo_wrp + 1;

  # COMB_DELAY;

  forever
  begin

    if (tb_ipkt_send_eop)
    begin
      // send end-of-packet to uut
      uut_ispl_data = encode_nrz_2of7 (`EOP, tb_old_data);

      // generate next packet
      tb_ipkt_type = tb_ipkt_type + 1;
      tb_ipkt_key = tb_ipkt_key + 1;
      tb_ipkt_pld = tb_ipkt_pld + 1;
      if (tb_pkt_cnt > 13) tb_ipkt_send_pld = 1;

      # COMB_DELAY;

      // store new input packet in fifo for later checking
      tb_fifo_pkt[tb_fifo_wrp] = tb_ipkt;

      # COMB_DELAY;

      // update packet fifo write pointer
      tb_fifo_wrp <= tb_fifo_wrp + 1;

      // initialize flit counter
      tb_flt_cnt = 0;
    end
    else
    begin
      // send next flit to uut
      uut_ispl_data = encode_nrz_2of7 (tb_ipkt[(tb_flt_cnt * 4) +: 4],
                                        tb_old_data
                                      );

      # COMB_DELAY;

      // update flit counter
      tb_flt_cnt = tb_flt_cnt + 1;
    end

    // remember data (for nrz encoding)
    tb_old_data = uut_ispl_data;

    // remember ack (for transition detection)
    tb_old_ack = uut_ispl_ack;

    // wait for ack from uut
    wait (tb_old_ack != uut_ispl_ack);

    # SPL_HSDLY;  // handshake delay
  end
end


//--------------------------------------------------
// testbench: assemble input packet
//--------------------------------------------------
assign tb_ipkt = {tb_ipkt_pld, tb_ipkt_key, tb_ipkt_hdr};


//--------------------------------------------------
// testbench: generate input packet header
//--------------------------------------------------
assign tb_ipkt_hdr = {tb_ipkt_type, 4'b0000, tb_ipkt_send_pld, tb_ipkt_prty};


//--------------------------------------------------
// testbench: generate input packet parity bit
//--------------------------------------------------
assign tb_ipkt_prty = tb_ipkt_send_pld
                        ? ^(tb_ipkt_hdr[7:1] ^ tb_ipkt_key ^ tb_ipkt_pld)
                        : ~(^(tb_ipkt_hdr[7:1] ^ tb_ipkt_key));


//--------------------------------------------------
// testbench: time to send end-of-packet
//--------------------------------------------------
assign tb_ipkt_send_eop = (!tb_ipkt_hdr[1] && (tb_flt_cnt == 10))
                            || (tb_flt_cnt == 18);


//--------------------------------------------------
// packet interface: generate pkt ready
//--------------------------------------------------
always @ (posedge tb_clk or posedge tb_rst)
  if (tb_rst)
    uut_opkt_rdy <= # COMB_DELAY 1'b0;
  else
    if (uut_opkt_vld && uut_opkt_rdy && (tb_start_cnt != 0))
      uut_opkt_rdy <= # COMB_DELAY 1'b0;
    else if (tb_cycle_cnt == 0)
      uut_opkt_rdy <= # COMB_DELAY 1'b1;


always @ (posedge tb_clk or posedge tb_rst)
  if (tb_rst)
    tb_pkt_cnt <= 0;
  else
    if (uut_opkt_vld && uut_opkt_rdy)
      tb_pkt_cnt <= tb_pkt_cnt + 1;


always @ (posedge tb_clk or posedge tb_rst)
  if (tb_rst)
    tb_start_cnt <= 200;
  else
    if (tb_pkt_cnt <= 25)
    begin
      if (uut_opkt_vld && uut_opkt_rdy)
        if (tb_start_cnt <= 75)
          tb_start_cnt <= 200;
        else  
          tb_start_cnt <= tb_start_cnt - 15;
    end
    else
      tb_start_cnt <= 0;


always @ (*)
  if (uut_opkt_vld && uut_opkt_rdy && (tb_start_cnt != 0))
    tb_cnt_on = 1;
  else
    tb_cnt_on = 0;


//--------------------------------------------------
// testbench: detect received packet fields
//--------------------------------------------------
assign tb_opkt_hdr = uut_opkt_data[`PKT_HDR_RNG];
assign tb_opkt_key = uut_opkt_data[`PKT_KEY_RNG];
assign tb_opkt_pld = uut_opkt_data[`PKT_PLD_RNG];


//--------------------------------------------------
// testbench: check received packet
//--------------------------------------------------
//TODO: assumes fifo is never empty (safe for now!)
always @ (posedge tb_clk)
  if (uut_opkt_vld && uut_opkt_rdy)
    if ((tb_opkt_hdr !== tb_fifo_pkt[tb_fifo_rdp][`PKT_HDR_RNG])
         || (tb_opkt_key !== tb_fifo_pkt[tb_fifo_rdp][`PKT_KEY_RNG])
         || (tb_fifo_pkt[tb_fifo_rdp][1]
              && (tb_opkt_pld !== tb_fifo_pkt[tb_fifo_rdp][`PKT_PLD_RNG])
            )
       )
      tb_bad_pkt = 1;
    else
      tb_bad_pkt = 0;


//--------------------------------------------------
// testbench: update pakect fifo read pointer
//--------------------------------------------------
always @ (posedge tb_clk or posedge tb_rst)
  if (tb_rst)
    tb_fifo_rdp <= 0;
  else
    if (uut_opkt_vld && uut_opkt_rdy)
      tb_fifo_rdp <= tb_fifo_rdp + 1;


//--------------------------------------------------
// testbench: multi-purpose cycle counter
//--------------------------------------------------
always @ (posedge tb_clk or posedge tb_rst)
  if (tb_rst)
    tb_cycle_cnt <= 0;
  else
    if (tb_cnt_on)
      tb_cycle_cnt <= tb_start_cnt;
    else if (tb_cycle_cnt != 0)
      tb_cycle_cnt <= tb_cycle_cnt - 1;


//--------------------------------------------------
// unit under test: reset signal
//--------------------------------------------------
initial
begin
  uut_rst = 1'b0;

  // wait a few clock cycles before triggering the reset signal
  # INIT_DELAY;
   
  uut_rst = 1'b1;

  # RST_DELAY;

  # COMB_DELAY;
   
  uut_rst = 1'b0;  // release uut
end


//--------------------------------------------------
// unit under test: clock signal
//--------------------------------------------------
initial
begin
   uut_clk = 1'b0;

  forever
  begin
    # UUT_CLK_HPER;
     
    uut_clk = ~uut_clk;
   end
end


//--------------------------------------------------
// testbench: reset signal
//--------------------------------------------------
initial
begin
  tb_rst  = 1'b0;

  // wait a few clock cycles before triggering the reset signal
  # INIT_DELAY;
   
  tb_rst  = 1'b1;

  # RST_DELAY;

  # COMB_DELAY;
   
  tb_rst  = 1'b0;  // release tb
end


//--------------------------------------------------
// testbench: clock signal
//--------------------------------------------------
initial
begin
   tb_clk = 1'b0;

  forever
  begin
    # TB_CLK_HPER;
     
    tb_clk = ~tb_clk;
   end
end
endmodule
