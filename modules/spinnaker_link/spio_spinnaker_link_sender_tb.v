// -------------------------------------------------------------------------
//  SpiNNaker link transmitter testbench module
//
// -------------------------------------------------------------------------
// AUTHOR
//  lap - luis.plana@manchester.ac.uk
//
// -------------------------------------------------------------------------
// COPYRIGHT
//  Copyright (c) The University of Manchester, 2012-2016.
//  SpiNNaker Project
//  Advanced Processor Technologies Group
//  School of Computer Science
// -------------------------------------------------------------------------
// TODO
// - test non-maximum bandwidth on both interfaces
// - function complete_nrz_2of7 may need to check for errors
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

 localparam SPL_HSDLY = 12;  // external link delay estimate
//!! localparam SPL_HSDLY = 16;  // external link delay estimate
//!!localparam SPL_HSDLY = 23;  // external link delay estimate (includes SpiNNaker)

localparam INIT_DELAY = (10 * TB_CLK_HPER);
localparam RST_DELAY  = (51 * TB_CLK_HPER);  // align with clock posedge

localparam COMB_DELAY = 2;

localparam BPP_DELAY = 20;

localparam BPP_UUT =  4'd6;
localparam BSF_UUT = 5'd17;
localparam BAF_UUT =  3'd2;


//---------------------------------------------------------------
// internal signals
//---------------------------------------------------------------
reg         tb_rst;
reg         tb_clk;

reg         uut_rst;
reg         uut_clk;

wire [71:0] uut_ipkt_data;
reg         uut_ipkt_vld;
wire        uut_ipkt_rdy;

wire  [6:0] uut_ospl_data;
reg         uut_ospl_ack;

wire  [7:0] tb_ipkt_hdr; 
reg   [1:0] tb_ipkt_type; 
wire        tb_ipkt_prty; 
reg  [31:0] tb_ipkt_key;
reg  [31:0] tb_ipkt_pld;
reg         tb_ipkt_send_pld;

reg   [6:0] tb_old_data;
wire  [3:0] tb_rec_data;
wire        tb_rec_eop;

reg   [4:0] tb_flt_cnt;

reg         tb_bad_pkt;

reg  [71:0] tb_opkt;
wire  [7:0] tb_opkt_hdr; 
wire [31:0] tb_opkt_key;
wire [31:0] tb_opkt_pld;

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
// NRZ 2-of-7 completion detection
//---------------------------------------------------------------
//TODO: may need to take errors into account
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
// unit under test
//---------------------------------------------------------------
spio_spinnaker_link_sender uut
(
  .CLK_IN           (uut_clk),
  .RESET_IN         (uut_rst),

  // link error interface
  .ACK_ERR_OUT      (),
  .TMO_ERR_OUT      (),

  // back-pressure point interface
  .BPP_IN           (BPP_UUT),
  .BSF_LONG_IN      (BSF_UUT),
  .BAF_LONG_IN      (BAF_UUT),

  // incoming packet interface
  .PKT_DATA_IN      (uut_ipkt_data),
  .PKT_VLD_IN       (uut_ipkt_vld),
  .PKT_RDY_OUT      (uut_ipkt_rdy),

  // outpoing SpiNNaker link interface
  .SL_DATA_2OF7_OUT (uut_ospl_data),
  .SL_ACK_IN        (uut_ospl_ack)
);


//--------------------------------------------------
// packet interface: generate pkt data
//--------------------------------------------------
assign uut_ipkt_data = {tb_ipkt_pld, tb_ipkt_key, tb_ipkt_hdr};


//--------------------------------------------------
// packet interface: generate pkt valid
//--------------------------------------------------
always @ (posedge tb_clk or posedge tb_rst)
  if (tb_rst)
    uut_ipkt_vld <= # COMB_DELAY 1'b0;
  else
    if (uut_ipkt_vld && uut_ipkt_rdy && (tb_start_cnt != 0))
      uut_ipkt_vld <= # COMB_DELAY 1'b0;
    else if (tb_cycle_cnt == 0)
      uut_ipkt_vld <= # COMB_DELAY 1'b1;


always @ (posedge tb_clk or posedge tb_rst)
  if (tb_rst)
    tb_pkt_cnt <= 0;
  else
    if (uut_ipkt_vld && uut_ipkt_rdy)
      tb_pkt_cnt <= tb_pkt_cnt + 1;


always @ (posedge tb_clk or posedge tb_rst)
  if (tb_rst)
    tb_start_cnt <= 200;
  else
    if (tb_pkt_cnt <= 25)
    begin
      if (uut_ipkt_vld && uut_ipkt_rdy)
        if (tb_start_cnt <= 75)
          tb_start_cnt <= 200;
        else  
          tb_start_cnt <= tb_start_cnt - 15;
    end
    else
      tb_start_cnt <= 0;


always @ (*)
  if (uut_ipkt_vld && uut_ipkt_rdy && (tb_start_cnt != 0))
    tb_cnt_on = 1;
  else
    tb_cnt_on = 0;



//--------------------------------------------------
// testbench: generate input packet header
//--------------------------------------------------
assign tb_ipkt_hdr = {tb_ipkt_type, 4'b0000, tb_ipkt_send_pld, tb_ipkt_prty};


//--------------------------------------------------
// testbench: generate input packet type
//--------------------------------------------------
always @ (posedge tb_clk or posedge tb_rst)
  if (tb_rst)
  begin
    tb_ipkt_type <= 0;
  end
  else
    if (uut_ipkt_vld && uut_ipkt_rdy)
      tb_ipkt_type <= # COMB_DELAY tb_ipkt_type + 1;


//--------------------------------------------------
// testbench: generate input packet parity bit
//--------------------------------------------------
assign tb_ipkt_prty = tb_ipkt_send_pld
                        ? ^(tb_ipkt_hdr[7:1] ^ tb_ipkt_key ^ tb_ipkt_pld)
                        : ~(^(tb_ipkt_hdr[7:1] ^ tb_ipkt_key));


//--------------------------------------------------
// testbench: generate input packet key
//--------------------------------------------------
always @ (posedge tb_clk or posedge tb_rst)
  if (tb_rst)
  begin
    tb_ipkt_key <= 32'h0000_0001;
  end
  else
    if (uut_ipkt_vld && uut_ipkt_rdy)
      tb_ipkt_key <= # COMB_DELAY tb_ipkt_key + 1;


//--------------------------------------------------
// testbench: generate input packet payload
//--------------------------------------------------
//TODO: currently no payload sent!
always @ (posedge tb_clk or posedge tb_rst)
  if (tb_rst)
    tb_ipkt_pld <= 32'ha5a5_a5a5;
  else
    if (uut_ipkt_vld && uut_ipkt_rdy)
      tb_ipkt_pld <= # COMB_DELAY tb_ipkt_pld + 1;


//--------------------------------------------------
// testbench: control if payload is sent
//--------------------------------------------------
//TODO: currently no payload sent!
always @ (posedge tb_clk or posedge tb_rst)
  if (tb_rst)
    tb_ipkt_send_pld <= 1'b0;
  else
    if (tb_pkt_cnt >= 12)
      tb_ipkt_send_pld <= 1'b0;
    else if (tb_pkt_cnt >= 6)
      tb_ipkt_send_pld <= 1'b1;


//--------------------------------------------------
// testbench: store sent packets in fifo for later checking
//--------------------------------------------------
//TODO: assumes that fifo is never full (safe for now!)
always @ (posedge tb_clk)
  if (uut_ipkt_vld && uut_ipkt_rdy)
    tb_fifo_pkt[tb_fifo_wrp] <= uut_ipkt_data;


//--------------------------------------------------
// testbench: update pakect fifo write pointer
//--------------------------------------------------
always @ (posedge tb_clk or posedge tb_rst)
  if (tb_rst)
    tb_fifo_wrp <= 0;
  else
    if (uut_ipkt_vld && uut_ipkt_rdy)
      tb_fifo_wrp <= tb_fifo_wrp + 1;


//--------------------------------------------------
// SpiNNaker link interface: generate ack
//--------------------------------------------------
//TODO: test different (maybe time-varying) asynchronous delays!
initial
begin
  wait (tb_rst);   // wait for reset to be applied

  # COMB_DELAY;
   
  uut_ospl_ack = 0;
  tb_old_data = 0;

  tb_bad_pkt = 0;
  tb_fifo_rdp = 0;

  tb_opkt = 72'hxxxxxxxx_xxxxxxxx_xx;
  tb_flt_cnt = 0;

  wait (!tb_rst);  // wait for reset to be released

  # COMB_DELAY;
   
  uut_ospl_ack = 1;  // initial ack (mimic SpiNNaker behaviour)
  tb_old_data = uut_ospl_data;  // remember data (for nrz completion detection)

  forever
  begin
    wait (complete_nrz_2of7 (uut_ospl_data, tb_old_data));

    # COMB_DELAY;

    if (tb_rec_eop)
    begin
      // check received packet
      if ((tb_opkt_hdr !== tb_fifo_pkt[tb_fifo_rdp][`PKT_HDR_RNG])
           || (tb_opkt_key !== tb_fifo_pkt[tb_fifo_rdp][`PKT_KEY_RNG])
           || (tb_fifo_pkt[tb_fifo_rdp][1]
                && (tb_opkt_pld !== tb_fifo_pkt[tb_fifo_rdp][`PKT_PLD_RNG])
              )
         )
        tb_bad_pkt = 1;
      else
        tb_bad_pkt = 0;

      // update fifo read pointer
      tb_fifo_rdp = tb_fifo_rdp + 1;
      
      tb_opkt = 72'hxxxxxxxx_xxxxxxxx_xx;
      tb_flt_cnt = 0;
    end
    else
    begin
      tb_opkt[(tb_flt_cnt * 4) +: 4] = tb_rec_data;
      tb_flt_cnt = tb_flt_cnt + 1;
    end

    tb_old_data = uut_ospl_data;

    # SPL_HSDLY;
    if (tb_flt_cnt == BPP_UUT) # BPP_DELAY;
//!    if ((tb_flt_cnt != BPP_UUT) || (tb_pkt_cnt <= 10)) uut_ospl_ack = ~uut_ospl_ack;
    uut_ospl_ack = ~uut_ospl_ack;
  end
end


//--------------------------------------------------
// testbench: decode data and end-of-packet
//--------------------------------------------------
assign tb_rec_data = decode_nrz_2of7 (uut_ospl_data, tb_old_data);
assign tb_rec_eop  = eop_nrz_2of7 (uut_ospl_data, tb_old_data);


//--------------------------------------------------
// testbench: detect packet fields
//--------------------------------------------------
assign tb_opkt_hdr = tb_opkt[`PKT_HDR_RNG];
assign tb_opkt_key = tb_opkt[`PKT_KEY_RNG];
assign tb_opkt_pld = tb_opkt[`PKT_PLD_RNG];


//--------------------------------------------------
// testbench: multi-purpose cycle counter
//--------------------------------------------------
always @ (posedge tb_clk or posedge tb_rst)
  if (tb_rst)
    tb_cycle_cnt <= 200;
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
