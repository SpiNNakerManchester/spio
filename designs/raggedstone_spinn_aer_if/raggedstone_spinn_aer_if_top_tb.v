// -------------------------------------------------------------------------
//  testbench for bidirectional SpiNNaker link to AER device interface
//
// -------------------------------------------------------------------------
// AUTHOR
//  lap - luis.plana@manchester.ac.uk
//  Based on work by J Pepper (Date 08/08/2012)
//
// -------------------------------------------------------------------------
// Taken from:
// https://solem.cs.man.ac.uk/svn/spinn_aer2_if/spinn_aer2_if_tb.v
// Revision 2615 (Last-modified date: 2013-10-02 11:39:58 +0100)
//
// -------------------------------------------------------------------------
// COPYRIGHT
//  Copyright (c) The University of Manchester, 2012-2017.
//  SpiNNaker Project
//  Advanced Processor Technologies Group
//  School of Computer Science
// -------------------------------------------------------------------------
// TODO
// -------------------------------------------------------------------------

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`timescale 1ns / 1ps
module raggedstone_spinn_aer_if_top_tb ();
//---------------------------------------------------------------
// constants
//---------------------------------------------------------------
localparam CLK_HPER  = (31.25 / 2);

localparam IAER_HSDLY = 100;
localparam OAER_HSDLY = 150;
localparam SPL_HSDLY = 8;

// debouncer constants
localparam DBNCER_CONST = 20'hfffff;
localparam RST_DELAY    = (35 * DBNCER_CONST);
localparam INIT_DELAY   = (10 * CLK_HPER);


//---------------------------------------------------------------
// internal signals
//---------------------------------------------------------------
reg         nreset;
reg         rst_tb;
reg         clk;

reg   [6:0] ispl_data;
wire        ispl_ack;

wire  [6:0] ospl_data;
reg         ospl_ack;

reg  [15:0] iaer_data;
reg         iaer_req;
wire        iaer_ack;

wire [15:0] oaer_data;
wire        oaer_req;
reg         oaer_ack;

wire        dump_mode;


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//--------------------------- functions -------------------------
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

//---------------------------------------------------------------
// RTZ 2-of-7 decoder
//---------------------------------------------------------------
function [3:0] decode_2of7 ;
  input [6:0] data;

  case (data)
    7'b0010001: decode_2of7 = 0;    // 0
    7'b0010010: decode_2of7 = 1;    // 1
    7'b0010100: decode_2of7 = 2;    // 2
    7'b0011000: decode_2of7 = 3;    // 3
    7'b0100001: decode_2of7 = 4;    // 4
    7'b0100010: decode_2of7 = 5;    // 5
    7'b0100100: decode_2of7 = 6;    // 6
    7'b0101000: decode_2of7 = 7;    // 7
    7'b1000001: decode_2of7 = 8;    // 8
    7'b1000010: decode_2of7 = 9;    // 9
    7'b1000100: decode_2of7 = 10;   // 10
    7'b1001000: decode_2of7 = 11;   // 11
    7'b0000011: decode_2of7 = 12;   // 12
    7'b0000110: decode_2of7 = 13;   // 13
    7'b0001100: decode_2of7 = 14;   // 14
    7'b0001001: decode_2of7 = 15;   // 15
    default:    decode_2of7 = 4'hx; // eop, incomplete, oob
  endcase
endfunction
//---------------------------------------------------------------

//---------------------------------------------------------------
// RTZ 2-of-7 completion detection
//---------------------------------------------------------------
function [3:0] complete_2of7 ;
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
                complete_2of7 = 1;
    default:    complete_2of7 = 0;
  endcase
endfunction
//---------------------------------------------------------------

//---------------------------------------------------------------
// RTZ 2-of-7 enoder
//---------------------------------------------------------------
function [6:0] encode_2of7 ;
  input [4:0] data;

  casex (data)
    5'b00000 : encode_2of7 = 7'b0010001; // 0
    5'b00001 : encode_2of7 = 7'b0010010; // 1
    5'b00010 : encode_2of7 = 7'b0010100; // 2
    5'b00011 : encode_2of7 = 7'b0011000; // 3
    5'b00100 : encode_2of7 = 7'b0100001; // 4
    5'b00101 : encode_2of7 = 7'b0100010; // 5
    5'b00110 : encode_2of7 = 7'b0100100; // 6
    5'b00111 : encode_2of7 = 7'b0101000; // 7
    5'b01000 : encode_2of7 = 7'b1000001; // 8
    5'b01001 : encode_2of7 = 7'b1000010; // 9
    5'b01010 : encode_2of7 = 7'b1000100; // 10
    5'b01011 : encode_2of7 = 7'b1001000; // 11
    5'b01100 : encode_2of7 = 7'b0000011; // 12
    5'b01101 : encode_2of7 = 7'b0000110; // 13
    5'b01110 : encode_2of7 = 7'b0001100; // 14
    5'b01111 : encode_2of7 = 7'b0001001; // 15
    5'b1xxxx : encode_2of7 = 7'b1100000; // EOP
    default  : encode_2of7 = 7'bxxxxxxx;
  endcase
endfunction
//---------------------------------------------------------------
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//--------------------------- datapath --------------------------
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
raggedstone_spinn_aer_if_top
#(
  .DBNCER_CONST (DBNCER_CONST)
) dut
(
  .ext_nreset               (nreset),
  .ext_clk                  (clk),

  .ext_mode_sel             (1'b1),
  .ext_7seg                 (),
  .ext_strobe               (),
  .ext_led2                 (),
  .ext_led3                 (),
  .ext_led4                 (dump_mode),

  .data_2of7_from_spinnaker (ispl_data),
  .ack_to_spinnaker         (ispl_ack),

  .data_2of7_to_spinnaker   (ospl_data),
  .ack_from_spinnaker       (ospl_ack),

  .iaer_data                (iaer_data),
  .iaer_req                 (iaer_req),
  .iaer_ack                 (iaer_ack),

  .oaer_data                (oaer_data),
  .oaer_req                 (oaer_req),
  .oaer_ack                 (oaer_ack)
);

//--------------------------------------------------
// drive the input AER interface
//--------------------------------------------------
initial
begin
  iaer_data = 0;
  iaer_req  = 1'b1;  // active LOW!

  wait (rst_tb);
  wait (!rst_tb);

  forever
  begin
    # IAER_HSDLY 
      iaer_req = 1'b0;

    wait (~iaer_ack);
    # IAER_HSDLY
      iaer_req = 1'b1;

    wait (iaer_ack);

    if (iaer_data == 15)
      iaer_data = 0;
    else
      iaer_data = iaer_data + 1;
  end
end
//--------------------------------------------------

//--------------------------------------------------
// drive the output AER interface
//--------------------------------------------------
initial
begin
  oaer_ack = 1'b1;  // active LOW!

  wait (rst_tb);
  wait (!rst_tb);

  forever
  begin
    wait (!oaer_req);  // active LOW
    # OAER_HSDLY
      oaer_ack = 1'b0;

    wait (oaer_req);
    # OAER_HSDLY
      oaer_ack = 1'b1;
  end
end
//--------------------------------------------------

//--------------------------------------------------
// drive the input SpiNNaker interface
//--------------------------------------------------
wire [39:0] packet;
reg  [15:0] pkt_data;
wire        parity;

reg         old_ack;

integer  data;
integer  count;

assign parity = ~(^pkt_data);
assign packet = {8'hff, 8'hff, pkt_data, 7'b0000000, parity};

initial
begin
  data  = 0;
  count = 0;

  pkt_data = data;

  ispl_data = 0;

  wait (rst_tb);
  wait (!rst_tb);

  forever
  begin
    # SPL_HSDLY
    ispl_data = ispl_data ^ encode_2of7 ({01'b0, packet[3:0]});

    old_ack = ispl_ack;
    wait (ispl_ack != old_ack);

    # SPL_HSDLY
    ispl_data = ispl_data ^ encode_2of7 ({01'b0, packet[7:4]});

    old_ack = ispl_ack;
    wait (ispl_ack != old_ack);

    # SPL_HSDLY
    ispl_data = ispl_data ^ encode_2of7 ({01'b0, packet[11:8]});

    old_ack = ispl_ack;
    wait (ispl_ack != old_ack);

    # SPL_HSDLY
    ispl_data = ispl_data ^ encode_2of7 ({01'b0, packet[15:12]});

    old_ack = ispl_ack;
    wait (ispl_ack != old_ack);

    # SPL_HSDLY
    ispl_data = ispl_data ^ encode_2of7 ({01'b0, packet[19:16]});

    old_ack = ispl_ack;
    wait (ispl_ack != old_ack);

    # SPL_HSDLY
    ispl_data = ispl_data ^ encode_2of7 ({01'b0, packet[23:20]});

    old_ack = ispl_ack;
    wait (ispl_ack != old_ack);

    # SPL_HSDLY
    ispl_data = ispl_data ^ encode_2of7 ({01'b0, packet[27:24]});

    old_ack = ispl_ack;
    wait (ispl_ack != old_ack);

    # SPL_HSDLY
    ispl_data = ispl_data ^ encode_2of7 ({01'b0, packet[31:28]});

    old_ack = ispl_ack;
    wait (ispl_ack != old_ack);

    # SPL_HSDLY
    ispl_data = ispl_data ^ encode_2of7 ({01'b0, packet[35:32]});

    old_ack = ispl_ack;
    wait (ispl_ack != old_ack);

    # SPL_HSDLY
    ispl_data = ispl_data ^ encode_2of7 ({01'b0, packet[39:36]});

    old_ack = ispl_ack;
    wait (ispl_ack != old_ack);

    # SPL_HSDLY
      ispl_data = ispl_data ^ encode_2of7 ({01'b1, 4'b0000});

    old_ack = ispl_ack;
    wait (ispl_ack != old_ack);

    data  = data  + 1;
    count = count + 1;

    if (count == 6)
      pkt_data = 16'hffff;  // turn in_mapper on!
    else
      pkt_data = data;
  end
end
//--------------------------------------------------

//--------------------------------------------------
// drive the output SpiNNaker interface
//--------------------------------------------------
reg  [6:0] old_data;
wire [3:0] rec_data;
wire       rec_eop;

integer so_count;

assign rec_data = decode_nrz_2of7 (ospl_data, old_data);
assign rec_eop  = eop_nrz_2of7 (ospl_data, old_data);

initial
begin
  so_count = 0;

  old_data = 0;
  ospl_ack = 0;

  wait (rst_tb);
  wait (!rst_tb);

  # SPL_HSDLY
    ospl_ack = 1;
  old_data = ospl_data;

  forever
  begin
    wait (complete_nrz_2of7 (ospl_data, old_data));
    # SPL_HSDLY
      ospl_ack = ~ospl_ack;
    old_data = ospl_data;

    so_count = so_count + 1;
    if (so_count == (21 * 11))
      # (100 * 11 * SPL_HSDLY);
  end
end
//--------------------------------------------------
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//---------------------------- control --------------------------
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
initial
begin
  nreset = 1'b1;
  rst_tb = 1'b1;

  // wait a few clock cycles before triggering nreset
  # INIT_DELAY
    nreset = 1'b0;

  # RST_DELAY
    nreset = 1'b1;

  # RST_DELAY 
    rst_tb = 1'b0;
end

initial
begin
   clk = 1'b0;

  forever
  begin
    # CLK_HPER
      clk = ~clk;
   end
end
endmodule
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
