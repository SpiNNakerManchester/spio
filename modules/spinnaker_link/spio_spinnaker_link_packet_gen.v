`timescale 1ns / 1ps

module spio_spinnaker_link_packet_gen
(
    clk,
    reset,
    data_2of7,
    ack
//    gen_data
);

input         reset;
input         clk;
output [6:0]  data_2of7;
input         ack;
//input  [31:0] gen_data;


///////////////////////////////////////////////////////////////////
// 2 of 7 packet generator


// Packet generator state machine //

reg  [8:0]  gen_state;
reg  [6:0]  data_2of7;
reg  [4:0]  quibble;
reg  [39:0] packet;
reg         old_ack;

reg [15:0] cnt;
   

initial     gen_state = 0;
initial     data_2of7 = 0;
//#parameter   gen_data = 'h76543210;
parameter   gen_data = 32'hf5000000;
parameter   gen_header = 8'hb0;

always @(posedge clk or posedge reset)
 begin
 if(reset == 1)
  begin
   gen_state <= 0;
   data_2of7<= 0;
   cnt <= 0;
  end
 else
  case(gen_state)
        0 :   begin
//#	       packet <= {gen_data, 8'h0}; //create spinnaker packet
	       packet <= {(gen_data | cnt), gen_header}; //create spinnaker packet
	       gen_state <= gen_state + 1;
	      end
	1 :   begin
               quibble <= {1'b0, packet[3:1], ~(^packet[39:1])};         			 
	       gen_state <= gen_state + 1;
	      end
  	2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22
          :   begin
	       data_2of7 <= code_2of7_lut(quibble, data_2of7);     //transmit 2of7 code to spinnaker
	       packet <= packet >> 4;                              //shift the next packet nibble down	 
	       old_ack <= ack;
	       gen_state <= gen_state + 1;
	      end
        3, 5, 7, 9, 11, 13, 15, 17, 19 // set the nibble 
           :  begin
	       quibble[3:0] <= packet[3:0];
	       if(ack != old_ack) gen_state <= gen_state +1; //wait for ack from spinnaker
	      end
	21 :   begin // set eop
                quibble[4] <= 1;
	        if(ack != old_ack) gen_state <= gen_state +1;
	       end
	23 :  if(ack != old_ack)
              begin
                gen_state <= 0;
		cnt <= cnt + 1;
	      end 

   default:   ;
  endcase; 
 end


// Define functions

 function [6:0] code_2of7_lut ;
  input   [4:0] din;
  input   [6:0] code_2of7;

	casez(din)
		5'b00000 : code_2of7_lut = code_2of7 ^ 7'b0010001; // 0
		5'b00001 : code_2of7_lut = code_2of7 ^ 7'b0010010; // 1
		5'b00010 : code_2of7_lut = code_2of7 ^ 7'b0010100; // 2
		5'b00011 : code_2of7_lut = code_2of7 ^ 7'b0011000; // 3
		5'b00100 : code_2of7_lut = code_2of7 ^ 7'b0100001; // 4
		5'b00101 : code_2of7_lut = code_2of7 ^ 7'b0100010; // 5
		5'b00110 : code_2of7_lut = code_2of7 ^ 7'b0100100; // 6
		5'b00111 : code_2of7_lut = code_2of7 ^ 7'b0101000; // 7
		5'b01000 : code_2of7_lut = code_2of7 ^ 7'b1000001; // 8
		5'b01001 : code_2of7_lut = code_2of7 ^ 7'b1000010; // 9
		5'b01010 : code_2of7_lut = code_2of7 ^ 7'b1000100; // 10
		5'b01011 : code_2of7_lut = code_2of7 ^ 7'b1001000; // 11
		5'b01100 : code_2of7_lut = code_2of7 ^ 7'b0000011; // 12
		5'b01101 : code_2of7_lut = code_2of7 ^ 7'b0000110; // 13
		5'b01110 : code_2of7_lut = code_2of7 ^ 7'b0001100; // 14
		5'b01111 : code_2of7_lut = code_2of7 ^ 7'b0001001; // 15
		5'b1???? : code_2of7_lut = code_2of7 ^ 7'b1100000; // EOP
		default  : code_2of7_lut = 7'bxxxxxxx;
	endcase;
 endfunction;


endmodule

