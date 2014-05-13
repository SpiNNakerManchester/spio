`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// From 2 of 7 codes, assemble a SpiNNaker packet as a 72bit word,
// Drop bad packets
// Author J Pepper
// Date 13/07/2012
//////////////////////////////////////////////////////////////////////////////////
module spio_spinnaker_link_packet_con(
    input clk,
    input reset,
    input fifo_full,
    input  [6:0] code2of7,
    output reg [71:0] packet,
    output reg ack,
    output reg fifo_write
    );

reg [6:0] old_code2of7;     //Previous code2of7 symbol 
reg [3:0] nibble;           //Decoded valid value of 2of7code symbol
reg [4:0] symbol_count;     //Used to keep track of number of symbols in a packet
reg       eop;              //Flag to indicate end of packet symbol
reg       oob;              //Flag to indicate out of bounds symbol
reg       tom;              //Flag to indicate symbol detected  
reg       long_packet;      //Flag to indicate that packet should be long

reg [2:0] state;

initial                     //Initial power up values
 begin
  state = 0;
  old_code2of7 = 0;
  ack = 0;
  fifo_write = 0;
  packet = 0;
  symbol_count = 0;
  long_packet = 0;
 end

always @(posedge clk or posedge reset)
 if(reset == 1)            //Reset values  
  begin
   state <= 0;
   old_code2of7 <= 0;
   ack <= 0;
   fifo_write <= 0;
   packet <= 0;
   symbol_count <= 0;
   long_packet <= 0;
  end
 else 
  case(state)
   0, 1 : begin                                                      
           state <= state +1;  ack <= 0; old_code2of7 <= 0;          // Wait for the synchronizer on the input to be flushed before reading the input code2of7
          end  
   2 : begin
        state <= 3; ack <= ~ack; old_code2of7 <= code2of7;           // Read the current 2of7code, send an ack to clear the link
       end  
   3 : begin                                                         // Packet assembly state. Assemble and write good spinnaker packets, throw away bad packets
        if(eop == 1)                                                 // EOP symbol detected, now check packet is correct size and write this packet
         begin
          if(((symbol_count == 10) && (long_packet == 0)) ||         // Packet good
             ((symbol_count == 18) && (long_packet == 1))) 
           begin
            if(fifo_full != 1)                                       // Wait for space to store packet then write it
             begin 
              ack <= ~ack; old_code2of7 <= code2of7;
              symbol_count <= 0; fifo_write <= 1;
             end
           end                                                                                         
          else                                                       // Packet bad, don't store this packet
           begin
            ack <= ~ack; old_code2of7 <= code2of7;
            symbol_count <= 0; fifo_write <= 0; 
           end
         end
        else if(oob == 1)                                            // OOB symbol detected, throw packet away and move to resync state
         begin
          ack <= ~ack; old_code2of7 <= code2of7;
          symbol_count <= 0; fifo_write <= 0;
          state <= 4; 
         end
        else if(tom == 1)                                            // Valid symbol detected
         begin 
          ack <= ~ack; old_code2of7 <= code2of7;
          symbol_count <= symbol_count +1; fifo_write <= 0;
          packet <= packet_order(symbol_count, nibble, packet);      // Put nibble into the packet packaging process
          if(symbol_count == 0) long_packet <= nibble[1];            // Set long flag if packet is long type
          if(((symbol_count >= 10) && (long_packet == 0)) ||           // Packet too long, enter resync state
            (symbol_count >= 18))  state <= 4;                        
         end
        else fifo_write <= 0;
       end
   4 : begin                                                         // Resync state
        if(eop == 1)                                                 // Sync to EOP symbol before moving to packet assembly state
         begin
          ack <= ~ack; old_code2of7 <= code2of7;
          symbol_count <= 0; fifo_write <= 0;
          state <= 3;
         end
        else if(tom == 1) 
         begin 
          ack <= ~ack; old_code2of7 <= code2of7;
          symbol_count <= 0; fifo_write <= 0;
         end
       end

  endcase



// Convert 2 of 7 codes to nibble, end of packet, out of bounds, and 2 or more bits changed
always @(code2of7 or old_code2of7)
   begin
    case(code2of7 ^ old_code2of7)
     7'b0010001 : begin nibble = 0;   eop = 0; oob = 0; tom = 1; end // 0
     7'b0010010 : begin nibble = 1;   eop = 0; oob = 0; tom = 1; end // 1
     7'b0010100 : begin nibble = 2;   eop = 0; oob = 0; tom = 1; end // 2
     7'b0011000 : begin nibble = 3;   eop = 0; oob = 0; tom = 1; end // 3
     7'b0100001 : begin nibble = 4;   eop = 0; oob = 0; tom = 1; end // 4
     7'b0100010 : begin nibble = 5;   eop = 0; oob = 0; tom = 1; end // 5
     7'b0100100 : begin nibble = 6;   eop = 0; oob = 0; tom = 1; end // 6
     7'b0101000 : begin nibble = 7;   eop = 0; oob = 0; tom = 1; end // 7
     7'b1000001 : begin nibble = 8;   eop = 0; oob = 0; tom = 1; end // 8
     7'b1000010 : begin nibble = 9;   eop = 0; oob = 0; tom = 1; end // 9
     7'b1000100 : begin nibble = 10;  eop = 0; oob = 0; tom = 1; end // 10
     7'b1001000 : begin nibble = 11;  eop = 0; oob = 0; tom = 1; end // 11
     7'b0000011 : begin nibble = 12;  eop = 0; oob = 0; tom = 1; end // 12
     7'b0000110 : begin nibble = 13;  eop = 0; oob = 0; tom = 1; end // 13
     7'b0001100 : begin nibble = 14;  eop = 0; oob = 0; tom = 1; end // 14
     7'b0001001 : begin nibble = 15;  eop = 0; oob = 0; tom = 1; end // 15
     7'b1100000 : begin nibble = 4'hx;   eop = 1; oob = 0; tom = 1; end // EOP symbol
     0, 1, 2, 4,
     8, 16, 32,
     64         : begin nibble = 4'hx;  eop = 0; oob = 0; tom = 0; end //  
     default    : begin nibble = 4'hx;  eop = 0; oob = 1; tom = 1; end // OOB symbol
    endcase
   end    

// Define functions
// Put nibbles in the correct order
// Uses a shift rather than a demux, I think this is quicker
function  [71:0] packet_order ;
  input   [4:0]  symbol_count;
  input   [3:0]  nibble;
  input   [71:0] packet;
  begin
   if(symbol_count > 9) //pack payload into packet type long
    packet_order = {nibble, packet[71:44], packet[39:0]};
   else //pack header and routing key into packet
    packet_order = {packet[71:40], nibble, packet[39:4]};
 end
endfunction;

endmodule
