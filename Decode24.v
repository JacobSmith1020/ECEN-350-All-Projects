`timescale 1ns / 1ps

module Decode24(out, in);
  input [1:0] in;
  output [3:0] out;
  reg [3:0] out;//need reg to assign values to out via '='
  
  //when input changes, initiate always block
  always@(in)
    //check each case of input and direct to correct output
    case(in)
      2'b00: out = 4'b0001;
      2'b01: out = 4'b0010;
      2'b10: out = 4'b0100;
      2'b11: out = 4'b1000;
    endcase
endmodule