`timescale 1ns / 1ps

module Mux21(out, in, sel);
  input [1:0] in;
  input sel;
  output out;
  wire out0, out1, out2;
  
  //compute internal calculations
  not INV1(out0, sel);
  and AND1(out1, out0, in[0]);
  and AND2(out2, sel, in[1]);
  
  //compute output
  or OR1(out, out1, out2);
endmodule