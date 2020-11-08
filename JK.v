`timescale 1ns / 1ps
module JK(out, j, k, clk, reset);

  //declare variables
  output out;
  input j, k;
  input clk;
  input reset;
  wire notreset;
  wire tempout0, tempout1, notout, wireout;
  
  //gate level calculations
  not INV1(notreset, reset);
  nand #2 NAND1(tempout0, j, clk, notout);
  nand #2 NAND2(tempout1, k, clk, wireout);
  nand #2 NAND3(wireout, tempout0, notout);
  nand #2 NAND4(notout, tempout1, wireout, notreset);
  
  //assign the correct value to output variable
  assign out = wireout;
endmodule