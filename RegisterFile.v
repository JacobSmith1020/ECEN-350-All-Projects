`timescale 1ns / 1ps
module RegisterFile(BusA, BusB, BusW, RA, RB, RW, RegWr, Clk);
  output [63:0] BusA;
  output [63:0] BusB;
  input [63:0] BusW;
  input [4:0]RW;
  input [4:0]RA;
  input [4:0]RB;
  input RegWr;
  input Clk;
  reg [31:0] registers [63:0];

  assign #2 BusA = registers[RA];
  assign #2 BusB = registers[RB];
  
  always@(Clk)//every Clk cycle, reset reg 31 to 0
    begin
      registers[31] <= #3 1'd0;
    end
  

  always@(negedge Clk) 
    begin
      if(RegWr)//if RegWr == 1
        if(RW != 5'd31)//if RW is not reg 31, write a new value to it
          registers[RW] <= #3 BusW;
      	else
          registers[31] <= #3 1'd0;//else if RW is reg 31, write a 0 to it
    end
    
endmodule