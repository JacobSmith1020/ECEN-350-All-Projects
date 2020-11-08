`timescale 1ns / 1ps
module NextPClogic(NextPC, CurrentPC, SignExtImm64, Branch, ALUZero, UCbranch); 
  input [63:0] CurrentPC, SignExtImm64; 
  input Branch, ALUZero, UCbranch; 
  output reg [63:0] NextPC;
  reg [63:0]SignExtImm64SL;
  
  //mux section
  always @ (*) begin
    SignExtImm64SL <= #1 SignExtImm64 * 4;
  	if(UCbranch)
  		NextPC <= #2 CurrentPC + SignExtImm64SL;
    else if(Branch && ALUZero)
    	NextPC <= #2 CurrentPC + SignExtImm64SL;
    else 
    	NextPC <= #2 CurrentPC + 4;
  end
endmodule