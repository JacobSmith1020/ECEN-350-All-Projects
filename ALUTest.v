`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   10:02:47 03/05/2009
// Design Name:   ALU
// Module Name:   E:/350/Lab8/ALU/ALUTest.v
// Project Name:  ALU
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: ALU
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

`define STRLEN 32
module ALUTest_v;

	task passTest;
		input [64:0] actualOut, expectedOut;
		input [`STRLEN*8:0] testType;
		inout [7:0] passed;
	
		if(actualOut == expectedOut) begin $display ("%s passed", testType); passed = passed + 1; end
		else $display ("%s failed: %x should be %x", testType, actualOut, expectedOut);
	endtask
	
	task allPassed;
		input [7:0] passed;
		input [7:0] numTests;
		
		if(passed == numTests) $display ("All tests passed");
		else $display("Some tests failed");
	endtask

	// Inputs
	reg [63:0] BusA;
	reg [63:0] BusB;
	reg [3:0] ALUCtrl;
	reg [7:0] passed;

	// Outputs
	wire [63:0] BusW;
	wire Zero;

	// Instantiate the Unit Under Test (UUT)
	ALU uut (
		.BusW(BusW), 
		.Zero(Zero), 
		.BusA(BusA), 
		.BusB(BusB), 
		.ALUCtrl(ALUCtrl)
	);

	initial begin
		// Initialize Inputs
		BusA = 0;
		BusB = 0;
		ALUCtrl = 0;
		passed = 0;

                // Here is one example test vector, testing the ADD instruction:
		{BusA, BusB, ALUCtrl} = {64'h1234, 64'hABCD0000, 4'h2}; #40; passTest({Zero, BusW}, 65'h0ABCD1234, "ADD 0x1234,0xABCD0000", passed);
		//Reformate and add your test vectors from the prelab here following the example of the testvector above.	
      
      
      {BusA, BusB, ALUCtrl} = {64'h4500, 64'h2000, 4'h2}; #40; passTest({Zero, BusW}, 65'h6500, "ADD1", passed);
      {BusA, BusB, ALUCtrl} = {64'h25, 64'h15, 4'h2}; #40; passTest({Zero, BusW}, 65'h3A, "ADD2", passed);
      {BusA, BusB, ALUCtrl} = {64'h10001, 64'h182165, 4'h2}; #40; passTest({Zero, BusW}, 65'h192166, "ADD3", passed);
      
      
      {BusA, BusB, ALUCtrl} = {64'h1, 64'h1, 4'h0}; #40; passTest({Zero, BusW}, 65'h1, "AND1", passed);
      {BusA, BusB, ALUCtrl} = {64'h1, 64'h0, 4'h0}; #40; passTest( BusW, 65'h0, "AND2", passed);
      {BusA, BusB, ALUCtrl} = {64'h112, 64'h65, 4'h0}; #40; passTest(BusW, 65'h0, "AND3", passed);
      
      
      {BusA, BusB, ALUCtrl} = {64'h1, 64'h1, 4'h1}; #40; passTest({Zero, BusW}, 65'h1, "OR1", passed);
      {BusA, BusB, ALUCtrl} = {64'h1, 64'h0, 4'h1}; #40; passTest({Zero, BusW}, 65'h1, "OR2", passed);
      {BusA, BusB, ALUCtrl} = {64'h112, 64'h65, 4'h1}; #40; passTest({Zero, BusW}, 65'h177, "OR3", passed);
      
      
      {BusA, BusB, ALUCtrl} = {64'h4500, 64'h2000, 4'h6}; #40; passTest({Zero, BusW}, 65'h2500, "SUB1", passed);
      {BusA, BusB, ALUCtrl} = {64'h25, 64'h15, 4'h6}; #40; passTest({Zero, BusW}, 65'h10, "SUB2", passed);
      {BusA, BusB, ALUCtrl} = {64'h5, 64'h4, 4'h6}; #40; passTest({Zero, BusW}, 65'h1, "SUB3", passed);
      
      
      {BusA, BusB, ALUCtrl} = {64'h1100, 64'h1100, 4'h7}; #40; passTest({Zero, BusW}, 65'h1100, "PASSB1", passed);
      {BusA, BusB, ALUCtrl} = {64'h15, 64'h87, 4'h7}; #40; passTest({Zero, BusW}, 65'h87, "PASSB2", passed);
      {BusA, BusB, ALUCtrl} = {64'h225, 64'h225, 4'h7}; #40; passTest({Zero, BusW}, 65'h225, "PASSB3", passed);
      
      
      {BusA, BusB, ALUCtrl} = {64'h181, 64'h2, 4'h3}; #40; passTest({Zero, BusW}, 65'h604, "LSL1", passed);
      {BusA, BusB, ALUCtrl} = {64'h229, 64'h5, 4'h3}; #40; passTest({Zero, BusW}, 65'h4520, "LSL2", passed);
      {BusA, BusB, ALUCtrl} = {64'h115000, 64'h3, 4'h3}; #40; passTest({Zero, BusW}, 65'h8A8000, "LSL3", passed);
      
      
      {BusA, BusB, ALUCtrl} = {64'h181, 64'h2, 4'h4}; #40; passTest({Zero, BusW}, 65'h60, "LSR1", passed);
      {BusA, BusB, ALUCtrl} = {64'h229, 64'h5, 4'h4}; #40; passTest({Zero, BusW}, 65'h11, "LSR2", passed);
      {BusA, BusB, ALUCtrl} = {64'h115000, 64'h3, 4'h4}; #40; passTest({Zero, BusW}, 65'h22A00, "LSR3", passed);


		allPassed(passed, 22);
	end
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars(1);
  end
      
endmodule

