`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   16:19:23 03/09/2009
// Design Name:   DataMemory
// Module Name:   E:/350/Lab9/DataMemory/DataMemoryTest.v
// Project Name:  DataMemory
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: DataMemory
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

`define STRLEN 32
module DataMemoryTest_v;


	task passTest;
		input [63:0] actualOut, expectedOut;
		input [`STRLEN*8:0] testType;
		inout [7:0] passed;
	
		if(actualOut == expectedOut) begin $display ("%s passed", testType); passed = passed + 1; end
		else $display ("%s failed: %d should be %d", testType, actualOut, expectedOut);
	endtask
	
	task allPassed;
		input [7:0] passed;
		input [7:0] numTests;
		
		if(passed == numTests) $display ("All tests passed");
		else $display("Some tests failed");
	endtask


	// Inputs
	reg [63:0] Address;
	reg [63:0] WriteData;
	reg MemoryRead;
	reg MemoryWrite;
	reg Clock;
	reg [7:0] passed;

	// Outputs
	wire [63:0] ReadData;

	// Instantiate the Unit Under Test (UUT)
	DataMemory uut (
		.ReadData(ReadData), 
		.Address(Address), 
		.WriteData(WriteData), 
		.MemoryRead(MemoryRead), 
		.MemoryWrite(MemoryWrite), 
		.Clock(Clock)
	);

	initial begin
		// Initialize Inputs
		Address = 0;
		WriteData = 0;
		MemoryRead = 0;
		MemoryWrite = 0;
		Clock = 0;
		passed = 0;

		// Add stimulus here
		$display("Init Memory with some useful data");
		#50 Clock = 1;{Address, WriteData, MemoryWrite, MemoryRead} = {64'h0, 64'h4, 2'h2};#50 Clock = 0;
		#50 Clock = 1;{Address, WriteData, MemoryWrite, MemoryRead} = {64'h4, 64'h3, 2'h2};#50 Clock = 0;
		#50 Clock = 1;{Address, WriteData, MemoryWrite, MemoryRead} = {64'h8, 64'd50, 2'h2};#50 Clock = 0;
		#50 Clock = 1;{Address, WriteData, MemoryWrite, MemoryRead} = {64'hc, 64'd40, 2'h2};#50 Clock = 0;
		#50 Clock = 1;{Address, WriteData, MemoryWrite, MemoryRead} = {64'h10, 64'd30, 2'h2};#50 Clock = 0;
		#50 Clock = 1;{Address, WriteData, MemoryWrite, MemoryRead} = {64'h14, 64'h0, 2'h2};#50 Clock = 0;
		#50 Clock = 1;{Address, WriteData, MemoryWrite, MemoryRead} = {64'h20, 64'h0, 2'h2};#50 Clock = 0;
		#50 Clock = 1;{Address, WriteData, MemoryWrite, MemoryRead} = {64'h1e, 64'h132, 2'h2};#50 Clock = 0;
		#50 Clock = 1;{Address, WriteData, MemoryWrite, MemoryRead} = {64'h20, 64'd16435934, 2'h2};#50 Clock = 0;
		#50 Clock = 1;{Address, WriteData, MemoryWrite, MemoryRead} = {64'h32, 64'haaaaffff, 2'h2};#50 Clock = 0;
		#50 Clock = 1;{Address, WriteData, MemoryWrite, MemoryRead} = {64'h33, 64'd1431699200, 2'h2};#50 Clock = 0;
		#50 Clock = 1;{Address, WriteData, MemoryWrite, MemoryRead} = {64'h3c, 64'hffff0000, 2'h2};#50 Clock = 0;

		#50 Clock = 1;
		
		{Address, WriteData, MemoryWrite, MemoryRead} = {64'h14, 64'hffff0000, 2'h1};
		#50 Clock = 0;
		#50 Clock = 1;
		passTest(ReadData, 64'h0, "Read address 0x14", passed);
		
		{Address, WriteData, MemoryWrite, MemoryRead} = {64'h3c, 64'hffff0000, 2'h1};
		#50 Clock = 0;
		#50 Clock = 1;
		passTest(ReadData, 64'hffff0000, "Read address 0xf0", passed);
		
		{Address, WriteData, MemoryWrite, MemoryRead} = {64'h33, 64'hffff0000, 2'h1};
		#50 Clock = 0;
		#50 Clock = 1;
		passTest(ReadData, 64'd1431699200, "Read address 0xcc", passed);
		
		{Address, WriteData, MemoryWrite, MemoryRead} = {64'h32, 64'hffff0000, 2'h1};
		#50 Clock = 0;
		#50 Clock = 1;
		passTest(ReadData, 64'haaaaffff, "Read address 0xc8", passed);
		
		{Address, WriteData, MemoryWrite, MemoryRead} = {64'hc, 64'hffff0000, 2'h1};
		#50 Clock = 0;
		#50 Clock = 1;
		passTest(ReadData, 64'd40, "Read address 0xc", passed);
		
		allPassed(passed, 5);

	end
	initial begin
		$dumpfile("dump.vcd");
		$dumpvars(1);
	end
      
endmodule

