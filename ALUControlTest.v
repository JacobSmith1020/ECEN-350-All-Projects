`timescale 1ns / 1ps

`define STRLEN 32
module ALUControlTest_v;


	task passTest;
      input [3:0] actualOut, expectedOut;
      input [`STRLEN*8:0] testType;
      inout [8:0] passed;
	
		if(actualOut == expectedOut) begin $display ("%s passed", testType); passed = passed + 1; end
		else $display ("%s failed: %d should be %d", testType, actualOut, expectedOut);
	endtask
	
	task allPassed;
      input [8:0] passed;
      input [8:0] numTests;
		
		if(passed == numTests) $display ("All tests passed");
		else $display("Some tests failed");
	endtask


	// Inputs
    reg [1:0]ALUop;
    reg [10:0]Opcode;
  	reg [8:0]passed;

	// Outputs
	wire [3:0]ALUCtrl;

	// Instantiate the Unit Under Test (UUT)
	ALUControl uut (
      .ALUop(ALUop),
      .Opcode(Opcode),
      .ALUCtrl(ALUCtrl)    
	);

	initial begin
      // Initialize Inputs
      ALUop = 0;
      Opcode = 0;
      passed = 0;

      
      {ALUop, Opcode} = {2'b00, 11'b11001011000};
      #50;
      $display ("ALUop = 00, Opcode = 11001011000, ALUCtrl = %b", ALUCtrl);
      passTest(ALUCtrl, 4'b0010, "Test case 1", passed);
      
      {ALUop, Opcode} = {2'b01, 11'b11001011000};
      #50;
      $display ("ALUop = 01, Opcode = 11001011000, ALUCtrl = %b", ALUCtrl);
      passTest(ALUCtrl, 4'b0111, "Test case 2", passed);
      
      {ALUop, Opcode} = {2'b10, 11'b11001011000};
      #50;
      $display ("ALUop = 10, Opcode = 11001011000, ALUCtrl = %b", ALUCtrl);
      passTest(ALUCtrl, 4'b0110, "Test case 3", passed);
      
      {ALUop, Opcode} = {2'b10, 11'b10101010000};
      #50;
      $display ("ALUop = 10, Opcode = 10101010000, ALUCtrl = %b", ALUCtrl);
      passTest(ALUCtrl, 4'b0001, "Test case 4", passed);
      
      {ALUop, Opcode} = {2'b10, 11'b10001010000};
      #50;
      $display ("ALUop = 10, Opcode = 10001010000, ALUCtrl = %b", ALUCtrl);
      passTest(ALUCtrl, 4'b0000, "Test case 5", passed);
      
      {ALUop, Opcode} = {2'b10, 11'b10001011000};
      #50;
      $display ("ALUop = 10, Opcode = 10001011000, ALUCtrl = %b", ALUCtrl);
      passTest(ALUCtrl, 4'b0010, "Test case 6", passed);
      
      {ALUop, Opcode} = {2'b10, 11'b10101010000};
      #50;
      $display ("ALUop = 10, Opcode = 10101010000, ALUCtrl = %b", ALUCtrl);
      passTest(ALUCtrl, 4'b0001, "Test case 7", passed);
      
      {ALUop, Opcode} = {2'b10, 11'b10001011000};
      #50;
      $display ("ALUop = 10, Opcode = 10001011000, ALUCtrl = %b", ALUCtrl);
      passTest(ALUCtrl, 4'b0010, "Test case 8", passed);
      
      {ALUop, Opcode} = {2'b10, 11'b11001011000};
      #50;
      $display ("ALUop = 10, Opcode = 11001011000, ALUCtrl = %b", ALUCtrl);
      passTest(ALUCtrl, 4'b0110, "Test case 9", passed);
		
		
      allPassed(passed, 9);

	end
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars(1);
  end
      
endmodule