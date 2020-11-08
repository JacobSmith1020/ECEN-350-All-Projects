`timescale 1ns / 1ps
`define STRLEN 32
module SignExtenderTB;


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
  reg [31:0] Imm32;
  reg Ctrl;
  reg [7:0] passed;

  // Outputs
  wire [63:0] BusImm;

  // Instantiate the Unit Under Test (UUT)
  SignExtender uut (
    .BusImm(BusImm),
    .Imm32(Imm32),
    .Ctrl(Ctrl)
  );
  
  initial begin
    passed = 0;
    Imm32 = 0;
    Ctrl = 0;
    
    {Imm32, Ctrl} = {32'b11100001, 1'b1};
    #40;
    passTest(BusImm, 64'b11100001, "Check 1", passed);
    
    {Imm32, Ctrl} = {32'b1000011001011, 1'b1};
    #40;
    passTest(BusImm, 64'b1000011001011, "Check 1", passed);
    
    {Imm32, Ctrl} = {32'b1000011001011, 1'b0};
    #40;
    passTest(BusImm, 32'b1000011001011, "Check 1", passed);
  end
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars(1);
  end
endmodule