`timescale 1ns / 1ps
`define STRLEN 32
module RegisterFileTest_v;


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
  reg [63:0] BusW;
  reg [4:0] RA;
  reg [4:0] RB;
  reg [4:0] RW;
  reg RegWr;
  reg Clk;
  reg [7:0] passed;

  // Outputs
  wire [63:0] BusA;
  wire [63:0] BusB;

  // Instantiate the Unit Under Test (UUT)
  RegisterFile uut (
    .BusA(BusA), 
    .BusB(BusB), 
    .BusW(BusW), 
    .RA(RA), 
    .RB(RB), 
    .RW(RW), 
    .RegWr(RegWr), 
    .Clk(Clk)
  );

  initial begin
    // Initialize Inputs
    BusW = 0;
    RA = 0;
    RB = 0;
    RW = 0;
    RegWr = 0;
    Clk = 1;
    passed = 0;

    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd0, 64'h0, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd1, 64'h1, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd2, 64'h2, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd3, 64'h3, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd4, 64'h4, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd5, 64'h5, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd6, 64'h6, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd7, 64'h7, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd8, 64'h8, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd9, 64'h9, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd10, 64'h10, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd11, 64'h11, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd12, 64'h12, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd13, 64'h13, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd14, 64'h14, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd15, 64'h15, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd16, 64'h16, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd17, 64'h17, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd18, 64'h18, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd19, 64'h19, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd20, 64'h20, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd21, 64'h21, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd22, 64'h22, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd23, 64'h23, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd24, 64'h24, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd25, 64'h25, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd26, 64'h26, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd27, 64'h27, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd28, 64'h28, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd29, 64'h29, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd30, 64'h30, 1'b1};#10; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd0, 5'd31, 64'h31, 1'b1};#10; Clk = 0; #10; Clk = 1;


    //Adding custom test cases
    {RA, RB, RW, BusW, RegWr} = {5'd0, 5'd1, 5'd0, 64'h0, 1'b0};
    #4;
    passTest(BusA, 64'h0, "Check 1 Bus A", passed);
    passTest(BusB, 64'h1, "Check 1 Bus B", passed);
    #6; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd2, 5'd3, 5'd1, 64'h1000, 1'b0};
    #4;
    passTest(BusA, 64'h2, "Check 2 Bus A", passed);
    passTest(BusB, 64'h3, "Check 2 Bus B", passed);
    #6; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd4, 5'd5, 5'd0, 64'h1000, 1'b1};
    #4;
    passTest(BusA, 64'h4, "Check 3 Bus A", passed);
    passTest(BusB, 64'h5, "Check 3 Bus B", passed);
    #6; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd6, 5'd7, 5'd10, 64'h1010, 1'b1};
    #4;
    passTest(BusA, 64'h6, "Check 4 Bus A", passed);
    passTest(BusB, 64'h7, "Check 4 Bus B", passed);
    #6; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd8, 5'd9, 5'd11, 64'h103000, 1'b1};
    #4;
    passTest(BusA, 64'h8, "Check 5 Bus A", passed);
    passTest(BusB, 64'h9, "Check 5 Bus B", passed);
    #6; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd10, 5'd11, 5'd12, 64'h0, 1'b0};
    #4;
    passTest(BusA, 64'd4112, "Check 6 Bus A", passed);
    passTest(BusB, 64'd1060864, "Check 6 Bus B", passed);
    #6; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd12, 5'd13, 5'd13, 64'hABCD, 1'b1};
    #4;
    passTest(BusA, 64'd18, "Check 7 Bus A", passed);
    passTest(BusB, 64'd19, "Check 7 Bus B", passed);
    #6; Clk = 0; #10; Clk = 1;
    {RA, RB, RW, BusW, RegWr} = {5'd14, 5'd15, 5'd14, 64'h9080009, 1'b0};
    #4;
    passTest(BusA, 64'd20, "Check 8 Bus A", passed);
    passTest(BusB, 64'd21, "Check 8 Bus B", passed);
    #6; Clk = 0; #10; Clk = 1;
    
    allPassed(passed, 16);
    end
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars(1);
  end
endmodule