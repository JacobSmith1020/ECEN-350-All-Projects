`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    12:23:34 03/10/2009 
// Design Name: 
// Module Name:    SingleCycleControl 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
`define LDUROPCODE 11'b11111000010
`define STUROPCODE 11'b11111000000
`define ADDOPCODE  11'b10001011000
`define SUBOPCODE  11'b11001011000
`define ANDOPCODE  11'b10001010000
`define ORROPCODE  11'b10101010000
`define CBZOPCODE  11'b10110100???
`define BOPCODE    11'b000101?????
/* Note, the "?"'s in CBZ and B indicate don't care in a casez */

`define SIZE 1024

`define LS 4'b00
`define CBZ 4'b01
`define R 4'b10
`define ADD 11'b10001011000
`define SUB 11'b11001011000
`define AND 11'b10001010000
`define ORR 11'b10101010000

`define ANDA 4'b0000
`define ORA 4'b0001
`define ADDA 4'b0010
`define LSLA 4'b0011
`define LSRA 4'b0100
`define SUBA 4'b0110
`define PassBA 4'b0111

module DataMemory(ReadData , Address , WriteData , MemoryRead , MemoryWrite , Clock);
   input [63:0] WriteData;
   input [63:0] Address;
   input 	Clock,MemoryRead,MemoryWrite;
   output reg [63:0] ReadData;
   reg [7:0] 	     memBank[`SIZE-1:0];


   // This task is used to write arbitrary data to the Data Memory by
   // the intialization block.
   task initset;
      input [63:0] addr;
      input [63:0] data;
      begin
	 memBank[addr] =  data[63:56] ; // Big-endian for the win...
	 memBank[addr+1] =  data[55:48];
	 memBank[addr+2] =  data[47:40];
	 memBank[addr+3] =  data[39:32];
	 memBank[addr+4] =  data[31:24];
	 memBank[addr+5] =  data[23:16];
	 memBank[addr+6] =  data[15:8];
	 memBank[addr+7] =  data[7:0];
      end
   endtask
     
	     
   initial
     begin
	// preseting some data in the data memory used by test #1

	// Address 0x0 gets 0x1
	initset( 64'h0,  64'h1);  //Counter variable
	initset( 64'h8,  64'ha);  //Part of mask
	initset( 64'h10, 64'h5);  //Other part of mask
	initset( 64'h18, 64'h0ffbea7deadbeeff); //big constant
	initset( 64'h20, 64'h0); //clearing space

	// Add any data you need for your tests here.
	 
     end
	
   // This always block reads the data memory and places a double word
   // on the ReadData bus.
   always @(posedge Clock)
     begin
	if(MemoryRead)
	  begin
	     ReadData[63:56] <= #20 memBank[Address];
	     ReadData[55:48] <= #20 memBank[Address+1];
	     ReadData[47:40] <= #20 memBank[Address+2];
	     ReadData[39:32] <= #20 memBank[Address+3];
	     ReadData[31:24] <= #20 memBank[Address+4];
	     ReadData[23:16] <= #20 memBank[Address+5];
	     ReadData[15:8] <= #20 memBank[Address+6];
	     ReadData[7:0] <= #20 memBank[Address+7];
	  end
     end

   // This always block takes data from the WriteData bus and writes
   // it into the DataMemory.
   always @(posedge Clock)
     begin
	if(MemoryWrite)
	  begin
	     memBank[Address] <= #20 WriteData[63:56] ;
	     memBank[Address+1] <= #20 WriteData[55:48];
	     memBank[Address+2] <= #20 WriteData[47:40];
	     memBank[Address+3] <= #20 WriteData[39:32];
	     memBank[Address+4] <= #20 WriteData[31:24];
	     memBank[Address+5] <= #20 WriteData[23:16];
	     memBank[Address+6] <= #20 WriteData[15:8];
	     memBank[Address+7] <= #20 WriteData[7:0];
	     // Could be useful for debugging:
	     // $display("Writing Address:%h Data:%h",Address, WriteData);
	     
	  end
     end
endmodule

module InstructionMemory(Data, Address);
   parameter T_rd = 20;
   parameter MemSize = 40;
   
   output [31:0] Data;
   input [63:0]  Address;
   reg [31:0] 	 Data;
   
   /*
    * ECEN 350 Processor Test Functions
    * Texas A&M University
    */
   
   always @ (Address) begin
      case(Address)

	/* Test Program 1:
	 * Program loads constants from the data memory. Uses these constants to test
	 * the following instructions: LDUR, ORR, AND, CBZ, ADD, SUB, STUR and B.
	 * 
	 * Assembly code for test:
	 * 
	 * 0: LDUR X9, [XZR, 0x0]    //Load 1 into x9
	 * 4: LDUR X10, [XZR, 0x8]   //Load a into x10
	 * 8: LDUR X11, [XZR, 0x10]  //Load 5 into x11
	 * C: LDUR X12, [XZR, 0x18]  //Load big constant into x12
	 * 10: LDUR X13, [XZR, 0x20]  //load a 0 into X13
	 * 
	 * 14: ORR X10, X10, X11  //Create mask of 0xf
	 * 18: AND X12, X12, X10  //Mask off low order bits of big constant
	 * 
	 * loop:
	 * 1C: CBZ X12, end  //while X12 is not 0
	 * 20: ADD X13, X13, X9  //Increment counter in X13
	 * 24: SUB X12, X12, X9  //Decrement remainder of big constant in X12
	 * 28: B loop  //Repeat till X12 is 0
	 * 2C: STUR X13, [XZR, 0x20]  //store back the counter value into the memory location 0x20
	 */
	

	63'h000: Data = 32'hF84003E9;
	63'h004: Data = 32'hF84083EA;
	63'h008: Data = 32'hF84103EB;
	63'h00c: Data = 32'hF84183EC;
	63'h010: Data = 32'hF84203ED;
	63'h014: Data = 32'hAA0B014A;
	63'h018: Data = 32'h8A0A018C;
	63'h01c: Data = 32'hB400008C;
	63'h020: Data = 32'h8B0901AD;
	63'h024: Data = 32'hCB09018C;
	63'h028: Data = 32'h17FFFFFD;
	63'h02c: Data = 32'hF80203ED;
	63'h030: Data = 32'hF84203ED;  //One last load to place stored value on memdbus for test checking.

	/* Add code for your tests here */

			
	default: Data = 32'hXXXXXXXX;
      endcase
   end
endmodule

module SingleCycleProc(CLK, Reset_L, startPC, currentPC, dMemOut);
   input CLK;
   input Reset_L;
   input [63:0] startPC;
   output [63:0] currentPC;
   output [63:0] dMemOut;
   
   //PC Logic
   wire [63:0] 	 nextPC;
   reg [63:0] 	 currentPC;
   
   //Instruction Decode
   wire [31:0] 	 currentInstruction;
   wire [10:0] 	 opcode;
   wire [4:0] 	 rm,rn,rd;
   wire [5:0] 	 shamt;

   // Decoding instruction fields
   assign {opcode, rm, shamt, rn, rd} = currentInstruction;
   
   //Register wires
   wire [63:0] 	 busA, busB, busW; //buses for inputs and
   //outputs of regfile
   reg [4:0] 	 rB; // Used to attach output of
   // Reg2Loc mux to B input register
   // index input
   reg [63:0] writeRegister;
   
   //Control Logic Wires
   wire 	 Reg2Loc, ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, Branch, Uncondbranch;
   wire [1:0] 	 ALUOp;
   wire [3:0] 	 ALUCtrl;
   
   //ALU Wires
   wire [63:0] 	 signExtImm64;
   wire [63:0] ALUImmRegChoice;
   wire [63:0] 	 ALUResult;
   wire 	 ALUZero;
   
   //Data Memory Wires
   wire [63:0] 	 dMemOut;


   //Instruction Memory
   InstructionMemory instrMem(currentInstruction, currentPC);
   
   //PC Logic
   NextPClogic next(nextPC, currentPC, signExtImm64, Branch, ALUZero, Uncondbranch);
   always @ (negedge CLK, negedge Reset_L) begin
      if(~Reset_L)
	      currentPC = startPC;
      else
	      currentPC = nextPC;
   end
   
   //Control
   SingleCycleControl control(Reg2Loc, ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, Branch, Uncondbranch, ALUOp, opcode);
   
   //Register file
   always@(Reg2Loc) begin
      if(Reg2Loc == 0)
         rB <= #2 rm;
      else if(Reg2Loc == 1)
         rB <= #2 rd;
   end


   RegisterFile registers(busA, busB, busW, rn, rB, rd, RegWrite, CLK);
   
   //Sign Extender
   /*instantiate your sign extender*/
   SignExtender sgnext(signExtImm64, currentInstruction);
  
  
   //ALU
   ALUControl ALUCont(ALUCtrl, ALUOp, opcode);
   assign #2 ALUImmRegChoice = ALUSrc ? signExtImm64 : busB;
   ALU mainALU(ALUResult, ALUZero, busA, ALUImmRegChoice, ALUCtrl);
   
   //Data Memory
   DataMemory data(dMemOut, ALUResult, busB, MemRead, MemWrite, CLK);
   /*create MemToReg mux */
   always@(MemToReg) begin
      if(MemToReg == 0)
         writeRegister <= #2 ALUResult;
      else if(MemToReg == 1)
         writeRegister <= #2 dMemOut;
   end
   
endmodule



module ALUControl(ALUCtrl, ALUop, Opcode); 
  input [1:0] ALUop; 
  input [10:0] Opcode; 
  output reg [3:0] ALUCtrl; 
  
  always @(ALUop | Opcode) begin //when ALUop changes or Opcode changes, modify the ALUCtrl value
    case(ALUop)
      `LS: begin //if ALUop == load/store
        ALUCtrl <= #2 4'b0010;
      end
      `CBZ: begin //if ALUop == CBZ
        ALUCtrl <= #2 4'b0111;
      end
      `R: begin //if ALUop == R type instruction
        if(Opcode == `ADD)
          ALUCtrl <= #2 4'b0010;
        else if(Opcode == `SUB)
          ALUCtrl <= #2 4'b0110;
        else if(Opcode == `AND)
          ALUCtrl <= #2 4'b0000;
        else if(Opcode == `ORR)
          ALUCtrl <= #2 4'b0001;
      end
    endcase
  end
endmodule

module SignExtender(BusImm, Imm32); 
  output reg [63:0] BusImm; 
  input [31:0] Imm32; 

  always@(Imm32) begin
  	BusImm[63:0] <= {{32{Imm32[31]}}, Imm32[31:0]};
  end

endmodule

module ALU(BusW, Zero, BusA, BusB, ALUCtrl);

  output  [63:0] BusW;
  input   [63:0] BusA, BusB;
  input   [3:0] ALUCtrl;
  output reg  Zero;

  reg     [63:0] BusW;

  always @(ALUCtrl or BusA or BusB) begin
    case(ALUCtrl)
      `ANDA: begin
        BusW <= #20 (BusA & BusB);
      end
      `ORA: begin
        BusW <= #20 (BusA | BusB);
      end
      `ADDA: begin
        BusW <= #20 (BusA + BusB);
      end
      `LSLA: begin
        BusW <= #20 (BusA * (2 ** BusB));
      end
      `LSRA: begin
        BusW <= #20 (BusA / (2 ** BusB));
      end
      `SUBA: begin
        BusW <= #20 (BusA - BusB);
      end
      `PassBA: begin
        BusW <= #20 BusB;
      end

    endcase
  end
	
  always@(BusW)
    begin
      if(BusW == 0)
        assign Zero = 1;
      else
        assign Zero = 0;
    end
endmodule

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

module SingleCycleControl(Reg2Loc, ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, Branch, Uncondbranch, ALUOp, Opcode);
   input [10:0] Opcode;
   output 	Reg2Loc;
   output 	ALUSrc;
   output 	MemToReg;
   output 	RegWrite;
   output 	MemRead;
   output 	MemWrite;
   output 	Branch;
   output 	Uncondbranch;
   
   output [1:0] ALUOp;
   
   reg 		Reg2Loc, ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, Branch, Uncondbranch;
   reg [1:0] 	ALUOp;
   always @ (Opcode) begin
     casex(Opcode)
       `LDUROPCODE: begin
         Reg2Loc <= #2 1'bx;
         Uncondbranch <= #2 1'b0;
         Branch <= #2 1'b0;
         MemRead <= #2 1'b1;
         MemToReg <= #2 1'b1;
         MemWrite <= #2 1'b0;
         ALUSrc <= #2 1'b1;
         RegWrite <= #2 1'b1;
         ALUOp <= #2 2'b00;
       end
       `STUROPCODE: begin
         Reg2Loc <= #2 1'b1;
         Uncondbranch <= #2 1'b0;
         Branch <= #2 1'b0;
         MemRead <= #2 1'b0;
         MemToReg <= #2 1'bx;
         MemWrite <= #2 1'b1;
         ALUSrc <= #2 1'b1;
         RegWrite <= #2 1'b0;
         ALUOp <= #2 2'b00;
       end
       `ADDOPCODE: begin
         Reg2Loc <= #2 1'b0;
         Uncondbranch <= #2 1'b0;
         Branch <= #2 1'b0;
         MemRead <= #2 1'b0;
         MemToReg <= #2 1'b0;
         MemWrite <= #2 1'b0;
         ALUSrc <= #2 1'b0;
         RegWrite <= #2 1'b1;
         ALUOp <= #2 2'b10;
       end
       `SUBOPCODE: begin
         Reg2Loc <= #2 1'b0;
         Uncondbranch <= #2 1'b0;
         Branch <= #2 1'b0;
         MemRead <= #2 1'b0;
         MemToReg <= #2 1'b0;
         MemWrite <= #2 1'b0;
         ALUSrc <= #2 1'b0;
         RegWrite <= #2 1'b1;
         ALUOp <= #2 2'b10;
       end
       `ANDOPCODE: begin
         Reg2Loc <= #2 1'b0;
         Uncondbranch <= #2 1'b0;
         Branch <= #2 1'b0;
         MemRead <= #2 1'b0;
         MemToReg <= #2 1'b0;
         MemWrite <= #2 1'b0;
         ALUSrc <= #2 1'b0;
         RegWrite <= #2 1'b1;
         ALUOp <= #2 2'b10;
       end
       `ORROPCODE: begin
         Reg2Loc <= #2 1'b0;
         Uncondbranch <= #2 1'b0;
         Branch <= #2 1'b0;
         MemRead <= #2 1'b0;
         MemToReg <= #2 1'b0;
         MemWrite <= #2 1'b0;
         ALUSrc <= #2 1'b0;
         RegWrite <= #2 1'b1;
         ALUOp <= #2 2'b10;
       end
       `CBZOPCODE: begin
         Reg2Loc <= #2 1'b1;
         Uncondbranch <= #2 1'b0;
         Branch <= #2 1'b1;
         MemRead <= #2 1'b0;
         MemToReg <= #2 1'bx;
         MemWrite <= #2 1'b0;
         ALUSrc <= #2 1'b0;
         RegWrite <= #2 1'b0;
         ALUOp <= #2 2'b01;
       end
       `BOPCODE: begin
         Reg2Loc <= #2 1'bx;
         Uncondbranch <= #2 1'b1;
         Branch <= #2 1'b0;
         MemRead <= #2 1'bx;
         MemToReg <= #2 1'bx;
         MemWrite <= #2 1'bx;
         ALUSrc <= #2 1'bx;
         RegWrite <= #2 1'bx;
         ALUOp <= #2 2'bx1;
       end
        default: begin
           Reg2Loc <= #2 1'bx;
           ALUSrc <= #2 1'bx;
           MemToReg <= #2 1'bx;
           RegWrite <= #2 1'bx;
           MemRead <= #2 1'bx;
           MemWrite <= #2 1'bx;
           Branch <= #2 1'bx;
           ALUOp <= #2 2'bxx;
        end
      endcase
   end
endmodule