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
