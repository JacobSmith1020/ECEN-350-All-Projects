`timescale 1ns / 1ps
`define LS 4'b00
`define CBZ 4'b01
`define R 4'b10
`define ADD 11'b10001011000
`define SUB 11'b11001011000
`define AND 11'b10001010000
`define ORR 11'b10101010000
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