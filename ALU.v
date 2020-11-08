`timescale 1ns / 1ps
`define ANDA 4'b0000
`define ORA 4'b0001
`define ADDA 4'b0010
`define LSLA 4'b0011
`define LSRA 4'b0100
`define SUBA 4'b0110
`define PassBA 4'b0111


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
