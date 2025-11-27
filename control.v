`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/01/2025 04:21:41 PM
// Design Name: 
// Module Name: control
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
//==============================================================
//  RISC-V Control Unit
//==============================================================

module control (
    input  [6:0] opcode,     // Opcode field from instruction
    output reg Branch,
    output reg MemRead,
    output reg MemtoReg,
    output reg [1:0] ALUOp,
    output reg MemWrite,
    output reg ALUSrc,
    output reg RegWrite
);

    always @(*) begin
        // Default values
        Branch   = 0;
        MemRead  = 0;
        MemtoReg = 0;
        ALUOp    = 2'b00;
        MemWrite = 0;
        ALUSrc   = 0;
        RegWrite = 0;

        case (opcode)
            7'b0110011: begin // R-type (ADD, SUB, AND, OR)
                RegWrite = 1;
                ALUSrc   = 0;
                ALUOp    = 2'b10;
            end

            7'b0010011: begin // I-type (ADDI)
                RegWrite = 1;
                ALUSrc   = 1;
                ALUOp    = 2'b11;
            end

            7'b0000011: begin // Load (LW)
                RegWrite = 1;
                ALUSrc   = 1;
                MemRead  = 1;
                MemtoReg = 1;
                ALUOp    = 2'b00;
            end

            7'b0100011: begin // Store (SW)
                ALUSrc   = 1;
                MemWrite = 1;
                ALUOp    = 2'b00;
            end

            7'b1100011: begin // Branch (BEQ)
                Branch   = 1;
                ALUOp    = 2'b01;
            end

            7'b1101111: begin // JAL
                RegWrite = 1;
                Branch   = 1;
                ALUOp    = 2'b00;
            end

            7'b0110111: begin // LUI
                RegWrite = 1;
                ALUSrc   = 1;
                ALUOp    = 2'b00;
            end

            default: begin
                // NOP or unsupported opcode
                Branch   = 0;
                MemRead  = 0;
                MemtoReg = 0;
                ALUOp    = 2'b00;
                MemWrite = 0;
                ALUSrc   = 0;
                RegWrite = 0;
            end
        endcase
    end
endmodule

