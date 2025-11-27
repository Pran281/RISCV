`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/02/2025 05:54:40 PM
// Design Name: 
// Module Name: alu
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
//====================================================
// 32-bit ALU for RISC-V Processor
//====================================================

module alu (
    input  wire [31:0] A,          // Operand 1
    input  wire [31:0] B,          // Operand 2
    input  wire [3:0]  ALU_Ctrl,   // Control signal from ALU control unit
    output reg  [31:0] ALU_Result, // Result of ALU operation
    output wire Zero                // Zero flag (1 if result == 0)
);

    always @(*) begin
        case (ALU_Ctrl)
            4'b0010: ALU_Result = A + B;      // ADD
            4'b0110: ALU_Result = A - B;      // SUB
            4'b0000: ALU_Result = A & B;      // AND
            4'b0001: ALU_Result = A | B;      // OR
            4'b0111: ALU_Result = (A < B) ? 32'b1 : 32'b0; // SLT
            4'b0011: ALU_Result = A ^ B;      // XOR
            4'b0100: ALU_Result = A << B[4:0]; // SLL
            4'b0101: ALU_Result = A >> B[4:0]; // SRL
            default: ALU_Result = 32'b0;
        endcase
    end

    assign Zero = (ALU_Result == 32'b0) ? 1'b1 : 1'b0;

endmodule

