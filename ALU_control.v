`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/02/2025 05:35:27 PM
// Design Name: 
// Module Name: ALU_control
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
//========================================================
// ALU Control Unit for RISC-V Single Cycle Processor
//========================================================
module ALU_control (
    input wire [1:0] ALUOp,     // from main control unit
    input wire [2:0] funct3,    // from instruction[14:12]
    input wire [6:0] funct7,    // from instruction[31:25]
    output reg [3:0] alu_ctrl   // control signal to ALU
);

    always @(*) begin
        case (ALUOp)
            2'b00: alu_ctrl = 4'b0010; // Load/Store ? ADD
            2'b01: alu_ctrl = 4'b0110; // Branch ? SUB
            2'b10: begin               // R-type (based on funct3, funct7)
                case (funct3)
                    3'b000: alu_ctrl = (funct7[5]) ? 4'b0110 : 4'b0010; // SUB or ADD
                    3'b111: alu_ctrl = 4'b0000; // AND
                    3'b110: alu_ctrl = 4'b0001; // OR
                    3'b010: alu_ctrl = 4'b0111; // SLT
                    default: alu_ctrl = 4'b1111; // invalid
                endcase
            end
            default: alu_ctrl = 4'b1111;
        endcase
    end
endmodule

