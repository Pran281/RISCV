`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/02/2025 06:05:22 PM
// Design Name: 
// Module Name: Branch_unit
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
//===========================================
// RISC-V Branch Unit
//===========================================

module Branch_unit (
    input wire [31:0] PC,          // Current Program Counter
    input wire [31:0] imm,         // Immediate value (from Immediate Generator)
    input wire Branch,             // Control signal from Control Unit
    input wire Zero,               // Zero flag from ALU
    output reg [31:0] PC_next,     // Next PC value
    output reg branch_taken        // 1 if branch is taken
);

always @(*) begin
    if (Branch && Zero) begin
        branch_taken = 1'b1;
        PC_next = PC + imm;        // Take branch
    end
    else begin
        branch_taken = 1'b0;
        PC_next = PC + 4;          // Normal sequential flow
    end
end

endmodule

