`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/05/2025 12:12:08 PM
// Design Name: 
// Module Name: PC
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
// 32-bit Program Counter (PC) for RISC-V Processor
//==============================================================

module PC (
    input wire clk,              // Clock signal
    input wire rst,              // Active-high reset
    input wire hold,             // Hold PC (stall signal)
    input wire pc_sel,           // 0 = PC + 4, 1 = load next_pc (branch/jump)
    input wire [31:0] next_pc,   // Next PC address from branch/jump logic
    output reg [31:0] pc         // Current PC value
);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pc <= 32'b0;                     // Reset PC to 0x00000000
        end
        else if (!hold) begin                // Update only when not holding
            if (pc_sel)
                pc <= next_pc;               // Load branch/jump target
            else
                pc <= pc + 4;                // Increment normally
        end
    end

endmodule

