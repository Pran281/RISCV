`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/05/2025
// Design Name: Instruction Fetch Stage
// Module Name: IF_stage
// Project Name: RISC-V Single Cycle / Pipeline
// Target Devices: 
// Tool Versions: 
// Description: 
//     Instruction Fetch (IF) stage integrates the Program Counter (PC)
//     and Instruction Memory (inst_mem). It provides the current PC
//     and fetched instruction to the next pipeline stage.
//
// Dependencies: 
//     PC.v
//     inst_mem.v
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module IF_stage (
    input  wire        clk,         // Clock signal
    input  wire        rst,         // Active-high reset
    input  wire        hold,        // Stall signal from pipeline
    input  wire        pc_sel,      // PC select: 0 = PC+4, 1 = next_pc (branch/jump)
    input  wire [31:0] next_pc,     // Target PC for branch/jump
    output wire [31:0] instr,       // Instruction fetched from memory
    output wire [31:0] pc_out       // Current Program Counter
);

    //==========================================================
    // Instantiate Program Counter (PC)
    //==========================================================
    wire [31:0] pc;

    PC u_PC (
        .clk(clk),
        .rst(rst),
        .hold(hold),
        .pc_sel(pc_sel),
        .next_pc(next_pc),
        .pc(pc)
    );

    //==========================================================
    // Instantiate Instruction Memory
    //==========================================================
    inst_mem u_IMEM (
        .addr(pc),
        .instr(instr)
    );

    //==========================================================
    // Output Current PC Value
    //==========================================================
    assign pc_out = pc;

endmodule
