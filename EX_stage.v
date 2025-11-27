`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/02/2025 06:24:51 PM
// Design Name: 
// Module Name: EX_stage
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
// RISC-V Execution (EX) Stage
// Integrates ALU, ALU Control, Branch Unit
//===========================================

module EX_stage (
    input  wire [31:0] pc_in,          // Current PC
    input  wire [31:0] rdata1,         // Source register 1 data
    input  wire [31:0] rdata2,         // Source register 2 data
    input  wire [31:0] imm_out,        // Immediate value
    input  wire [6:0] funct7,          // Function7 from instruction
    input  wire [2:0] funct3,          // Function3 from instruction
    input  wire [1:0] ALUOp,           // ALU operation type (from Control Unit)
    input  wire        ALUSrc,         // Select immediate or reg2 as ALU input
    input  wire        Branch,         // Branch control signal
    output wire [31:0] ALU_result,     // ALU output result
    output wire        Zero,           // Zero flag from ALU
    output wire [31:0] PC_next,        // Next PC after branch calc
    output wire        branch_taken    // Branch decision output
);

    // Intermediate wire between ALU Control and ALU
    wire [3:0] ALU_Ctrl;
    wire [31:0] ALU_input2;

    //==============================
    // ALU Control
    //==============================
    ALU_control alu_ctrl_inst (
        .ALUOp(ALUOp),
        .funct3(funct3),
        .funct7(funct7),
        .alu_ctrl(ALU_Ctrl)
    );

    //==============================
    // ALU Input Mux
    //==============================
    assign ALU_input2 = (ALUSrc) ? imm_out : rdata2;

    //==============================
    // ALU Module
    //==============================
    alu alu_inst (
        .A(rdata1),
        .B(ALU_input2),
        .ALU_Ctrl(ALU_Ctrl),
        .ALU_Result(ALU_result),
        .Zero(Zero)
    );

    //==============================
    // Branch Unit
    //==============================
    Branch_unit branch_inst (
        .PC(pc_in),
        .imm(imm_out),
        .Branch(Branch),
        .Zero(Zero),
        .PC_next(PC_next),
        .branch_taken(branch_taken)
    );

endmodule
