`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/01/2025 06:43:02 PM
// Design Name: 
// Module Name: ID_stage
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
//  RISC-V Instruction Decode (ID) Stage
//  Modules Included: Decoder, Immediate Generator, Control Unit, Register File
//==============================================================
//==============================================================
//  RISC-V Instruction Decode (ID) Stage (Complete)
//  Includes: Decoder, Immediate Generator, Register File, Control Unit
//==============================================================

module ID_stage (
    input wire clk,
    input wire rst,
    input wire [31:0] instr,        // Instruction from IF stage
    input wire [31:0] write_data,   // Data from WB stage
    output wire [31:0] rdata1,      // Register rs1 data
    output wire [31:0] rdata2,      // Register rs2 data
    output wire [31:0] imm_out,     // Immediate value
    output wire RegWrite,           // Control signal
    output wire MemRead,
    output wire MemWrite,
    output wire Branch,
    output wire MemtoReg,
    output wire ALUSrc,
    output wire [1:0] ALUOp,        // ALU operation select
    output wire [6:0] opcode,
    output wire [2:0] funct3,
    output wire [6:0] funct7,
    output wire [4:0] rs1,
    output wire [4:0] rs2,
    output wire [4:0] rd
);

    // Internal wires for connection
    wire [6:0] opcode_w;
    wire [2:0] funct3_w;
    wire [6:0] funct7_w;
    wire [4:0] rs1_w, rs2_w, rd_w;

    //============================
    // Decoder
    //============================
    decoder dec_inst (
        .instr(instr),
        .opcode(opcode_w),
        .funct3(funct3_w),
        .funct7(funct7_w),
        .rs1(rs1_w),
        .rs2(rs2_w),
        .rd(rd_w)
    );

    //============================
    // Immediate Generator
    //============================
    imgen imm_inst (
        .instr(instr),
        .imm_out(imm_out)
    );

    //============================
    // Control Unit
    //============================
    control ctrl_inst (
        .opcode(opcode),
        .Branch(Branch),
        .MemRead(MemRead),
        .MemtoReg(MemtoReg),
        .ALUOp(ALUOp),
        .MemWrite(MemWrite),
        .ALUSrc(ALUSrc),
        .RegWrite(RegWrite)
    );
    //============================
    // Register File
    //============================
    register reg_inst (
            .clk(clk),
            .rst(rst),
            .RegWrite(RegWrite),
            .rs1(rs1_w),
            .rs2(rs2_w),
            .rd(rd_w),
            .write_data(write_data),
            .rdata1(rdata1),
            .rdata2(rdata2)
        );

    //============================
    // Output Assignments
    //============================
    assign opcode = opcode_w;
    assign funct3 = funct3_w;
    assign funct7 = funct7_w;
    assign rs1 = rs1_w;
    assign rs2 = rs2_w;
    assign rd  = rd_w;

endmodule
