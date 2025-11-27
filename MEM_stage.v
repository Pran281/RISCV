`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/04/2025 04:46:54 PM
// Design Name: 
// Module Name: MEM_stage
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
// MEMORY STAGE (MEM)
//========================================================

module MEM_stage (
    input  wire        clk,            // Clock signal
    input  wire        MemRead,        // Control: Memory read
    input  wire        MemWrite,       // Control: Memory write
    input  wire [31:0] ALU_result,     // Address (from EX stage)
    input  wire [31:0] rdata2,         // Data to be written
    output wire [31:0] read_data,      // Data read from memory
    output wire [31:0] ALU_result_out  // Forwarded ALU result
);

    //===========================
    // Internal Data Memory (DMEM)
    //===========================
    reg [31:0] data_mem [0:255]; // 256 words = 1 KB

    //===========================
    // Memory Write Operation
    //===========================
    always @(posedge clk) begin
        if (MemWrite)
            data_mem[ALU_result[9:2]] <= rdata2; // word-aligned address
    end

    //===========================
    // Memory Read Operation
    //===========================
    assign read_data = (MemRead) ? data_mem[ALU_result[9:2]] : 32'b0;

    //===========================
    // Forward ALU result
    //===========================
    assign ALU_result_out = ALU_result;

endmodule
