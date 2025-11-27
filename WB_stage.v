`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/04/2025 05:05:56 PM
// Design Name: 
// Module Name: WB_stage
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
// Write Back Stage for RISC-V Processor
//====================================================
module WB_stage (
    input  wire [31:0] read_data,     // Data from memory
    input  wire [31:0] ALU_result,    // Data from ALU
    input  wire        MemtoReg,      // Control: select memory or ALU result
    output wire [31:0] write_data     // Final data to be written to register file
);

    // If MemtoReg = 1 ? choose memory data, else ? choose ALU result
    assign write_data = (MemtoReg) ? read_data : ALU_result;

endmodule

