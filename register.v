// register.v  (modified to add tasks for testbench access)
`timescale 1ns/1ps
module register (
    input  wire clk,
    input  wire rst,
    input  wire RegWrite,
    input  wire [4:0] rs1,
    input  wire [4:0] rs2,
    input  wire [4:0] rd,
    input  wire [31:0] write_data,
    output wire [31:0] rdata1,
    output wire [31:0] rdata2
);
    // internal reg file array (private)
    reg [31:0] regs_arr [0:31];
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1) regs_arr[i] = 32'd0;
    end

    // read ports (combinational)
    assign rdata1 = (rs1 == 5'd0) ? 32'd0 : regs_arr[rs1];
    assign rdata2 = (rs2 == 5'd0) ? 32'd0 : regs_arr[rs2];

    // write port
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < 32; i = i + 1) regs_arr[i] <= 32'd0;
        end else begin
            if (RegWrite && rd != 5'd0) regs_arr[rd] <= write_data;
        end
    end

    // -----------------------
    // Testbench-accessible tasks
    // -----------------------
    // Read single register (call from TB)
    task read_reg;
        input  integer idx;
        output reg [31:0] val;
        begin
            if (idx >= 0 && idx < 32) val = regs_arr[idx];
            else val = 32'hxxxx_xxxx;
        end
    endtask

    // Dump all registers to console (call from TB)
    task dump_regs;
        integer j;
        begin
            $display("----- Register file dump -----");
            for (j = 0; j < 32; j = j + 1) begin
                $display("x%0d = 0x%08h", j, regs_arr[j]);
            end
            $display("-------------------------------");
        end
    endtask

endmodule
