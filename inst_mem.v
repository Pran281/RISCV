//==============================================================
// 32-bit Instruction Memory (Read-Only) for RISC-V
//==============================================================
module inst_mem (
    input  wire [31:0] addr,   // Address from Program Counter
    output reg  [31:0] instr   // 32-bit instruction output
);

    // Memory array: 256 words × 32 bits = 1 KB
    reg [31:0] memory [0:255];

    // Step 1: Initialize with program instructions
    initial begin
        // Load machine code (hexadecimal instructions) from file
        $readmemh("program.hex.txt", memory);
    end

    // Step 2: Fetch instruction based on address
    always @(*) begin
        instr = memory[addr[9:2]]; // Word-aligned access
    end

endmodule
//==============================================================
