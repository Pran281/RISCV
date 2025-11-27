`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/27/2025 11:22:57 AM
// Design Name: 
// Module Name: RV32I
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


`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Top-level: RV32I single-core, 5-stage pipeline (IF - ID - EX - MEM - WB)
// Features:
//  - IF/ID, ID/EX, EX/MEM, MEM/WB pipeline registers
//  - simple control & immediate gen for R, I, S, B, LOAD, STORE
//  - hazard detection (load-use stall)
//  - forwarding unit (EX stage takes forwarded operands from MEM/WB and EX/MEM)
//  - branch resolved in EX stage, flush on branch_taken
// Notes:
//  - ALU_control, alu, Branch_unit expected inside EX_stage (from your code).
//  - MEM_stage uses word-aligned addresses (ALU_result[9:2]) as in your module.
//  - IMEM initialized via $readmemh external file or manually in initial block.
//  - No byte/halfword memory ops, no exceptions, no CSRs, no prediction.
//////////////////////////////////////////////////////////////////////////////////

module RV32I (
    input  wire        clk,
    input  wire        rst,            // active-low reset
    input  wire [31:0] imem_init_addr    // optional starting PC
);

    // ----------------------------
    // Program Counter (IF)
    // ----------------------------
    reg [31:0] PC;
    wire [31:0] PC_plus4 = PC + 4;

    // instruction memory (simple array)
    localparam IMEM_SIZE = 256;
    reg [31:0] instr_mem [0:IMEM_SIZE-1];
    // Simulation-only initialization (legal inside module)
    integer i;
    initial begin
        // default fill with NOP (ADDI x0,x0,0 -> 32'h00000013)
        for (i = 0; i < IMEM_SIZE; i = i + 1)
            instr_mem[i] = 32'h00000013;
        // optionally load from file:
        // $readmemh("imem.hex", instr_mem);
    end

    wire [31:0] instr_fetch = instr_mem[PC[9:2]];

    // ----------------------------
    // IF/ID pipeline register
    // ----------------------------
    reg [31:0] IFID_PC;
    reg [31:0] IFID_instr;

    // ----------------------------
    // Register file (32 x 32)
    // ----------------------------
    reg [31:0] regfile [0:31];
    integer idx;
    initial begin
        for (idx=0; idx<32; idx=idx+1) regfile[idx] = 32'b0;
    end

    // WB side signals
    wire [31:0] WB_write_data;
    wire [4:0]  WB_rd;
    wire        WB_RegWrite;

    // synchronous write-back to regfile
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            for (idx=0; idx<32; idx=idx+1) regfile[idx] <= 32'b0;
        end else begin
            if (WB_RegWrite && (WB_rd != 5'd0))
                regfile[WB_rd] <= WB_write_data;
        end
    end

    // ----------------------------
    // ID stage: decode + control + imm gen
    // ----------------------------
    wire [6:0] ID_opcode = IFID_instr[6:0];
    wire [4:0] ID_rd     = IFID_instr[11:7];
    wire [2:0] ID_funct3 = IFID_instr[14:12];
    wire [4:0] ID_rs1    = IFID_instr[19:15];
    wire [4:0] ID_rs2    = IFID_instr[24:20];
    wire [6:0] ID_funct7 = IFID_instr[31:25];

    // ID outputs/control signals
    reg        ctrl_RegWrite;
    reg        ctrl_MemtoReg;
    reg        ctrl_MemRead;
    reg        ctrl_MemWrite;
    reg [1:0]  ctrl_ALUOp;
    reg        ctrl_ALUSrc;
    reg        ctrl_Branch;

    reg [31:0] imm_ID;

    always @(*) begin
        // defaults
        ctrl_RegWrite = 1'b0;
        ctrl_MemtoReg = 1'b0;
        ctrl_MemRead  = 1'b0;
        ctrl_MemWrite = 1'b0;
        ctrl_ALUOp    = 2'b00;
        ctrl_ALUSrc   = 1'b0;
        ctrl_Branch   = 1'b0;
        imm_ID        = 32'b0;

        case (ID_opcode)
            7'b0110011: begin // R-type
                ctrl_RegWrite = 1'b1;
                ctrl_ALUOp    = 2'b10;
                ctrl_ALUSrc   = 1'b0;
            end
            7'b0010011: begin // I-type ALU (ADDI etc.)
                ctrl_RegWrite = 1'b1;
                ctrl_ALUOp    = 2'b10; // treat like R-type but ALUSrc=1
                ctrl_ALUSrc   = 1'b1;
                imm_ID = {{20{IFID_instr[31]}}, IFID_instr[31:20]};
            end
            7'b0000011: begin // LOAD
                ctrl_RegWrite = 1'b1;
                ctrl_MemtoReg = 1'b1;
                ctrl_MemRead  = 1'b1;
                ctrl_ALUOp    = 2'b00; // add
                ctrl_ALUSrc   = 1'b1;
                imm_ID = {{20{IFID_instr[31]}}, IFID_instr[31:20]};
            end
            7'b0100011: begin // STORE (S)
                ctrl_MemWrite = 1'b1;
                ctrl_ALUOp    = 2'b00;
                ctrl_ALUSrc   = 1'b1;
                imm_ID = {{20{IFID_instr[31]}}, IFID_instr[31:25], IFID_instr[11:7]};
            end
            7'b1100011: begin // BRANCH
                ctrl_Branch = 1'b1;
                ctrl_ALUOp  = 2'b01; // branch compare
                ctrl_ALUSrc = 1'b0;
                imm_ID = {{19{IFID_instr[31]}}, IFID_instr[31], IFID_instr[7],
                          IFID_instr[30:25], IFID_instr[11:8], 1'b0};
            end
            default: begin
                // leave zeros
            end
        endcase
    end

    // read register file (combinational)
    wire [31:0] ID_rdata1 = regfile[ID_rs1];
    wire [31:0] ID_rdata2 = regfile[ID_rs2];

    // ----------------------------
    // ID/EX pipeline register
    // ----------------------------
    reg [31:0] IDEX_PC;
    reg [31:0] IDEX_rdata1;
    reg [31:0] IDEX_rdata2;
    reg [31:0] IDEX_imm;
    reg [6:0]  IDEX_funct7;
    reg [2:0]  IDEX_funct3;
    reg [4:0]  IDEX_rs1;
    reg [4:0]  IDEX_rs2;
    reg [4:0]  IDEX_rd;
    // control signals
    reg        IDEX_RegWrite;
    reg        IDEX_MemtoReg;
    reg        IDEX_MemRead;
    reg        IDEX_MemWrite;
    reg [1:0]  IDEX_ALUOp;
    reg        IDEX_ALUSrc;
    reg        IDEX_Branch;

    // ----------------------------
    // EX/MEM pipeline register
    // ----------------------------
    reg [31:0] EXMEM_PC;
    reg [31:0] EXMEM_ALU_result;
    reg [31:0] EXMEM_rdata2;
    reg [4:0]  EXMEM_rd;
    // control
    reg        EXMEM_RegWrite;
    reg        EXMEM_MemtoReg;
    reg        EXMEM_MemRead;
    reg        EXMEM_MemWrite;
    reg        EXMEM_Branch;

    // ----------------------------
    // MEM/WB pipeline register
    // ----------------------------
    reg [31:0] MEMWB_read_data;
    reg [31:0] MEMWB_ALU_result;
    reg [4:0]  MEMWB_rd;
    reg        MEMWB_RegWrite;
    reg        MEMWB_MemtoReg;

    // ----------------------------
    // Instantiate EX_stage, MEM_stage, WB_stage
    // ----------------------------
    // We'll provide rdata1/rdata2 (after forwarding muxes) to EX_stage inputs.
    wire [31:0] EX_ALU_result;
    wire        EX_Zero;
    wire [31:0] EX_PC_next;
    wire        EX_branch_taken;

    // forwarded operand wires
    wire [31:0] ex_in_rdata1;
    wire [31:0] ex_in_rdata2;

    EX_stage ex_stage_inst (
        .pc_in(IDEX_PC),
        .rdata1(ex_in_rdata1),
        .rdata2(ex_in_rdata2),
        .imm_out(IDEX_imm),
        .funct7(IDEX_funct7),
        .funct3(IDEX_funct3),
        .ALUOp(IDEX_ALUOp),
        .ALUSrc(IDEX_ALUSrc),
        .Branch(IDEX_Branch),
        .ALU_result(EX_ALU_result),
        .Zero(EX_Zero),
        .PC_next(EX_PC_next),
        .branch_taken(EX_branch_taken)
    );

    // MEM stage instance
    wire [31:0] MEM_read_data;
    wire [31:0] MEM_ALU_result_out;
    MEM_stage mem_stage_inst (
        .clk(clk),
        .MemRead(EXMEM_MemRead),
        .MemWrite(EXMEM_MemWrite),
        .ALU_result(EXMEM_ALU_result),
        .rdata2(EXMEM_rdata2),
        .read_data(MEM_read_data),
        .ALU_result_out(MEM_ALU_result_out)
    );

    // WB stage instance
    wire [31:0] WB_write_back;
    WB_stage wb_stage_inst (
        .read_data(MEMWB_read_data),
        .ALU_result(MEMWB_ALU_result),
        .MemtoReg(MEMWB_MemtoReg),
        .write_data(WB_write_back)
    );

    // connect WB outputs to regfile write signals
    assign WB_write_data = WB_write_back;
    assign WB_rd         = MEMWB_rd;
    assign WB_RegWrite   = MEMWB_RegWrite;

    // ----------------------------
    // Forwarding unit (simple)
    // Forward sources:
    //   00 -> use IDEX_rdataX (register file value)
    //   10 -> forward from EX/MEM.ALU_result (result in MEM stage)
    //   01 -> forward from MEM/WB (either ALU_result or MEM read_data depends on MemtoReg)
    // ----------------------------
    reg [1:0] forwardA;
    reg [1:0] forwardB;

    always @(*) begin
        // default: no forwarding
        forwardA = 2'b00;
        forwardB = 2'b00;

        // EX hazard: if EXMEM_RegWrite and EXMEM_rd != 0 and EXMEM_rd == IDEX_rs1 -> forward from EX/MEM
        if (EXMEM_RegWrite && (EXMEM_rd != 5'd0) && (EXMEM_rd == IDEX_rs1))
            forwardA = 2'b10;

        if (EXMEM_RegWrite && (EXMEM_rd != 5'd0) && (EXMEM_rd == IDEX_rs2))
            forwardB = 2'b10;

        // MEM hazard: if MEMWB_RegWrite and MEMWB_rd !=0 and MEMWB_rd == IDEX_rs1 -> forward from MEM/WB
        if (MEMWB_RegWrite && (MEMWB_rd != 5'd0) && (MEMWB_rd == IDEX_rs1))
            forwardA = 2'b01;

        if (MEMWB_RegWrite && (MEMWB_rd != 5'd0) && (MEMWB_rd == IDEX_rs2))
            forwardB = 2'b01;
    end

    // choose data for ALU inputs (ALUSrc still chooses between rdata2 and imm)
    reg [31:0] forwarded_A;
    reg [31:0] forwarded_B;

    always @(*) begin
        // forwarded A
        case (forwardA)
            2'b00: forwarded_A = IDEX_rdata1;
            2'b10: forwarded_A = EXMEM_ALU_result;
            2'b01: forwarded_A = (MEMWB_MemtoReg) ? MEMWB_read_data : MEMWB_ALU_result;
            default: forwarded_A = IDEX_rdata1;
        endcase

        // forwarded B
        case (forwardB)
            2'b00: forwarded_B = IDEX_rdata2;
            2'b10: forwarded_B = EXMEM_ALU_result;
            2'b01: forwarded_B = (MEMWB_MemtoReg) ? MEMWB_read_data : MEMWB_ALU_result;
            default: forwarded_B = IDEX_rdata2;
        endcase
    end

    // EX_stage expects ALU input chosen by ALUSrc inside EX_stage between imm and rdata2.
    // So feed forwarded values to ex_in_rdata1 and ex_in_rdata2.
    assign ex_in_rdata1 = forwarded_A;
    assign ex_in_rdata2 = forwarded_B;

    // ----------------------------
    // Hazard detection (load-use)
    // If IDEX_MemRead && (ID_rs1 == IDEX_rd || ID_rs2 == IDEX_rd) then stall 1 cycle:
    //   - freeze IF/ID and PC (PCWrite = 0)
    //   - insert bubble in ID/EX (zero control signals)
    // ----------------------------
    reg PCWrite;
    reg IFIDWrite;
    reg control_stall; // signal to zero IDEX control fields

    always @(*) begin
        if (IDEX_MemRead &&
           ((IDEX_rd == ID_rs1) || (IDEX_rd == ID_rs2))) begin
            // stall
            PCWrite = 1'b0;
            IFIDWrite = 1'b0;
            control_stall = 1'b1;
        end else begin
            PCWrite = 1'b1;
            IFIDWrite = 1'b1;
            control_stall = 1'b0;
        end
    end

    // ----------------------------
    // Pipeline sequential update
    // ----------------------------
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            // reset all pipeline regs
            PC <= imem_init_addr;
            IFID_PC <= 32'b0; IFID_instr <= 32'b0;

            IDEX_PC <= 32'b0; IDEX_rdata1 <= 32'b0; IDEX_rdata2 <= 32'b0; IDEX_imm <= 32'b0;
            IDEX_funct7 <= 7'd0; IDEX_funct3 <= 3'd0; IDEX_rs1 <= 5'd0; IDEX_rs2 <= 5'd0; IDEX_rd <= 5'd0;
            IDEX_RegWrite <= 1'b0; IDEX_MemtoReg <= 1'b0; IDEX_MemRead <= 1'b0; IDEX_MemWrite <= 1'b0;
            IDEX_ALUOp <= 2'b00; IDEX_ALUSrc <= 1'b0; IDEX_Branch <= 1'b0;

            EXMEM_PC <= 32'b0; EXMEM_ALU_result <= 32'b0; EXMEM_rdata2 <= 32'b0; EXMEM_rd <= 5'd0;
            EXMEM_RegWrite <= 1'b0; EXMEM_MemtoReg <= 1'b0; EXMEM_MemRead <= 1'b0; EXMEM_MemWrite <= 1'b0; EXMEM_Branch <= 1'b0;

            MEMWB_read_data <= 32'b0; MEMWB_ALU_result <= 32'b0; MEMWB_rd <= 5'd0;
            MEMWB_RegWrite <= 1'b0; MEMWB_MemtoReg <= 1'b0;
        end else begin
            // ---- PC and IF/ID
            if (EX_branch_taken) begin
                // branch taken: set PC to EX_PC_next and flush IF/ID
                PC <= EX_PC_next;
                IFID_PC <= 32'b0;
                IFID_instr <= 32'b0;
            end else begin
                if (PCWrite) begin
                    PC <= PC_plus4;
                end
                if (IFIDWrite) begin
                    IFID_PC <= PC;
                    IFID_instr <= instr_fetch;
                end
                // if IFIDWrite==0, keep IFID as-is (stall)
            end

            // ---- ID/EX
            if (EX_branch_taken) begin
                // flush IDEX on branch
                IDEX_PC <= 32'b0;
                IDEX_rdata1 <= 32'b0;
                IDEX_rdata2 <= 32'b0;
                IDEX_imm <= 32'b0;
                IDEX_funct7 <= 7'd0;
                IDEX_funct3 <= 3'd0;
                IDEX_rs1 <= 5'd0;
                IDEX_rs2 <= 5'd0;
                IDEX_rd <= 5'd0;
                IDEX_RegWrite <= 1'b0;
                IDEX_MemtoReg <= 1'b0;
                IDEX_MemRead <= 1'b0;
                IDEX_MemWrite <= 1'b0;
                IDEX_ALUOp <= 2'b00;
                IDEX_ALUSrc <= 1'b0;
                IDEX_Branch <= 1'b0;
            end else begin
                // if control_stall, insert bubble into IDEX controls but keep IF/ID values frozen
                IDEX_PC <= IFID_PC;
                IDEX_rdata1 <= ID_rdata1;
                IDEX_rdata2 <= ID_rdata2;
                IDEX_imm <= imm_ID;
                IDEX_funct7 <= IFID_instr[31:25];
                IDEX_funct3 <= IFID_instr[14:12];
                IDEX_rs1 <= ID_rs1;
                IDEX_rs2 <= ID_rs2;
                IDEX_rd  <= ID_rd;

                if (control_stall) begin
                    // zero control signals to create bubble
                    IDEX_RegWrite <= 1'b0;
                    IDEX_MemtoReg <= 1'b0;
                    IDEX_MemRead  <= 1'b0;
                    IDEX_MemWrite <= 1'b0;
                    IDEX_ALUOp    <= 2'b00;
                    IDEX_ALUSrc   <= 1'b0;
                    IDEX_Branch   <= 1'b0;
                end else begin
                    IDEX_RegWrite <= ctrl_RegWrite;
                    IDEX_MemtoReg <= ctrl_MemtoReg;
                    IDEX_MemRead  <= ctrl_MemRead;
                    IDEX_MemWrite <= ctrl_MemWrite;
                    IDEX_ALUOp    <= ctrl_ALUOp;
                    IDEX_ALUSrc   <= ctrl_ALUSrc;
                    IDEX_Branch   <= ctrl_Branch;
                end
            end

            // ---- EX -> MEM
            EXMEM_PC <= EX_PC_next;
            EXMEM_ALU_result <= EX_ALU_result;
            // take forwarded_B if ALUSrc==0 else rdata2 for store address/data calc?
            // Our MEM_stage expects rdata2 forwarded from EX stage results; use forwarded_B
            EXMEM_rdata2 <= forwarded_B;
            EXMEM_rd <= IDEX_rd;
            EXMEM_RegWrite <= IDEX_RegWrite;
            EXMEM_MemtoReg <= IDEX_MemtoReg;
            EXMEM_MemRead <= IDEX_MemRead;
            EXMEM_MemWrite <= IDEX_MemWrite;
            EXMEM_Branch <= IDEX_Branch;

            // ---- MEM -> WB
            MEMWB_read_data <= MEM_read_data;
            MEMWB_ALU_result <= EXMEM_ALU_result;
            MEMWB_rd <= EXMEM_rd;
            MEMWB_RegWrite <= EXMEM_RegWrite;
            MEMWB_MemtoReg <= EXMEM_MemtoReg;
        end
    end

    // End of module
endmodule

