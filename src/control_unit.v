// Control Unit for RISC-V CPU
module control_unit (
    input  logic [6:0] opcode_i,    // Instruction opcode
    input  logic [2:0] funct3_i,    // Instruction funct3
    input  logic       funct7_b5_i, // Instruction funct7 bit 5 (for SUB/SRA)

    // Control Signals Outputs (Refined)
    // For EX Stage
    output logic       alu_src_a_sel_o, // ALU operand A source (0: PC, 1: rs1) - Placeholder value
    output logic [1:0] alu_src_b_sel_o, // ALU operand B source (00: rs2, 01: imm, 10: 4)
    output logic [3:0] alu_op_o,        // ALU operation code
    // For MEM Stage
    output logic       mem_read_o,      // Memory read enable (for Loads)
    output logic       mem_write_o,     // Memory write enable (for Stores)
    // output logic [1:0] mem_op_size_o, // Memory operation size (byte, half, word) - Add later
    // For WB Stage
    output logic       reg_we_o,        // Register write enable
    output logic [1:0] wb_mux_sel_o,    // Write back data select (00: ALU, 01: Mem, 10: PC+4)
    // For IF Stage (PC Update Logic)
    output logic       branch_o,        // Instruction is a branch type
    output logic       jump_o           // Instruction is JAL or JALR
);

    // Define opcodes for clarity
    localparam OPCODE_R_TYPE  = 7'b0110011; // ADD, SUB, SLL, SLT, SLTU, XOR, SRL, SRA, OR, AND
    localparam OPCODE_I_TYPE  = 7'b0010011; // ADDI, SLTI, SLTIU, XORI, ORI, ANDI, SLLI, SRLI, SRAI
    localparam OPCODE_LOAD    = 7'b0000011; // LB, LH, LW, LBU, LHU
    localparam OPCODE_STORE   = 7'b0100011; // SB, SH, SW
    localparam OPCODE_BRANCH  = 7'b1100011; // BEQ, BNE, BLT, BGE, BLTU, BGEU
    localparam OPCODE_LUI     = 7'b0110111; // Load Upper Immediate
    localparam OPCODE_AUIPC   = 7'b0010111; // Add Upper Immediate to PC
    localparam OPCODE_JAL     = 7'b1101111; // Jump and Link
    localparam OPCODE_JALR    = 7'b1100111; // Jump and Link Register

    // Define ALU operation codes used by this control unit
    // These should correspond to the codes expected by the ALU module
    localparam ALU_OP_ADD  = 4'b0000;
    localparam ALU_OP_SUB  = 4'b0001;
    localparam ALU_OP_SLL  = 4'b0010;
    localparam ALU_OP_SLT  = 4'b0011;
    localparam ALU_OP_SLTU = 4'b0100;
    localparam ALU_OP_XOR  = 4'b0101;
    localparam ALU_OP_SRL  = 4'b0110;
    localparam ALU_OP_SRA  = 4'b0111;
    localparam ALU_OP_OR   = 4'b1000;
    localparam ALU_OP_AND  = 4'b1001;
    localparam ALU_OP_PASS_B = 4'b1010; // Pass operand B through (for LUI)
    // localparam ALU_OP_PASS_A = 4'b????; // If needed for AUIPC with PC as A

    // Default control signal values (inactive state)
    localparam DEFAULT_ALU_SRC_A = 1'b1; // Default to rs1
    localparam DEFAULT_ALU_SRC_B = 2'b00; // Default to rs2
    localparam DEFAULT_ALU_OP  = 4'b0000; // Default to ADD (can be anything safe)
    localparam DEFAULT_MEM_READ = 1'b0;
    localparam DEFAULT_MEM_WRITE= 1'b0;
    localparam DEFAULT_REG_WE  = 1'b0;
    localparam DEFAULT_WB_MUX  = 2'b00; // Default to ALU result
    localparam DEFAULT_BRANCH = 1'b0;
    localparam DEFAULT_JUMP   = 1'b0;

    // Combinational logic to generate control signals based on opcode
    always_comb begin
        // Assign default values first
        alu_src_a_sel_o = DEFAULT_ALU_SRC_A;
        alu_src_b_sel_o = DEFAULT_ALU_SRC_B;
        alu_op_o        = DEFAULT_ALU_OP;
        mem_read_o      = DEFAULT_MEM_READ;
        mem_write_o     = DEFAULT_MEM_WRITE;
        reg_we_o        = DEFAULT_REG_WE;
        wb_mux_sel_o    = DEFAULT_WB_MUX;
        branch_o        = DEFAULT_BRANCH;
        jump_o          = DEFAULT_JUMP;

        case (opcode_i)
            OPCODE_R_TYPE: begin
                // ALU uses rs1 and rs2, writes back ALU result to reg
                // R-type: ADD/SUB, SLL, SLT, SLTU, XOR, SRL/SRA, OR, AND
                alu_src_a_sel_o = 1'b1; // rs1
                alu_src_b_sel_o = 2'b00; // rs2
                reg_we_o        = 1'b1;
                wb_mux_sel_o    = 2'b00; // ALU result
                case (funct3_i)
                    3'b000: alu_op_o = funct7_b5_i ? ALU_OP_SUB : ALU_OP_ADD; // ADD or SUB
                    3'b001: alu_op_o = ALU_OP_SLL;
                    3'b010: alu_op_o = ALU_OP_SLT;
                    3'b011: alu_op_o = ALU_OP_SLTU;
                    3'b100: alu_op_o = ALU_OP_XOR;
                    3'b101: alu_op_o = funct7_b5_i ? ALU_OP_SRA : ALU_OP_SRL; // SRL or SRA
                    3'b110: alu_op_o = ALU_OP_OR;
                    3'b111: alu_op_o = ALU_OP_AND;
                    default: alu_op_o = DEFAULT_ALU_OP; // Should not happen
                endcase
            end
            OPCODE_I_TYPE: begin
                // I-type (ALU immediate): ADDI, SLTI, SLTIU, XORI, ORI, ANDI, SLLI, SRLI, SRAI
                alu_src_a_sel_o = 1'b1; // rs1
                alu_src_b_sel_o = 2'b01; // immediate
                reg_we_o        = 1'b1;
                wb_mux_sel_o    = 2'b00; // ALU result
                case (funct3_i)
                    3'b000: alu_op_o = ALU_OP_ADD;  // ADDI
                    3'b010: alu_op_o = ALU_OP_SLT;  // SLTI
                    3'b011: alu_op_o = ALU_OP_SLTU; // SLTIU
                    3'b100: alu_op_o = ALU_OP_XOR;  // XORI
                    3'b110: alu_op_o = ALU_OP_OR;   // ORI
                    3'b111: alu_op_o = ALU_OP_AND;  // ANDI
                    3'b001: alu_op_o = ALU_OP_SLL;  // SLLI (funct7 is 0)
                    3'b101: alu_op_o = funct7_b5_i ? ALU_OP_SRA : ALU_OP_SRL; // SRLI or SRAI
                    default: alu_op_o = DEFAULT_ALU_OP; // Should not happen
                endcase
            end
            OPCODE_LOAD: begin
                // ALU calculates address (rs1 + imm), reads mem, writes back mem data to reg
                alu_src_a_sel_o = 1'b1; // rs1
                alu_src_b_sel_o = 2'b01; // immediate (offset)
                alu_op_o        = 4'b0000; // ADD for address calculation
                mem_read_o      = 1'b1;
                reg_we_o        = 1'b1;
                wb_mux_sel_o    = 2'b01; // Memory data
            end
            OPCODE_STORE: begin
                // ALU calculates address (rs1 + imm), writes rs2 data to mem
                alu_src_a_sel_o = 1'b1; // rs1
                alu_src_b_sel_o = 2'b01; // immediate (offset)
                alu_op_o        = 4'b0000; // ADD for address calculation
                mem_write_o     = 1'b1;
                // No register write
            end
            OPCODE_BRANCH: begin
                // Branch: BEQ, BNE, BLT, BGE, BLTU, BGEU
                alu_src_a_sel_o = 1'b1; // rs1
                alu_src_b_sel_o = 2'b00; // rs2
                // ALU operation is SUB for comparison (result zero/sign used for branch decision)
                alu_op_o        = ALU_OP_SUB;
                branch_o        = 1'b1; // This is a branch instruction
                // No register write, no memory access in this stage
            end
            OPCODE_LUI: begin
                // Load Upper Immediate: rd = imm << 12
                alu_src_a_sel_o = 1'b0; // Doesn't matter, using PASS_B
                alu_src_b_sel_o = 2'b01; // immediate
                alu_op_o        = ALU_OP_PASS_B; // ALU passes immediate value through
                reg_we_o        = 1'b1;
                wb_mux_sel_o    = 2'b00; // ALU result (which is the immediate)
            end
            OPCODE_AUIPC: begin
                // Add Upper Immediate to PC: rd = pc + (imm << 12)
                alu_src_a_sel_o = 1'b0; // Select PC as operand A (Requires EX stage MUX update)
                alu_src_b_sel_o = 2'b01; // immediate
                alu_op_o        = ALU_OP_ADD; // ADD
                reg_we_o        = 1'b1;
                wb_mux_sel_o    = 2'b00; // ALU result
            end
            OPCODE_JAL: begin
                // Jump and Link: rd = pc + 4; pc = pc + imm
                // Write back PC+4
                reg_we_o        = 1'b1;
                wb_mux_sel_o    = 2'b10; // Select PC+4 (Requires WB stage MUX update)
                // ALU operation isn't strictly needed here for the write-back path,
                // but might be used for PC update logic if done in EX stage.
                // Set to something safe or specific if needed.
                alu_op_o        = DEFAULT_ALU_OP; // ALU not used for WB path, but PC update needs calculation
                jump_o          = 1'b1; // This is a jump instruction
            end
            OPCODE_JALR: begin
                // Jump and Link Register: rd = pc + 4; pc = (rs1 + imm) & ~1
                // Write back PC+4
                alu_src_a_sel_o = 1'b1; // rs1
                alu_src_b_sel_o = 2'b01; // immediate
                alu_op_o        = ALU_OP_ADD; // ADD for target address calculation (PC update logic)
                reg_we_o        = 1'b1;
                wb_mux_sel_o    = 2'b10; // Select PC+4 (Requires WB stage MUX update)
                jump_o          = 1'b1; // This is a jump instruction
            end
            default: begin
                // Treat as NOP or illegal instruction
                // Keep default inactive values
            end
        endcase
    end

endmodule
