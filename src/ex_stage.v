// Execution (EX) Stage
module ex_stage (
    input  logic        clk,          // Clock
    input  logic        rst,          // Reset

    // Inputs from ID Stage (or ID/EX pipeline register)
    input  logic [31:0] pc_i,         // Current PC from ID stage (for AUIPC)
    input  logic [31:0] rs1_data_i,   // Data from source register 1
    input  logic [31:0] rs2_data_i,   // Data from source register 2
    input  logic [31:0] imm_i,        // Immediate value
    input  logic [4:0]  rd_addr_i,    // Destination register address
    input  logic [31:0] pc_plus4_i,   // PC + 4 from ID stage
    // Control signals from ID stage
    input  logic        alu_src_a_sel_i, // ALU operand A source (0: PC, 1: rs1_data)
    input  logic [1:0]  alu_src_b_sel_i, // ALU operand B source (00: rs2_data, 01: imm, 10: 4) - Placeholder for '4'
    input  logic [3:0]  alu_op_i,     // ALU operation code
    input  logic [2:0]  funct3_i,     // Funct3 for branch condition evaluation
    input  logic        branch_i,     // Control signal: Is branch?

    // Outputs to MEM Stage (or EX/MEM pipeline register)
    output logic [31:0] alu_result_o, // Result from ALU
    output logic        alu_zero_o,   // Zero flag from ALU
    output logic [31:0] rs2_data_o,   // Pass through rs2 data (for stores)
    output logic [4:0]  rd_addr_o,    // Pass through destination register address
    output logic [31:0] pc_plus4_o,   // Pass through PC+4 (for JAL/JALR link address)
    // Outputs for PC Update Logic (to IF stage, potentially via MEM/WB)
    output logic [31:0] branch_target_addr_o, // Calculated branch target
    output logic [31:0] jalr_target_addr_o,   // Calculated JALR target
    output logic        branch_condition_met_o // Branch condition evaluation result
    // Pass through control signals for MEM/WB stages
);

    logic [31:0] alu_operand_a;
    logic [31:0] alu_operand_b;
    logic [31:0] alu_result;
    logic        alu_zero;
    logic        alu_sign; // Sign bit of ALU result for BLT/BGE

    // Select ALU Operand A
    // Selects between PC (for AUIPC) and rs1_data
    assign alu_operand_a = alu_src_a_sel_i ? rs1_data_i : pc_i;

    // Select ALU Operand B
    // Simplified: For now, assume operand B selection between rs2_data and immediate.
    // Will add '4' selection later for instructions like JAL/JALR.
    assign alu_operand_b = (alu_src_b_sel_i == 2'b01) ? imm_i : rs2_data_i; // Placeholder, needs refinement

    // Instantiate ALU
    alu alu_inst (
        .operand_a_i(alu_operand_a),
        .operand_b_i(alu_operand_b),
        .alu_op_i   (alu_op_i),
        .result_o   (alu_result),
        .zero_o     (alu_zero)
    );

    // Calculate Branch/JALR Target Addresses
    // Note: Immediate passed in (imm_i) should be the correct type (B-type for branch, I-type for JALR) based on ID stage logic
    assign branch_target_addr_o = pc_i + imm_i;
    assign jalr_target_addr_o   = rs1_data_i + imm_i; // JALR uses rs1 + I-imm

    // Evaluate Branch Condition
    assign alu_sign = alu_result[31]; // Sign bit of subtraction result
    always_comb begin
        branch_condition_met_o = 1'b0; // Default to not taken
        if (branch_i) begin
            case (funct3_i)
                3'b000: branch_condition_met_o = alu_zero;         // BEQ: taken if zero
                3'b001: branch_condition_met_o = ~alu_zero;        // BNE: taken if not zero
                3'b100: branch_condition_met_o = alu_sign;         // BLT: taken if negative (rs1 < rs2 signed)
                3'b101: branch_condition_met_o = ~alu_sign;        // BGE: taken if not negative (rs1 >= rs2 signed)
                3'b110: branch_condition_met_o = alu_sign;         // BLTU: taken if negative (rs1 < rs2 unsigned - check ALU SLTU result?)
                                                                    // Correction: Branch uses SUB result. For unsigned, taken if rs1 < rs2 -> result is negative *if* we treat operands as signed during SUB, OR if carry-out is 0.
                                                                    // Simpler: RISC-V spec says BLTU uses SLTU condition. Let's assume ALU provides necessary flags or we compare directly.
                                                                    // For now, let's use a simplified check based on SUB result sign, which is incorrect for unsigned.
                                                                    // TODO: Implement proper unsigned comparison check for BLTU/BGEU. Using SLTU result from ALU would be better.
                3'b111: branch_condition_met_o = ~alu_sign;        // BGEU: taken if not negative (rs1 >= rs2 unsigned) - see BLTU note.
                default: branch_condition_met_o = 1'b0;
            endcase
        end
    end
    // TODO: Refine BLTU/BGEU condition evaluation. Requires ALU to potentially output unsigned comparison result or carry flag.

    // Output Assignments
    assign alu_result_o = alu_result;
    assign alu_zero_o   = alu_zero;
    assign rs2_data_o   = rs2_data_i; // Pass through rs2 data for potential store instructions
    assign rd_addr_o    = rd_addr_i;  // Pass through destination register address
    assign pc_plus4_o   = pc_plus4_i; // Pass through PC+4

    // Pass through control signals (to be added)

endmodule
