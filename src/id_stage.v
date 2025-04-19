// Instruction Decode (ID) Stage
module id_stage (
    input  logic        clk,          // Clock
    input  logic        rst,          // Reset

    // Inputs from IF Stage (or IF/ID pipeline register)
    input  logic [31:0] pc_i,         // Current PC from IF stage
    input  logic [31:0] instr_i,      // Instruction from IF stage
    input  logic [31:0] pc_plus4_i,   // PC + 4 from IF stage

    // Inputs for Write Back (WB) stage (writing result to register file)
    input  logic        wb_reg_we_i,  // Register write enable from WB
    input  logic [4:0]  wb_rd_addr_i, // Destination register address from WB
    input  logic [31:0] wb_rd_data_i, // Data to write back from WB

    // Outputs to EX Stage (or ID/EX pipeline register)
    output logic [31:0] rs1_data_o,   // Data from source register 1
    output logic [31:0] rs2_data_o,   // Data from source register 2
    output logic [31:0] imm_o,        // Immediate value generated
    output logic [4:0]  rd_addr_o,    // Destination register address
    output logic [6:0]  opcode_o,     // Instruction opcode
    output logic [2:0]  funct3_o,     // Instruction funct3
    output logic [6:0]  funct7_o,     // Instruction funct7
    output logic [31:0] pc_o,         // Pass PC through
    output logic [31:0] pc_plus4_o    // Pass PC+4 through
    // Add control signals outputs later (e.g., alu_op, mem_read, mem_write, etc.)
);

    // Instruction field extraction
    logic [4:0] rs1_addr;
    logic [4:0] rs2_addr;

    assign opcode_o = instr_i[6:0];
    assign rd_addr_o = instr_i[11:7];
    assign funct3_o = instr_i[14:12];
    assign rs1_addr = instr_i[19:15];
    assign rs2_addr = instr_i[24:20];
    assign funct7_o = instr_i[31:25];

    // Register File
    logic [31:0] rf [31:0]; // 32 registers, 32 bits each

    // Read Ports (combinational read)
    // Handle reading from x0 (always zero)
    assign rs1_data_o = (rs1_addr == 5'b0) ? 32'b0 : rf[rs1_addr];
    assign rs2_data_o = (rs2_addr == 5'b0) ? 32'b0 : rf[rs2_addr];

    // Write Port (synchronous write)
    always_ff @(posedge clk) begin
        if (wb_reg_we_i && (wb_rd_addr_i != 5'b0)) begin // Check write enable and not writing to x0
            rf[wb_rd_addr_i] <= wb_rd_data_i;
        end
    end

    // Initialize Register File on Reset (optional, good for simulation)
    // synthesis translate_off
    integer i;
    initial begin
        if (rst) begin // Use initial block for reset initialization in simulation
             for (i = 0; i < 32; i = i + 1) begin
                 rf[i] = 32'b0;
             end
        end
    end
    // synthesis translate_on

    // Immediate Generation Logic
    logic [31:0] imm_i_type;
    logic [31:0] imm_s_type;
    logic [31:0] imm_b_type;
    logic [31:0] imm_u_type;
    logic [31:0] imm_j_type;

    // I-type immediate (Instructions: ADDI, SLTI, SLTIU, XORI, ORI, ANDI, SLLI, SRLI, SRAI, LB, LH, LW, LBU, LHU, JALR)
    // imm[11:0] = instr[31:20]
    assign imm_i_type = {{20{instr_i[31]}}, instr_i[31:20]}; // Sign extension

    // S-type immediate (Instructions: SB, SH, SW)
    // imm[11:5] = instr[31:25], imm[4:0] = instr[11:7]
    assign imm_s_type = {{20{instr_i[31]}}, instr_i[31:25], instr_i[11:7]}; // Sign extension

    // B-type immediate (Instructions: BEQ, BNE, BLT, BGE, BLTU, BGEU)
    // imm[12|10:5] = instr[31|30:25], imm[4:1|11] = instr[11:8|7]
    // Immediate is shifted left by 1 bit in hardware (address is PC + imm)
    assign imm_b_type = {{20{instr_i[31]}}, instr_i[7], instr_i[30:25], instr_i[11:8], 1'b0}; // Sign extension

    // U-type immediate (Instructions: LUI, AUIPC)
    // imm[31:12] = instr[31:12]
    assign imm_u_type = {instr_i[31:12], 12'b0}; // No sign extension needed, lower bits are 0

    // J-type immediate (Instructions: JAL)
    // imm[20|10:1|11|19:12] = instr[31|30:21|20|19:12]
    // Immediate is shifted left by 1 bit in hardware (address is PC + imm)
    assign imm_j_type = {{12{instr_i[31]}}, instr_i[19:12], instr_i[20], instr_i[30:21], 1'b0}; // Sign extension

    // Select the correct immediate based on opcode
    // This is a simplified selection logic, a more robust control unit will refine this.
    // We will add a dedicated control unit later.
    always_comb begin
        case (opcode_o)
            // R-type: No immediate used directly here (handled by ALU)
            7'b0110011: imm_o = 32'b0; // R-type (e.g., ADD, SUB)

            // I-type (ALU immediate, Load, JALR)
            7'b0010011: imm_o = imm_i_type; // I-type (e.g., ADDI)
            7'b0000011: imm_o = imm_i_type; // I-type Load (e.g., LW)
            7'b1100111: imm_o = imm_i_type; // I-type JALR

            // S-type (Store)
            7'b0100011: imm_o = imm_s_type; // S-type (e.g., SW)

            // B-type (Branch)
            7'b1100011: imm_o = imm_b_type; // B-type (e.g., BEQ)

            // U-type (LUI, AUIPC)
            7'b0110111: imm_o = imm_u_type; // U-type LUI
            7'b0010111: imm_o = imm_u_type; // U-type AUIPC

            // J-type (JAL)
            7'b1101111: imm_o = imm_j_type; // J-type JAL

            default: imm_o = 32'b0; // Default or unsupported opcode
        endcase
    end

    // Pass PC+4 through
    assign pc_o = pc_i;               // Pass PC through
    assign pc_plus4_o = pc_plus4_i;

    // Control Signal Generation Logic (To be added)

endmodule
