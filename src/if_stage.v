// Instruction Fetch (IF) Stage
module if_stage (
    input  logic        clk,          // Clock
    input  logic        rst,          // Reset

    // Inputs for PC selection
    input  logic        jump_i,       // Is JAL or JALR?
    input  logic        branch_i,     // Is Branch?
    input  logic        branch_condition_met_i, // Branch condition met? (From EX stage)
    input  logic [31:0] jal_target_addr_i,    // Target address for JAL
    input  logic [31:0] jalr_target_addr_i,   // Target address for JALR (From EX stage)
    input  logic [31:0] branch_target_addr_i, // Target address for Branch (From EX stage)
    input  logic        is_jalr_i,    // Specific flag for JALR (to select between JAL/JALR target)

    output logic [31:0] pc_o,         // Current Program Counter value
    output logic [31:0] pc_plus4_o    // PC + 4, for next sequential instruction
);

    logic [31:0] pc_reg;      // Program Counter register
    logic [31:0] next_pc;     // Next PC value determined by MUX
    logic [31:0] pc_plus_4;   // Combinational calculation of PC + 4

    assign pc_plus_4 = pc_reg + 4;

    // PC Selection Logic (Combinational)
    always_comb begin
        if (jump_i) begin
            // JAL uses PC + J-imm, JALR uses rs1 + I-imm
            // We assume the correct target address is provided based on is_jalr_i flag
            next_pc = is_jalr_i ? jalr_target_addr_i : jal_target_addr_i;
        end else if (branch_i && branch_condition_met_i) begin
            next_pc = branch_target_addr_i; // Branch taken
        end else begin
            next_pc = pc_plus_4; // Default: next sequential instruction
        end
        // Ensure target addresses for JALR/Branch are word-aligned (optional, depends on spec interpretation)
        // next_pc = {next_pc[31:1], 1'b0}; // Example: Force lower bit to 0
    end

    // PC Register Logic (Sequential)
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            pc_reg <= 32'b0; // Reset PC to 0
        end else begin
            pc_reg <= next_pc; // Update PC with selected value
        end
    end

    // Output Assignments
    assign pc_o = pc_reg;       // Output the current PC
    assign pc_plus4_o = pc_plus_4; // Output PC + 4 (calculated combinationally)

    // The instruction address output for the memory is simply the current PC
    // This will be connected to instr_addr_o in the top module later.

endmodule
