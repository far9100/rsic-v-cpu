// Arithmetic Logic Unit (ALU)
module alu (
    input  logic [31:0] operand_a_i, // Operand A
    input  logic [31:0] operand_b_i, // Operand B
    input  logic [3:0]  alu_op_i,    // ALU operation code
    output logic [31:0] result_o,    // ALU result
    output logic        zero_o       // Zero flag (result is zero)
);

    // Define ALU operation codes (example, can be refined)
    localparam ALU_OP_ADD  = 4'b0000;
    localparam ALU_OP_SUB  = 4'b0001;
    localparam ALU_OP_SLL  = 4'b0010; // Shift Left Logical
    localparam ALU_OP_SLT  = 4'b0011; // Set Less Than (Signed)
    localparam ALU_OP_SLTU = 4'b0100; // Set Less Than (Unsigned)
    localparam ALU_OP_XOR  = 4'b0101;
    localparam ALU_OP_SRL  = 4'b0110; // Shift Right Logical
    localparam ALU_OP_SRA  = 4'b0111; // Shift Right Arithmetic
    localparam ALU_OP_OR   = 4'b1000;
    localparam ALU_OP_AND  = 4'b1001;
    // Add more operations as needed (e.g., LUI, AUIPC handling might be done here or separately)

    logic [31:0] result_comb; // Combinational result

    always_comb begin
        case (alu_op_i)
            ALU_OP_ADD:  result_comb = operand_a_i + operand_b_i;
            ALU_OP_SUB:  result_comb = operand_a_i - operand_b_i;
            ALU_OP_SLL:  result_comb = operand_a_i << operand_b_i[4:0]; // Shift amount from lower 5 bits of B
            ALU_OP_SLT:  result_comb = ($signed(operand_a_i) < $signed(operand_b_i)) ? 32'd1 : 32'd0;
            ALU_OP_SLTU: result_comb = (operand_a_i < operand_b_i) ? 32'd1 : 32'd0;
            ALU_OP_XOR:  result_comb = operand_a_i ^ operand_b_i;
            ALU_OP_SRL:  result_comb = operand_a_i >> operand_b_i[4:0]; // Shift amount from lower 5 bits of B
            ALU_OP_SRA:  result_comb = $signed(operand_a_i) >>> operand_b_i[4:0]; // Arithmetic shift
            ALU_OP_OR:   result_comb = operand_a_i | operand_b_i;
            ALU_OP_AND:  result_comb = operand_a_i & operand_b_i;
            // Handle cases like LUI (result = imm) or AUIPC (result = PC + imm)
            // These might need specific opcodes or handling outside the main case
            default:     result_comb = 32'b0; // Default case
        endcase
    end

    assign result_o = result_comb;
    assign zero_o = (result_comb == 32'b0); // Set zero flag if result is 0

endmodule
