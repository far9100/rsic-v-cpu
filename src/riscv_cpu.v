// Top-level module for the RISC-V CPU
module riscv_cpu (
    input clk,          // Clock signal
    input rst,          // Reset signal

    // Instruction Memory Interface
    output logic [31:0] instr_addr_o, // Instruction address
    input  logic [31:0] instr_data_i, // Instruction data

    // Data Memory Interface
    output logic [31:0] data_addr_o,  // Data memory address
    output logic [31:0] data_wdata_o, // Data to write
    output logic [3:0]  data_we_o,    // Data write enable (byte level)
    input  logic [31:0] data_rdata_i  // Data read from memory
);

    // --- Signals connecting stages ---

    // IF Stage Outputs -> ID Stage Inputs
    logic [31:0] if_pc;
    logic [31:0] if_pc_plus4;

    // ID Stage Outputs -> EX Stage Inputs
    logic [31:0] id_rs1_data;
    logic [31:0] id_rs2_data;
    logic [31:0] id_imm;
    logic [4:0]  id_rd_addr;
    logic [6:0]  id_opcode;
    logic [2:0]  id_funct3;
    logic [6:0]  id_funct7;
    logic [31:0] id_pc;          // PC value from ID stage
    logic [31:0] id_pc_plus4;

    // Control Unit Signals
    logic        ctrl_alu_src_a_sel;
    logic [1:0]  ctrl_alu_src_b_sel;
    logic [3:0]  ctrl_alu_op;
    logic        ctrl_mem_read;
    logic        ctrl_mem_write;
    // logic [1:0] ctrl_mem_op_size; // Add when control unit outputs it
    logic        ctrl_reg_we;
    logic [1:0]  ctrl_wb_mux_sel;
    logic        ctrl_branch; // Control signal: Is branch?
    logic        ctrl_jump;   // Control signal: Is jump?

    // EX Stage Outputs -> MEM Stage Inputs / IF Stage Inputs
    logic [31:0] ex_alu_result;
    logic        ex_alu_zero;
    logic [31:0] ex_rs2_data;
    logic [4:0]  ex_rd_addr;
    logic [31:0] ex_pc_plus4;
    logic [31:0] ex_branch_target_addr; // Calculated branch target from EX
    logic [31:0] ex_jalr_target_addr;   // Calculated JALR target from EX
    logic        ex_branch_condition_met; // Branch condition result from EX
    // EX Control Signals (Passed through)
    logic        ex_branch; // Pass branch signal through EX (needed?) - Let's use ctrl_branch directly for now
    logic        ex_mem_read;
    logic        ex_mem_write;
    logic [1:0]  ex_mem_op_size;
    logic        ex_reg_we;
    logic [1:0]  ex_wb_mux_sel;


    // MEM Stage Outputs -> WB Stage Inputs
    logic [31:0] mem_rdata;
    logic [31:0] mem_alu_result;
    logic [4:0]  mem_rd_addr;
    logic [31:0] mem_pc_plus4;
    // MEM Control Signals (Passed through)
    logic        mem_reg_we;
    logic [1:0]  mem_wb_mux_sel;

    // WB Stage Outputs -> ID Stage Inputs (Register File Write)
    logic        wb_reg_we;
    logic [4:0]  wb_rd_addr;
    logic [31:0] wb_rd_data;

    // --- Stage Instantiations ---

    // --- PC Update Logic Signals ---
    logic [31:0] jal_target_addr; // Calculated JAL target address (PC + J-imm)
    logic        is_jalr;         // Flag indicating if the current instruction is JALR

    // --- Stage Instantiations ---

    // IF Stage
    if_stage if_stage_inst (
        .clk                    (clk),
        .rst                    (rst),
        .jump_i                 (ctrl_jump),                 // From Control Unit
        .branch_i               (ctrl_branch),               // From Control Unit
        .branch_condition_met_i (ex_branch_condition_met), // From EX Stage
        .jal_target_addr_i      (jal_target_addr),         // Calculated below
        .jalr_target_addr_i     (ex_jalr_target_addr),     // From EX Stage
        .branch_target_addr_i   (ex_branch_target_addr),   // From EX Stage
        .is_jalr_i              (is_jalr),                 // Determined below
        .pc_o                   (if_pc),
        .pc_plus4_o             (if_pc_plus4)
    );

    // ID Stage (includes Register File)
    id_stage id_stage_inst (
        .clk            (clk),
        .rst            (rst),
        .pc_i           (if_pc),        // Pass current PC to ID stage
        .instr_i        (instr_data_i), // Instruction comes from memory
        .pc_plus4_i     (if_pc_plus4),  // PC+4 from IF
        .wb_reg_we_i    (wb_reg_we),    // Write enable from WB
        .wb_rd_addr_i   (wb_rd_addr),   // Write address from WB
        .wb_rd_data_i   (wb_rd_data),   // Write data from WB
        .rs1_data_o     (id_rs1_data),
        .rs2_data_o     (id_rs2_data),
        .imm_o          (id_imm),
        .rd_addr_o      (id_rd_addr),
        .opcode_o       (id_opcode),
        .funct3_o       (id_funct3),
        .funct7_o       (id_funct7),
        .pc_o           (id_pc),        // Output PC from ID stage
        .pc_plus4_o     (id_pc_plus4)
        // Control signal outputs need to be added and connected
        // Note: Control unit is placed after ID stage as it depends on opcode from ID
    );

    // Control Unit
    control_unit control_unit_inst (
        .opcode_i       (id_opcode),
        .funct3_i       (id_funct3),     // Connect funct3 from ID stage
        .funct7_b5_i    (id_funct7[5]),  // Connect funct7[5] from ID stage
        .alu_src_a_sel_o(ctrl_alu_src_a_sel),
        .alu_src_b_sel_o(ctrl_alu_src_b_sel),
        .alu_op_o       (ctrl_alu_op),
        .mem_read_o     (ctrl_mem_read),
        .mem_write_o    (ctrl_mem_write),
        // .mem_op_size_o(ctrl_mem_op_size),
        .reg_we_o       (ctrl_reg_we),
        .wb_mux_sel_o   (ctrl_wb_mux_sel),
        .branch_o       (ctrl_branch), // Connect branch signal output
        .jump_o         (ctrl_jump)    // Connect jump signal output
    );

    // Calculate JAL target address and is_jalr flag
    // JAL target = PC + J-immediate (id_imm contains J-imm when opcode is JAL)
    assign jal_target_addr = if_pc + id_imm;
    assign is_jalr = (id_opcode == 7'b1100111); // Check if opcode is JALR

    // EX Stage (includes ALU)
    ex_stage ex_stage_inst (
        .clk            (clk),
        .rst            (rst),
        .pc_i           (id_pc),        // Connect PC from ID stage
        .rs1_data_i     (id_rs1_data),
        .rs2_data_i     (id_rs2_data),
        .imm_i          (id_imm),
        .rd_addr_i      (id_rd_addr),
        .pc_plus4_i     (id_pc_plus4),
        .alu_src_a_sel_i(ctrl_alu_src_a_sel), // From Control Unit
        .alu_src_b_sel_i(ctrl_alu_src_b_sel), // From Control Unit
        .alu_op_i       (ctrl_alu_op),       // From Control Unit
        .funct3_i       (id_funct3),         // Pass funct3 for branch evaluation
        .branch_i       (ctrl_branch),       // Pass branch control signal
        .alu_result_o   (ex_alu_result),
        .alu_zero_o     (ex_alu_zero),
        .rs2_data_o     (ex_rs2_data),
        .rd_addr_o      (ex_rd_addr),
        .pc_plus4_o     (ex_pc_plus4),
        .branch_target_addr_o (ex_branch_target_addr), // Output for IF stage
        .jalr_target_addr_o   (ex_jalr_target_addr),   // Output for IF stage
        .branch_condition_met_o(ex_branch_condition_met) // Output for IF stage
        // Pass through control signals
        // .mem_read_o     (ex_mem_read), // Need to add these pass-throughs in ex_stage.v
        // .mem_write_o    (ex_mem_write),
        // .mem_op_size_o  (ex_mem_op_size),
        // .reg_we_o       (ex_reg_we),
        // .wb_mux_sel_o   (ex_wb_mux_sel)
    );
    // Temporary pass-through for control signals until added to EX stage ports
    // Note: In a pipelined design, these would be registered in ID/EX register
    assign ex_mem_read = ctrl_mem_read;
    assign ex_mem_write = ctrl_mem_write;
    // assign ex_mem_op_size = ctrl_mem_op_size; // Assign when available
    assign ex_reg_we = ctrl_reg_we;
    assign ex_wb_mux_sel = ctrl_wb_mux_sel;


    // MEM Stage
    mem_stage mem_stage_inst (
        .clk            (clk),
        .rst            (rst),
        .alu_result_i   (ex_alu_result),
        .rs2_data_i     (ex_rs2_data),
        .rd_addr_i      (ex_rd_addr),
        .pc_plus4_i     (ex_pc_plus4),
        .mem_read_i     (ex_mem_read),     // From Control Unit (passed via EX)
        .mem_write_i    (ex_mem_write),    // From Control Unit (passed via EX)
        // .mem_op_size_i  (ex_mem_op_size), // From Control Unit (passed via EX) - Add later
        .data_addr_o    (data_addr_o),     // Connect to top-level memory interface
        .data_wdata_o   (data_wdata_o),    // Connect to top-level memory interface
        .data_we_o      (data_we_o),       // Connect to top-level memory interface
        .data_rdata_i   (data_rdata_i),    // Connect to top-level memory interface
        .mem_rdata_o    (mem_rdata),
        .alu_result_o   (mem_alu_result),
        .rd_addr_o      (mem_rd_addr),
        .pc_plus4_o     (mem_pc_plus4)
        // Pass through control signals
        // .reg_we_o       (mem_reg_we), // Need to add these pass-throughs in mem_stage.v
        // .wb_mux_sel_o   (mem_wb_mux_sel)
    );
    // Temporary pass-through for control signals until added to MEM stage ports
    // Note: In a pipelined design, these would be registered in EX/MEM register
    assign mem_reg_we = ex_reg_we;         // From Control Unit (passed via EX)
    assign mem_wb_mux_sel = ex_wb_mux_sel; // From Control Unit (passed via EX)

    // WB Stage
    wb_stage wb_stage_inst (
        .clk            (clk),
        .rst            (rst),
        .mem_rdata_i    (mem_rdata),
        .alu_result_i   (mem_alu_result),
        .rd_addr_i      (mem_rd_addr),
        .pc_plus4_i     (mem_pc_plus4),
        .reg_we_i       (mem_reg_we),      // From Control Unit (passed via EX, MEM)
        .wb_mux_sel_i   (mem_wb_mux_sel),  // From Control Unit (passed via EX, MEM)
        .wb_reg_we_o    (wb_reg_we),       // To ID stage (Reg File)
        .wb_rd_addr_o   (wb_rd_addr),      // To ID stage (Reg File)
        .wb_rd_data_o   (wb_rd_data)       // To ID stage (Reg File)
    );

    // Connect IF stage PC output to instruction memory address
    assign instr_addr_o = if_pc;

endmodule
